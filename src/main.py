from datetime import datetime, timedelta, timezone

from fastapi import Depends, FastAPI, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm

import jwt
from jwt.exceptions import InvalidTokenError

from models.user import Token, TokenData, User, UserInDB
import database_sentinel as ds

import pytz
import sqlite3

from passlib.context import CryptContext
from pydantic import BaseModel
from typing import Annotated

with open("/etc/secret") as f:
    global SECRET_KEY
    SECRET_KEY = f.readline().strip()

ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

app = FastAPI()
ds.init()


def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password):
    return pwd_context.hash(password)


def get_user(username: str):
    user_dict = ds.get_user_in_db(username)
    return UserInDB(**user_dict.dict())


def authenticate_user(username: str, password: str):
    user = ds.get_user_in_db(username)
    if not user:
        return False
    if not verify_password(password, user.hashed_password):
        return False
    print("Returning user")
    return user


def create_access_token(data: dict, expires_delta: timedelta | None = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


async def get_current_user(token: Annotated[str, Depends(oauth2_scheme)]):
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        username: str = payload.get("sub")
        if username is None:
            raise credentials_exception
        token_data = TokenData(username=username)
    except InvalidTokenError:
        raise credentials_exception
    user = get_user(username=token_data.username)
    if user is None:
        raise credentials_exception
    return user


async def get_current_active_user(
    current_user: Annotated[User, Depends(get_current_user)],
):
    if current_user.disabled:
        raise HTTPException(status_code=400, detail="Inactive user")
    return current_user


@app.post("/token")
async def login_for_access_token(
    form_data: Annotated[OAuth2PasswordRequestForm, Depends()], response_model=None
):
    user = authenticate_user(form_data.username, form_data.password)
    # print(user)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    elif user.disabled or user.blacklist:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User Disabled",
            headers={"WWW-Authenticate": "Bearer"},
        )
    elif (user.username != "root") and (
        (
            int(user.start_time) > int(datetime.now().strftime("%y%m%d%H%M%S"))
            or int(user.end_time) < int(datetime.now().strftime("%y%m%d%H%M%S"))
        )
    ):

        raise HTTPException(
            status_code=307,
            detail="Wait your turn",
            headers={"WWW-Authenticate": "Bearer", "Location": "/wait.html"},
        )

    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.username}, expires_delta=access_token_expires
    )
    return Token(access_token=access_token, token_type="bearer")


# TODO: Remove this test method
@app.get("/users/me/", response_model=User)
async def read_users_me(
    current_user: Annotated[User, Depends(get_current_active_user)],
):
    return current_user


# TODO: Remove this test method
@app.get("/users/me/items/")
async def read_own_items(
    current_user: Annotated[User, Depends(get_current_active_user)],
):
    return [{"item_id": "Foo", "owner": current_user.username}]


@app.post("/adduser/{username}")
async def add_user(
    current_user: Annotated[User, Depends(get_current_active_user)], user: UserInDB
):
    if current_user.username == "root":
        # Security feature to not allow the root to change the password on entry
        user.hashed_password = get_password_hash(user.username)
        try:
            ds.add_user(user)
            return ds.get_user(user.username)
        except sqlite3.IntegrityError:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Duplicate username",
                headers={"WWW-Authenticate": "Bearer"},
            )
    else:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not Authorized",
            headers={"WWW-Authenticate": "Bearer"},
        )
