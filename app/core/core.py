from ..database import operations as ds

from datetime import datetime, timedelta, timezone, date

from fastapi import APIRouter, HTTPException, Depends, status, FastAPI
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from fastapi.staticfiles import StaticFiles

import jwt
from jwt.exceptions import InvalidTokenError

import pytz
from passlib.context import CryptContext
from pydantic import BaseModel

from .schema import Token, TokenData, User, UserInDB
import sqlite3

from typing import Annotated

router = APIRouter(
    # prefix="/core",
)

with open("/etc/secret") as f:
    global SECRET_KEY
    SECRET_KEY = f.readline().strip()

ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password):
    return pwd_context.hash(password)


def get_user(username: str):
    user_dict = ds.get_user_in_db(username)
    return UserInDB(**user_dict.dict())


def authenticate_user(username: str, password: str):
    user: UserInDB = ds.get_user_in_db(username)
    if not user:
        return False
    if not verify_password(password, user.hashed_password):
        return False
    print("Returning user")
    return user


def create_access_token(data: dict, expires_delta: timedelta | None = None):
    to_encode: dict = data.copy()
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
    print(current_user, type(current_user))
    if current_user.disabled:
        raise HTTPException(status_code=400, detail="Inactive user")
    return current_user


@router.post("/token")
async def login_for_access_token(
    form_data: Annotated[OAuth2PasswordRequestForm, Depends()], response_model=None
):

    user = authenticate_user(form_data.username, form_data.password)

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
            status_code=201,
            detail="{'message': 'Wait your turn', 'timeslot_start': user.start_time, 'timeslot_end': user.end_time}",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.username}, expires_delta=access_token_expires
    )
    return Token(access_token=access_token, token_type="bearer")


@router.post("/adduser/{username}")
async def add_user(
    current_user: Annotated[User, Depends(get_current_active_user)], user: UserInDB
) -> User:
    if current_user.username == "root":

        # Security feature to not allow the root to change the password on entry
        user.hashed_password = get_password_hash(user.username)
        try:
            ds.add_user(user)
            return ds.get_user_in_db(user.username)
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


@router.post("/password/set")
async def set_password(
    current_user: Annotated[User, Depends(get_current_active_user)],
    username: str,
    password: str,
    date_of_birth: date,
) -> User:
    """
    Allow user to set password

    param: User, password, date_of_birth
    return:

        success: username
        failure: HTTPException (401, 500)

    """

    if username == "root":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not Allowed to change the root password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Root user with the date of birth
    if (current_user == "root") and (
        ds.get_user(username).date_of_birth == date_of_birth
    ):
        user: User = ds.get_user(username)
        flag: User | None = ds.change_password(
            user.username, get_password_hash(password)
        )
        if flag:
            return flag
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Password change failed",
                headers={"WWW-Authenticate": "Bearer"},
            )

    # Non-root user with the same username and date of birth
    elif (
        (current_user != "root")
        and (current_user.username == user.username)
        and (user.date_of_birth == date_of_birth)
    ):
        user: User = ds.get_user(username)
        flag: User | None = ds.change_password(
            user.username, get_password_hash(password)
        )
        if flag:
            return flag
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Password change failed",
                headers={"WWW-Authenticate": "Bearer"},
            )

    else:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not Authorized",
            headers={"WWW-Authenticate": "Bearer"},
        )

@router.get("/is_auth")
async def is_authenticated(current_user: Annotated[User, Depends(get_current_active_user)]):
    """Get the authenticated user, and authentication status
    
    @return
        {
            "is_authenticated": authentication status,
            "username": username
        }

    """
    return {"is_authenticated": True, "username": current_user.username}