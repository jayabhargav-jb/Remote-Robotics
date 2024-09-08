from datetime import datetime
from pydantic import BaseModel

class Token(BaseModel):
    access_token: str
    token_type: str


class TokenData(BaseModel):
    username: str


class User(BaseModel):
    username: str
    blacklist: bool = False
    disabled : bool = False
    start_time: str
    end_time: str


class UserInDB(User):
    hashed_password: str