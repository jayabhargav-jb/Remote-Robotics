from typing import Annotated
from fastapi import APIRouter, HTTPException, Depends, status, UploadFile

from datetime import datetime, timedelta, timezone

from ..database import operations as ds
from ..core.schemas import Token, TokenData, User, UserInDB

from ..core.core import get_current_active_user

router = APIRouter(
    prefix="/file"
)

@router.post("/post/iot")
async def push_code(
    current_user: Annotated[User, Depends(get_current_active_user)],
    file: UploadFile
):
    file_path = f"/tmp/iot/{file.filename}"
    with open(file_path, "wb") as f:
        f.write(await file.read())


@router.post("/post/ros")
async def push_code(
    current_user: Annotated[User, Depends(get_current_active_user)],
    file: UploadFile
):
    file_path = f"/tmp/ros/{file.filename}"
    with open(file_path, "wb") as f:
        f.write(await file.read())
