from typing import Annotated
from fastapi import APIRouter, HTTPException, Depends, status

from datetime import datetime, timedelta, timezone

from ..database import operations as ds
from ..core.schema import Token, TokenData, User, UserInDB

from ..core.core import get_current_active_user

router = APIRouter()


@router.get(
    "/timeslot",
    responses={
        200: {"description": "Get all the timeslots"},
        401: {"description": "Not Authorized"},
    },
)
async def get_timeslots(
    current_user: Annotated[User, Depends(get_current_active_user)]
) -> list[User] | None:
    if current_user.username == "root":
        users: list[User] = ds.get_users()
        # Removing the root entry
        users.pop(0)
        return users
    else:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not Authorized",
            headers={"WWW-Authenticate": "Bearer"},
        )


@router.get(
    "/timeslot/allot",
    responses={
        200: {"description": "Allot OK"},
        401: {
            "description": "User Not Authorized to allot timeslot. Only root user permitted"
        },
        404: {"description": "User not found"},
    },
)
async def set_timeslot(
    username: str,
    start_time: str,
    end_time: str,
    current_user: Annotated[User, Depends(get_current_active_user)],
) -> User:
    """Allot a timeslot to the user
    Only root user is authorized to allot timeslots
    """
    if current_user.username == "root":
        user: User = ds.get_user(username)
        if user:
            ds.allot_timeslot(username, start_time, end_time)
            return ds.get_user(username)
        else:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found",
                headers={"WWW-Authenticate": "Bearer"},
            )
    else:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not Authorized",
            headers={"WWW-Authenticate": "Bearer"},
        )
