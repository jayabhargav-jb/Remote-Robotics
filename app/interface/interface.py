from typing import Annotated

from fastapi import APIRouter, HTTPException, Depends

from ..core.schemas import User, UserInDB
from ..core.core import get_current_user, get_current_active_user

router = APIRouter()

@router.get("/")
async def serve_root():
    try:
        current_user = get_current_active_user(get_current_user)

        # If user logged in redirect to index.html
        raise HTTPException(
            status_code=307,
            detail="Redirecting to homepage",
            headers={"WWW-Authenticate": "Bearer", "Location": "/static/index.html"},
        )
    except HTTPException:

        # If user not logged in, redirect to login page
        raise HTTPException(
            status_code=307,
            detail="Please login",
            headers={"WWW-Authenticate": "Bearer", "Location": "static/login.html"},
        )

