from typing import Annotated

from fastapi import APIRouter, HTTPException, Depends

from ..core.schemas import User, UserInDB
from ..core.core import get_current_user, get_current_active_user
from fastapi.responses import FileResponse

router = APIRouter()

@router.get("/")
async def serve_root():
    try:
        current_user = await get_current_active_user(get_current_user)

        # Send the /static/index.html file to the user
        return FileResponse("/srv/http/index.html")
      
    except Exception as e:

        print(e)

        # If user not logged in, redirect to login page
        return FileResponse("/srv/http/login.html")

# @router.get("/home")