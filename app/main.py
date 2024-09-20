from fastapi import FastAPI

from .core import core
from .timeslot import timeslot_manager
from .database import operations
# from .communication import upload_manager

operations.init()

app = FastAPI()

app.include_router(core.router)
app.include_router(timeslot_manager.router)
# app.include_router(upload_manager.router)

