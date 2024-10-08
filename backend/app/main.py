from fastapi import FastAPI

from .core import core
from .timeslot import timeslot_manager
from .database import operations
from .communication import code_comms
from .communication import bot_comms

operations.init()

app = FastAPI()

app.include_router(core.router)
app.include_router(timeslot_manager.router)
app.include_router(code_comms.router)
app.include_router(bot_comms.router)

