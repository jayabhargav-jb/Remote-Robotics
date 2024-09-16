from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from .core import core
from .interface import interface

app = FastAPI()

app.include_router(core.router)
app.include_router(interface.router)

# app.mount("/static", StaticFiles(directory="/srv/http"), name="static")


