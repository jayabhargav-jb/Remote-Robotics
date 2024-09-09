from fastapi import FastAPI

from .core import core

app = FastAPI()

app.include_router(core.router)

