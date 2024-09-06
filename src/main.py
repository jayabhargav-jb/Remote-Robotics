from fastapi import FastAPI
import os
from fastapi.staticfiles import StaticFiles

app = FastAPI()

app.mount("/static", StaticFiles(directory="/srv/http"), name="static")

# @app.get("/")
# def read_root():
#     return {"Hello": "World"}

@app.post("/items/")
def create_item(item: str):
    return {"item": item}
