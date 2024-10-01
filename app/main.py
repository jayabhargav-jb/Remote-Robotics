from fastapi import FastAPI, HTTPException
import aiohttp
import aiofiles
import requests
from fastapi import UploadFile, File

app = FastAPI()

@app.post("/push_code")
async def save_file(file: UploadFile = File(...)):
    async with aiofiles.open(f'/tmp/{file.filename}', 'wb') as out_file:
        while content := await file.read(1024):  # Read file in chunks
            await out_file.write(content)
    return {"detail": "File saved successfully"}
