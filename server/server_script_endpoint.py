from fastapi import FastAPI, UploadFile

app = FastAPI()

@app.post("/upload")
async def get_file(file: UploadFile):
    try:
        file_path = "/tmp/ros/ros_code.py"
        with open(file_path, "w") as f:
            f.write(await file.read())
    except:
        print("failed")