# Author: Ujwal N K
# Date: 2024-01-25
# Communication from the server to the bots

from typing import Annotated
from fastapi import APIRouter, HTTPException, status

import requests
from requests import Response

# Bot IP Address constants
IP_ROS_BOT = "192.168.0.104:8081"
IP_IOT_BOT = "localhost:8082"

router = APIRouter()

def alert_bot(bot: str, file_path: str) -> bool:
    """
    Function to alert bot to get the latest code file from the server

    Makes a GET request to the bot
    """

    url: str = f"http://{bot}/alert" # http://192.168.0.104:8081/get_code -> ros-bot

    try:
        response: Response = requests.get(url)
        response.raise_for_status()
        return True
    except requests.RequestException as e:
        print(f"An error occurred: {e}")
        return False
    

@router.get("/get_code/iot", status_code=status.HTTP_200_OK)
def send_code_to_bot():
    """
    Endpoint to send the code file to the bot
    """
    try:
        with open("/tmp/iot/iot_bot.code", 'rb') as file:
            content = file.read()
        return {"file_content": content.decode('utf-8')}
    except FileNotFoundError:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="File not found")
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=str(e))
