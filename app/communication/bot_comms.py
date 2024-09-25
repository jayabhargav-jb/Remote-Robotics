import requests
from requests import Response

# Bot IP Address constants
BOT_ROS = "192.168.0.104:8082"
BOT_IOT = "192.168.1.15"


def alert_bot(bot: str, file_path: str) -> bool:
    """
    Function to alert bot to get the latest code file from the server

    Makes a POST request to the bot
    """

    url: str = f"http://{bot}/upload"
    files: dict = {"file": open(file_path, "rb")}
    response: Response = requests.post(url, files=files)
    files["file"].close()

    if response.status_code == 200:
        print("File sent successfully.")
        return True
    else:
        print(f"Failed to send file. Status code: {response.status_code}")
        return False
