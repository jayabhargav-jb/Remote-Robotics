cd /rero/ros_bot

. ./venv/bin/activate

uvicorn app.main:app --host 0.0.0.0 --port 8081 --reload