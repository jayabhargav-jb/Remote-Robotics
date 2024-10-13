cd /rero/ros_bot

source "/opt/ros/humble/setup.bash"
# . ./venv/bin/activate


uvicorn app.main:app --host 0.0.0.0 --port 8081 --reload