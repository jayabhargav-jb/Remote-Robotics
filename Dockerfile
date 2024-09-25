FROM ubuntu:22.04

ENV TZ="Asia/Kolkata"
RUN date

# Set timezone:
RUN ln -snf /usr/share/zoneinfo/$CONTAINER_TIMEZONE /etc/localtime && echo $CONTAINER_TIMEZONE > /etc/timezone

# Update and install the necessary packages
RUN apt update -y && apt upgrade -y && apt install -y python3 python3-pip python3-venv git python3-full

# Create the python virtual environment
RUN python3 -m venv /bot_comm
RUN . /bot_comm/bin/activate


VOLUME [ "/bot_comm/src" ]

# Install requirements
# Despite the use of volume, this is required as volumes are set up towards the end of the container setup
COPY ./server/requirements.txt /bot_comm/requirements.txt
RUN python3 -m pip install -r /bot_comm/requirements.txt

# Create directories for storing the files
RUN mkdir /tmp/ros


# Expose port 8080 for user interface
EXPOSE 8080

# Export port 8081 for the websocket
EXPOSE 8081

# TODO: Replace this with the command to run the server
CMD ["/bin/bash"]
# CMD [ "uvicorn",  "app.main:app", "--host", "0.0.0.0", "--port", "8080"]