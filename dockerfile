FROM ubuntu:latest

# Update and install the necessary packages
RUN apt update -y && apt upgrade -y && apt install -y python3 python3-pip python3-venv git python3-full

# Create the python virtual environment
RUN python3 -m venv /opt/.venv
WORKDIR /opt/.venv
RUN . ./bin/activate

# Install requirements
COPY ./requirements.txt /src/requirements.txt
RUN pip install -r /src/requirements.txt --break-system-packages

# Make a http folder for the static html files
RUN mkdir /srv/http
WORKDIR /srv/http

# Only for development purposes allow acces via volumes

# Maintain the folder structure as index.html followed by folders for css & js
# COPY ./http/ /srv/http/
VOLUME [ "/srv/http" ]

WORKDIR /src/
# COPY ./src /src/ # TODO: Uncomment this line
VOLUME [ "/src" ]

# Expose port 8080 for user interface
EXPOSE 8080

# Export port 8081 for the API connect
EXPOSE 8081

# Replace this with the command to run the server
CMD ["bash"]
# CMD [ "uvicorn",  "main:app", "--host", "0.0.0.0", "--port", "8080"]