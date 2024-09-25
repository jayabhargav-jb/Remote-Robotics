import socket
import json

HOST = '192.168.0.183'
PORT = 9090  # ROSBridge typically runs on port 9090

def connect_to_rosbridge(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"Connecting to {host}:{port}...")
        s.connect((host, port))
        print(f"Connected to {host}:{port}")

        # Subscribe to the LaserScan topic
        subscribe_message = json.dumps({
            "op": "subscribe",
            "topic": "/scan"  # Adjust the topic name if needed
        }) + "\n"
        
        s.sendall(subscribe_message.encode('utf-8'))

        while True:
            response = s.recv(4096)
            if not response:
                break

            # Decode the JSON message
            try:
                messages = response.decode('utf-8').strip().split('\n')
                for message in messages:
                    if message:
                        parsed_message = json.loads(message)
                        if parsed_message.get("op") == "publish" and parsed_message["topic"] == "/scan":
                            laserscan_data = parsed_message["msg"]
                            print("Received LaserScan data:")
                            print(laserscan_data)
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON: {e}")

if __name__ == "__main__":
    connect_to_rosbridge(HOST, PORT)
