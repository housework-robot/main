import websocket
import time

# ws_address = "ws://192.168.0.115:81"
ws_address = "ws://192.168.0.115"
print(f"[INFO] WebSocket server URL is: '{ws_address}'\n")

# Connect to WebSocket server
ws = websocket.WebSocket()
ws.connect(ws_address)
print(f"[INFO] Connected to WebSocket server: {ws_address}\n")

# Ask the user for some input and transmit it
str = input(">> Say something: ")
ws.send(str)

# Wait for server to respond and print it
result = ws.recv()
print("Received: " + result)

# Sleep for 10 seconds, then gracefully close WebSocket connection.
time.sleep(10)
ws.close()