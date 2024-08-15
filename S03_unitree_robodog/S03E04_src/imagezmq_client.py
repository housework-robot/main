# imagezmq_client.py
# run this program on the client to send a labelled image stream

import socket
import time
import imagezmq
import cv2

# You can change the 'localhost' to any remote server's IP address, either in local area network or internet
sender = imagezmq.ImageSender(connect_to='tcp://localhost:5555')

# hostname is equivalent to the CLI: echo $HOSTNAME
host_name = socket.gethostname() 
cap = cv2.VideoCapture(0)
while True: 
    ret, frame = cap.read()
    if ret:
        encoded, buf = cv2.imencode('.jpg', frame)
        sender.send_image(host_name, frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
sender.close()
