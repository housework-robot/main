# imagezmq_server.py, 08/25/2024
# run this program on the server to display image streams from multiple clients

import cv2
import imagezmq

image_hub = imagezmq.ImageHub()
while True:  # show streamed images until Ctrl-C
    host_name, image = image_hub.recv_image()
    print(f" rpi_name: '{host_name}' \n")
    cv2.imshow(host_name, image) # 1 window for each client
    cv2.waitKey(1)
    image_hub.send_reply(b'OK')

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

image_hub.close() 
cv2.destroyAllWindows()
