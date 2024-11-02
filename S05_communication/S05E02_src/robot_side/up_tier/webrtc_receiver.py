import asyncio
import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
from av import VideoFrame
from datetime import datetime, timedelta
import matplotlib.pyplot as plt

"""
This class is only for testing purpose, 
because in the real scenario, the webrtc_sender will send the video stream upto the remote stream server, 
say, Simple Realtime Server (SRS). 
To run the test, open 2 CLI terminals,
In the first terminal, run `$ python3 webrtc_sender.py`,
In the second terminal, run `$ python3 webrtc_receiver.py`.
Then you will see an openCV window popping up, and display the webcam stream. 
"""
class VideoReceiver:
    def __init__(self):
        self.track = None

        self.fig, self.ax = plt.subplots()
        self.imshow = self.ax.imshow(np.zeros([480, 640, 3], dtype=float))
        plt.ion()


    async def handle_track(self, track):
        print("Inside handle track")
        self.track = track
        frame_count = 0

        while True:
            try:
                print("Waiting for frame...")
                frame = await asyncio.wait_for(track.recv(), timeout=5.0)
                frame_count += 1
                print(f"Received frame {frame_count}")
                
                if isinstance(frame, VideoFrame):
                    print(f"Frame type: VideoFrame, pts: {frame.pts}, time_base: {frame.time_base}")
                    frame = frame.to_ndarray(format="bgr24")
                elif isinstance(frame, np.ndarray):
                    print(f"Frame type: numpy array")
                else:
                    print(f"Unexpected frame type: {type(frame)}")
                    continue
                
                # print(f"Frame shape: {frame.shape}")
                
                 # Add timestamp to the frame
                current_time = datetime.now()
                new_time = current_time - timedelta( seconds=55)
                timestamp = new_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                # timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Current time with milliseconds
                # cv2.putText(frame, ti/mestamp, (frame.shape[1] - 300, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, timestamp, (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                cv2.imwrite(f"imgs/received_frame_{frame_count}.jpg", frame)
                print(f"Saved frame {frame_count} to file")
                
                # cv2.imshow is broken when importing av
                # cv2.imshow("Webcam", frame)  

                print(f"\n type(frame): {type(frame)}, frame.shape: {frame.shape} \n")
                self.imshow.set_data(frame)
                plt.pause(0.01)

                # Exit on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    button = plt.waitforbuttonpress()
                    print(f"button: {button}")

                    plt.ioff()
                    break
                # if frame_count >= 300:
                #     break
            except asyncio.TimeoutError:
                print("Timeout waiting for frame, continuing...")
            except Exception as e:
                print(f"Error in handle_track: {str(e)}")
                if "Connection" in str(e):
                    break
        print("Exiting handle_track")


async def run(pc, signaling):
    await signaling.connect()

    @pc.on("track")
    def on_track(track):
        if isinstance(track, MediaStreamTrack):
            print(f"Receiving {track.kind} track")
            asyncio.ensure_future(video_receiver.handle_track(track))

    @pc.on("datachannel")
    def on_datachannel(channel):
        print(f"Data channel established: {channel.label}")

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"Connection state is {pc.connectionState}")
        if pc.connectionState == "connected":
            print("WebRTC connection established successfully")

    print("Waiting for offer from sender...")
    offer = await signaling.receive()
    print("Offer received")
    await pc.setRemoteDescription(offer)
    print("Remote description set")

    answer = await pc.createAnswer()
    print("Answer created")
    await pc.setLocalDescription(answer)
    print("Local description set")

    await signaling.send(pc.localDescription)
    print("Answer sent to sender")

    print("Waiting for connection to be established...")
    while pc.connectionState != "connected":
        await asyncio.sleep(0.1)

    print("Connection established, waiting for frames...")
    await asyncio.sleep(100)  # Wait for 35 seconds to receive frames

    print("Closing connection")

async def main():
    # signaling = TcpSocketSignaling("192.168.30.40", 9999)
    signaling = TcpSocketSignaling("127.0.0.1", 9999)
    pc = RTCPeerConnection()
    
    global video_receiver
    video_receiver = VideoReceiver()

    try:
        await run(pc, signaling)
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        print("Closing peer connection")
        await pc.close()

if __name__ == "__main__":
    asyncio.run(main())


    

# Learn how to stream camera frames in real-time from one machine to another using WebRTC and Python. This repo walks you through setting up WebRTC with Python, capturing video with OpenCV, and establishing peer-to-peer connections