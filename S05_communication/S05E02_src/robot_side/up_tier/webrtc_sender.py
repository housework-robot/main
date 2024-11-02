import asyncio
import cv2
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
from av import VideoFrame
import fractions
from datetime import datetime

class CustomVideoStreamTrack(VideoStreamTrack):
    def __init__(self, camera_id):
        super().__init__()
        self.cap = cv2.VideoCapture(camera_id)
        self.frame_count = 0

    async def recv(self):
        self.frame_count += 1
        print(f"Sending frame {self.frame_count}")
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to read frame from camera")
            return None
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = self.frame_count
        video_frame.time_base = fractions.Fraction(1, 30)  # Use fractions for time_base
        # Add timestamp to the frame
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Current time with milliseconds
        cv2.putText(frame, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = self.frame_count
        video_frame.time_base = fractions.Fraction(1, 30)  # Use fractions for time_base
        
        return video_frame


class WebrtcSender:
    def __init__(self):
        self.ip_addr = "127.0.0.1"
        self.port = 9999
        self.camera_id = -1  # Change this to the appropriate camera ID

        self.signaling = TcpSocketSignaling(self.ip_addr, self.port)
        self.pc = RTCPeerConnection()
        self.video_sender = CustomVideoStreamTrack(self.camera_id)


    def run(self):
        asyncio.run(self.run_webrtc()) 


    async def run_webrtc(self):
        self.pc.addTrack(self.video_sender)

        try:
            await self.signaling.connect()

            @self.pc.on("datachannel")
            def on_datachannel(channel):
                print(f"Data channel established: {channel.label}")

            @self.pc.on("connectionstatechange")
            async def on_connectionstatechange():
                print(f"Connection state is {self.pc.connectionState}")
                if self.pc.connectionState == "connected":
                    print("WebRTC connection established successfully")

            offer = await self.pc.createOffer()
            await self.pc.setLocalDescription(offer)
            await self.signaling.send(self.pc.localDescription)

            while True:
                obj = await self.signaling.receive()
                if isinstance(obj, RTCSessionDescription):
                    await self.pc.setRemoteDescription(obj)
                    print("Remote description set")
                elif obj is None:
                    print("Signaling ended")
                    break
            print("Closing connection")
        finally:
            await self.pc.close()


def testrun_webrtc():
    webrtc_sender = WebrtcSender()
    webrtc_sender.run()


if __name__ == "__main__":
    testrun_webrtc()

