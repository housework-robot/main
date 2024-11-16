On the robot side, we developed a system consisting of 3 tiers, up_tier, core_tier and down_tier.

1. The up_tier provides shareable services for various robots, like wireless communication.

2. The down_tier provide specific services for specific robots, so far we provides motion control, video capture, and audio playing services for unitree's Go2 robotic dog.

3. The core_tier behaves as an workflow and dataflow organizer.

For example, `core_tier/go2_core.py` reads video stream from `down_tier/unitree_go2/go2_media.py` which is specific for go2 robotic dog, and sends the stream to `up_tier/rtmp_sender.py` which is shareable among various robots.
