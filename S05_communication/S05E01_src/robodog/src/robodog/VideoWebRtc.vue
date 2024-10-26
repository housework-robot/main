<template>
  <video
    id="video"
    controls
    autoplay
    muted
    width="100%"
    height="100%"
    style="object-fit: fill"
  ></video>
</template>
<script>
// 引入webrtcstreamer.js，确保路径正确
import WebRtcStreamer from "./webrtcstreamer.js";
export default {
  name: "VideoWebRtc",
  components: {},
  data() {
    return {
      webRtcServer: null,
      camera_ip: "127.0.0.1:8000", //这里看自己的需要，也可以传入另一台电脑的ip，前提是都得在在一个局域网内
    };
  },
  mounted() {
    //video：需要绑定的video控件ID
    //127.0.0.1:8000：启动webrtc-streamer的设备IP和端口，默认8000
    this.webRtcServer = new WebRtcStreamer(
      "video",
      location.protocol + "//" + this.camera_ip
    );
    // //需要查看的rtsp地址,根据自己的摄像头传入对应的rtsp地址即可。注意：视频编码格式必须是H264的，否则无法正常显示，编码格式可在摄像头的后台更改
    this.webRtcServer.connect("rtsp://127.0.0.1:10054/hls/test");
  },
};
</script>
