<template>
  <div>
    <div><el-input></el-input></div>
    <video
      id="video"
      v-if="easyPlayerFlag == 'video'"
      controls
      autoplay
      muted
      loop
      crossOrign="anonymous"
      width="100%"
      height="100%"
      style="object-fit: fill"
    ></video>
    <easy-player
      ref="eaplayer"
      v-if="easyPlayerFlag == 'eaplayer'"
      :video-url="videoUri"
      :live="true"
      :stretch="false"
      :video-title="''"
      style="width: 100%; height: 85vh"
    ></easy-player>
    <!-- <EasyPlayerPro
      ref="eaplayerpro"
      v-if="easyPlayerFlag == 'eaplayerpro'"
    ></EasyPlayerPro> -->
    <!-- frameborder属性：不设置边框，如果是vue2.x版本，那么可以使用src属性  -->
    <!-- <iframe
      style="width: 100%; height: 800px"
      id="tempHtml"
      ref="tempHtml"
      frameborder="0"
      v-if="easyPlayerFlag == 'eaplayerpro'"
    ></iframe> -->
    <!-- <div v-html="demoHtml"></div> -->

    <webrtc-player :videoSrc="videoSrc" v-if="easyPlayerFlag == 'webrtcplayer'"></webrtc-player>
  </div>
</template>
<script>
// 引入webrtcstreamer.js，确保路径正确
import WebRtcStreamer from "./webrtcstreamer.js";
import EasyPlayer from "@easydarwin/easyplayer";
// import demoHtml from '../components/easyplayerpro/demo.html';
// import EasyPlayerPro from "../components/easyplayerpro/EasyPlayerPro.vue";
import WebrtcPlayer from "../components/jswebrtc/webrtcPlayer.vue";

export default {
  name: "VideoWebRtc",
  components: { EasyPlayer,WebrtcPlayer
//    EasyPlayerPro
    },
  data() {/*  */
    return {
      easyPlayerFlag: "webrtcplayer",//video eaplayer eaplayerpro webrtcplayer
      webRtcServer: null,
      camera_ip: "127.0.0.1:8000", //这里看自己的需要，也可以传入另一台电脑的ip，前提是都得在在一个局域网内
      //   videoUri:
      //     "http://127.0.0.1:8000/easyPlayerPro/live/lebtest?txSecret=f22a813b284137ed10d3259a7b5c224b&txTime=69f1eb8c&tabr_bitrates=d1080p,d540p,d360p&tabr_start_bitrate=d1080p",
      videoUri:
        "rtsp://127.0.0.1:10054/trailer",
      ifeData: "",
      // videoSrc: "webrtc://192.168.0.158/live/livestream",
      videoSrc: "webrtc://127.0.0.1:1935/live/livestream",
    };
  },
  mounted() {
    console.log("location.protocol", location.protocol);
    if (this.easyPlayerFlag == "eaplayer") {
      this.easyPlayer();
      this.$nextTick(() => {
        this.videoUri = this.src();
      });
    } else if (this.easyPlayerFlag == "video") {
      this.webrtcStreamerPlay();
    } else if (this.easyPlayerFlag == "eaplayerpro") {
      this.$refs.tempHtml.src = this.ifeData;
    } else if (this.easyPlayerFlag == "webrtcPlayer") {
      this.videoSrc = "";
    }
  },
  destroyed() {
    if (this.webRtcServer) {
      this.webRtcServer.disconnect();
    }
  },
  methods: {
    webrtcStreamerPlay() {
      //video：需要绑定的video控件ID
      //127.0.0.1:8000：启动webrtc-streamer的设备IP和端口，默认8000
      this.webRtcServer = new WebRtcStreamer(
        "video",
        location.protocol + "//" + this.camera_ip
      );
      // //需要查看的rtsp地址,根据自己的摄像头传入对应的rtsp地址即可。注意：视频编码格式必须是H264的，否则无法正常显示，编码格式可在摄像头的后台更改
      this.webRtcServer.connect(
        "http://localhost:8080/live/livestream.flv",
        null,
        "rtptransport=tcp&timeout=60",
        null
      );
      //   this.webRtcServer.connect("rtsp://127.0.0.1:10054/hls/test");
    },
    easyPlayer() {
      this.$refs.eaplayer.initPlayer();
    },
    src() {
      if (!this.videoUri) {
        return "";
      }
      if (this.videoUri.indexOf("rtmp://") === 0) {
        if (this.videoUri.indexOf("/") === 0) {
          return location.protocol + "//" + location.host + this.videoUri;
        }
        return this.videoUri;
      } else {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", this.videoUri);
        xhr.onreadystatechange = () => {
          if (/\.flv.*$/.test(xhr.responseURL || "") && this.typeFlv) {
            this.videoUri = xhr.responseURL;
            this.typeFlv = false;
          } else if (/\.m3u8.*$/.test(xhr.responseURL || "")) {
            this.videoUri = xhr.responseURL;
          }
        };
        xhr.send(null);
        return this.videoUri;
      }
    },
  },
};
</script>
<template>
  <div>
    <div><el-input></el-input></div>
    <video
      id="video"
      v-if="easyPlayerFlag == 'video'"
      controls
      autoplay
      muted
      loop
      crossOrign="anonymous"
      width="100%"
      height="100%"
      style="object-fit: fill"
    ></video>
    <easy-player
      ref="eaplayer"
      v-if="easyPlayerFlag == 'eaplayer'"
      :video-url="videoUri"
      :live="true"
      :stretch="false"
      :video-title="''"
      style="width: 100%; height: 85vh"
    ></easy-player>
    <!-- <EasyPlayerPro
      ref="eaplayerpro"
      v-if="easyPlayerFlag == 'eaplayerpro'"
    ></EasyPlayerPro> -->
    <!-- frameborder属性：不设置边框，如果是vue2.x版本，那么可以使用src属性  -->
    <!-- <iframe
      style="width: 100%; height: 800px"
      id="tempHtml"
      ref="tempHtml"
      frameborder="0"
      v-if="easyPlayerFlag == 'eaplayerpro'"
    ></iframe> -->
    <!-- <div v-html="demoHtml"></div> -->

    <webrtc-player :videoSrc="videoSrc" v-if="easyPlayerFlag == 'webrtcplayer'"></webrtc-player>
  </div>
</template>
<script>
// 引入webrtcstreamer.js，确保路径正确
import WebRtcStreamer from "./webrtcstreamer.js";
import EasyPlayer from "@easydarwin/easyplayer";
// import demoHtml from '../components/easyplayerpro/demo.html';
// import EasyPlayerPro from "../components/easyplayerpro/EasyPlayerPro.vue";
import WebrtcPlayer from "../components/jswebrtc/webrtcPlayer.vue";

export default {
  name: "VideoWebRtc",
  components: { EasyPlayer,WebrtcPlayer
//    EasyPlayerPro
    },
  data() {/*  */
    return {
      easyPlayerFlag: "webrtcplayer",//video eaplayer eaplayerpro webrtcplayer
      webRtcServer: null,
      camera_ip: "127.0.0.1:8000", //这里看自己的需要，也可以传入另一台电脑的ip，前提是都得在在一个局域网内
      //   videoUri:
      //     "http://127.0.0.1:8000/easyPlayerPro/live/lebtest?txSecret=f22a813b284137ed10d3259a7b5c224b&txTime=69f1eb8c&tabr_bitrates=d1080p,d540p,d360p&tabr_start_bitrate=d1080p",
      videoUri:
        "rtsp://127.0.0.1:10054/trailer",
      ifeData: "",
      videoSrc:"webrtc://127.0.0.1/live/livestream"
    };
  },
  mounted() {
    console.log("location.protocol", location.protocol,this.easyPlayerFlag);
    if (this.easyPlayerFlag == "eaplayer") {
      this.easyPlayer();
      this.$nextTick(() => {
        this.videoUri = this.src();
      });
    } else if (this.easyPlayerFlag == "video") {
      this.webrtcStreamerPlay();
    } else if (this.easyPlayerFlag == "eaplayerpro") {
      this.$refs.tempHtml.src = this.ifeData;
    } else if (this.easyPlayerFlag == "webrtcplayer") {
    //   this.videoSrc = "";
    }
  },
  destroyed() {
    if (this.webRtcServer) {
      this.webRtcServer.disconnect();
    }
  },
  methods: {
    webrtcStreamerPlay() {
      //video：需要绑定的video控件ID
      //127.0.0.1:8000：启动webrtc-streamer的设备IP和端口，默认8000
      this.webRtcServer = new WebRtcStreamer(
        "video",
        location.protocol + "//" + this.camera_ip
      );
      // //需要查看的rtsp地址,根据自己的摄像头传入对应的rtsp地址即可。注意：视频编码格式必须是H264的，否则无法正常显示，编码格式可在摄像头的后台更改
      this.webRtcServer.connect(
        "http://localhost:8080/live/livestream.flv",
        null,
        "rtptransport=tcp&timeout=60",
        null
      );
      //   this.webRtcServer.connect("rtsp://127.0.0.1:10054/hls/test");
    },
    easyPlayer() {
      this.$refs.eaplayer.initPlayer();
    },
    src() {
      if (!this.videoUri) {
        return "";
      }
      if (this.videoUri.indexOf("rtmp://") === 0) {
        if (this.videoUri.indexOf("/") === 0) {
          return location.protocol + "//" + location.host + this.videoUri;
        }
        return this.videoUri;
      } else {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", this.videoUri);
        xhr.onreadystatechange = () => {
          if (/\.flv.*$/.test(xhr.responseURL || "") && this.typeFlv) {
            this.videoUri = xhr.responseURL;
            this.typeFlv = false;
          } else if (/\.m3u8.*$/.test(xhr.responseURL || "")) {
            this.videoUri = xhr.responseURL;
          }
        };
        xhr.send(null);
        return this.videoUri;
      }
    },
  },
};
</script>

