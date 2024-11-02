<template>
  <!-- webrtc播放器 -->
  <video id="jswebrtc" ref="jswebrtc" controls style="width: 100%; height: 100%; object-fit: fill;"></video>
</template>

<script>
export default {
  name: 'webrtcPlayer',
  props: {
    videoSrc: {
      type: String,
      default: ''
    }
  },
  data () {
    return {
      player: null
    }
  },
  mounted() {
    // this.initVideo();
    this.$watch('videoSrc', () => {
      if (this.videoSrc) {
        this.initVideo(this.videoSrc);
      }
    }, {immediate: true})
  },
  methods: {
    initVideo(url) {
      if (this.player) {
        this.player.destroy();
        this.player = null;
      }

      let videoDom = document.getElementById('jswebrtc');
      // let url = 'webrtc://192.168.50.188/01/0001/aivision/stream'
      this.player = new JSWebrtc.Player(url, {
        video: videoDom,
        autoplay: true,
        onplay: (obj) => {
          videoDom.addEventListener('canplay', function(e) {
            videoDom.play();
          })
        }
      })
    }
  },
  beforeDestroy() {
    if (this.player) {
      this.player.destroy();
    }
  }
}
</script>

<style>
</style>