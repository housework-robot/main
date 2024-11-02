<template>
  <div id="app">
    <h2>EasyPlayerPro案例演示</h2>
    <div :class="['player_container', 'player_container_1']">
      <div class="player_item">
        <div class="player_box" ref="player_box1"></div>
      </div>
    </div>
    <div class="df">
      <div>
        <input @click="onUse('hasAudio')" type="checkbox" :checked="config.hasAudio" /><span @click="onUse('hasAudio')">音频</span>
      </div>
    </div>
    <div class="df">
      <div>播放地址：</div><input class="inputs" v-model="videoUrl">
    </div>
    <div class="df">
      <div class="radio-item" @click="onReplay()" v-if="isPlay">重播</div>
      <div class="radio-item" @click="onPlayer()" v-if="!isPlay">播放</div>
      <div class="radio-item" @click="onPlayerPlayback()" v-if="!isPlay">回放</div>
      <div class="radio-item" @click="onPause()">暂停</div>
      <div class="radio-item" @click="onMute()">静音</div>
      <div class="radio-item" @click="setFullscreen()">全屏</div>
      <div class="radio-item" @click="onStop()" v-if="isPlay">注销</div>
    </div>
  </div>
</template>

<script>
import {EasyPlayerPro} from './EasyPlayer-pro.js'; // 确保路径正确

export default {
  name: 'App',
  data() {
    return {
      videoUrl: "ws://localhost:6230/ws_flv/live/stream_1_0.flv",
      config: {
        hasAudio: true,
        isLive: true,
        MSE: false,
        WCS: false
      },
      isPlay: false,
      playerInfo: null,
    };
  },
  mounted() {
    this.playCreate();
  },
  methods: {
    onUse(type) {
      if (type === 'hasAudio') {
        this.config.hasAudio = !this.config.hasAudio;
      } else {
        this.config.MSE = false;
        this.config.WCS = false;
        if (type === 'MSE') this.config.MSE = true;
        if (type === 'WCS') this.config.WCS = true;
      }
      if (this.isPlay) {
        this.onDestroy().then(() => {
          this.playCreate();
          this.onPlayer();
        });
      }
    },
    setFullscreen() {
      this.playerInfo.setFullscreen(true);
    },
    onPause() {
      this.playerInfo.pause();
    },
    onMute() {
      this.playerInfo.setMute(true);
    },
    onPlayer() {
      this.isPlay = true;
      setTimeout(() => {
        this.playerInfo && this.playerInfo.play(this.videoUrl).then(() => {}).catch(e => {
          console.error(e);
        });
      }, 0);
    },
    onPlayerPlayback() {
      this.onDestroy().then(() => {
        this.playCreate();
        this.config.isLive = false;
        setTimeout(() => {
          this.playerInfo && this.playerInfo.play(this.videoUrl).then(() => {}).catch(e => {
            console.error(e);
          });
        }, 0);
      });
    },
    onStop() {
      this.isPlay = false;
      this.onDestroy().then(() => {
        this.playCreate();
      });
    },
    onDestroy() {
      let _this = this;
      return new Promise((resolve, reject) => {
        if (this.playerInfo) {
          this.playerInfo.destroy();
          this.playerInfo = null;
        }
        setTimeout(() => {
          resolve();
        }, 100);
      });
    },
    onReplay() {
      this.onDestroy().then(() => {
        this.playCreate();
        this.onPlayer();
      });
    },
    playCreate() {
      const container = this.$refs.player_box1;
      const easyplayer = new EasyPlayerPro(container, {
        isLive: this.config.isLive,
        bufferTime: 0.2,
        stretch: false,
        MSE: this.config.MSE,
        WCS: this.config.WCS,
        hasAudio: this.config.hasAudio,
        watermark: { text: { content: 'easyplayer-pro' }, right: 10, top: 10 },
      });

      easyplayer.on("fullscreen", function (flag) {
        console.log('is fullscreen', flag);
      });
      easyplayer.on('playbackRate', rate => {
        easyplayer.setRate(rate);
      });

      easyplayer.on('playbackSeek', data => {
        console.log('playbackSeek', data);
      });

      this.playerInfo = easyplayer;
    }
  }
};
</script>

<style scoped>
/* 将CSS样式放在这里 */
* {
  margin: 0;
  padding: 0;
}

p {
  line-height: 24px;
}

#app {
  padding-top: 10px;
  margin: auto;
  max-width: 1200px;
}

.radio-container {
  padding: 10px 0;
}

.radio-item {
  cursor: pointer;
  display: inline-block;
  padding: 6px 12px;
  margin-right: 15px;
  border-radius: 4px;
  border: 1px #ccc solid;
}

.radio-active {
  color: #fff;
  background-color: #07baf4;
  border-color: #07baf4;
}

.player_container {
  display: grid;
}

.player_container_1 {
  grid-template-columns: 1fr;
  grid-template-rows: 1fr;
}

.player_item {
  position: relative;
  padding-bottom: 56%;
  background-color: #000;
  border: 1px #fff solid;
}

.inputs {
  -webkit-appearance: none;
  background-color: #fff;
  background-image: none;
  border-radius: 4px;
  border: 1px solid #dcdfe6;
  box-sizing: border-box;
  color: #606266;
  display: inline-block;
  font-size: inherit;
  height: 36px;
  line-height: 36px;
  outline: none;
  padding: 0 15px;
  transition: border-color .2s cubic-bezier(.645, .045, .355, 1);
  width: 100%;
  max-width: 600px;
  margin-right: 16px;
}

.player_box {
  height: 100%;
  position: absolute;
  top: 0;
  bottom: 0;
  right: 0;
  left: 0;
}

.df {
  display: flex;
  align-items: center;
  margin-bottom: 16px;
}

.df span {
  margin-left: 4px;
}

.df form {
  margin-right: 4px;
}
</style>