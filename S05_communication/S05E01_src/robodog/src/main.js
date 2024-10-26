import Vue from 'vue'
import App from './App.vue'
import ElementUI from 'element-ui';
import 'element-ui/lib/theme-chalk/index.css';
// const WebRtcStreamer = require('webrtc-streamer');
Vue.use(ElementUI);
// Vue.use(WebRtcStreamer);
Vue.config.productionTip = false

new Vue({
    render: h => h(App),
}).$mount('#app')
