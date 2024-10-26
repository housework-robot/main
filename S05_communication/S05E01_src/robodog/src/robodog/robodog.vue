<template>
  <div>
    <div><h1>机器狗状态查看</h1></div>
    <el-container>
      <el-aside width="1000px">
        <div style="text-align: center"><h2>机器人周边视频查看</h2></div>
        <div style="text-align: center; width: 100%; height: 90%">
          <VideoWebRtc></VideoWebRtc>
        </div>
        <div>
        </div>

      </el-aside>
      <el-container>
        <el-main>
          <div style="text-align: left"><h2>机器人状态查看</h2></div>
          <div ref="chart" style="width: 600px; height: 200px"></div>
        </el-main>
        <el-footer style="height: 500px">
          <div style="text-align: left"><h2>机器人命令发送</h2></div>
          <el-row :gutter="20">
            <el-col :span="12"
              ><div class="grid-content bg-purple" style="text-align: center">
                <AnalogRemote></AnalogRemote></div
            ></el-col>
            <el-col :span="12"
              ><div class="grid-content bg-purple" style="text-align: center">
                <AnalogRemote></AnalogRemote></div
            ></el-col>
          </el-row>
          <el-row :gutter="20" style="margin-top: 10px">
            <el-col :span="12"
              ><div class="grid-content bg-purple">
                <span style="margin-left: -70px">前后左右运动</span>
              </div></el-col
            >
            <el-col :span="12"
              ><div class="grid-content bg-purple">
                <span style="margin-left: -70px">前后左右翻滚</span>
              </div></el-col
            >
          </el-row>
          <el-row style="margin-top: 10px">
            <el-col class="input-wrap">
              <el-input
                type="textarea"
                :rows="5"
                v-model="command"
                placeholder="请输入命令"
              >
              </el-input>
            </el-col>
          </el-row>
          <el-row>
            <el-col
              :xs="24"
              :sm="24"
              class="btn-wrap"
              style="text-align: left; padding-top: 10px"
            >
              <el-button type="primary" @click="sendCommand">发送</el-button>
              <el-button type="primary" @click="clean">清空</el-button>
            </el-col>
          </el-row>
        </el-footer>
      </el-container>
    </el-container>
  </div>
</template>
<script>
import * as echarts from "echarts";
import VideoWebRtc from "./VideoWebRtc.vue";
import AnalogRemote from "./AnalogRemote.vue";

export default {
  name: "RoboDog",
  components: {
    VideoWebRtc,
    AnalogRemote,
  },
  data() {
    return {
      command: "",
      chart: null,
      timer: null,
      angle: 0,
      x_axis: [],
      x_data: 0,
      y_axis: [],
      y_data: 0,
      option: {
        xAxis: {
          type: "category",
          data: [],
        },
        yAxis: {
          type: "value",
        },
        series: [
          {
            data: [],
            type: "line",
            smooth: true, //开启平滑曲线
          },
        ],
      },
    };
  },
  mounted() {
    this.initChart();
    this.startAnimation();
  },
  beforeDestroy() {
    if (this.timer) {
      clearInterval(this.timer);
    }
  },
  methods: {
    initChart() {
      this.chart = echarts.init(this.$refs.chart);
      this.chart.setOption(this.option);
      for (var i = 0; i < 100; i++) {
        this.addData(false);
      }

      this.option.xAxis.data = this.x_axis;
      this.option.series[0].data = this.y_axis;
      this.chart.setOption(this.option);
    },
    startAnimation() {
      let that = this;
      this.timer = setInterval(function () {
        that.addData(true); // 生成数据并左移
      }, 200); // 500ms刷新一次
      console.log(" this.timer", this.timer);
    },
    // 生成x轴和y轴数据
    addData(shift) {
      this.x_data++;
      this.x_axis.push(this.x_data);

      let y_data = 10 * Math.sin(this.angle * Math.PI);
      this.y_axis.push(y_data);
      this.angle += 0.1;

      if (shift) {
        this.x_axis.shift();
        this.y_axis.shift();
      }
      this.option.xAxis.data = this.x_axis;
      this.option.series[0].data = this.y_axis;
      this.chart.setOption(this.option);
    },
    sendCommand() {
      if (this.command != "") {
        this.$alert("命令已发送:" + this.command);
      } else {
        this.$alert("命令不能为空");
      }
    },
    clean() {
      this.command = "";
    },
  },
};
</script>
<style>
.el-header,
.el-footer {
  background-color: #b3c0d1;
  color: #333;
  text-align: center;
  line-height: 20px;
}

.el-aside {
  background-color: #d3dce6;
  color: #333;
  text-align: center;
  line-height: 20px;
}

.el-main {
  background-color: #e9eef3;
  color: #333;
  text-align: center;
  line-height: 20px;
}

body > .el-container {
  margin-bottom: 40px;
}

.el-container:nth-child(5) .el-aside,
.el-container:nth-child(6) .el-aside {
  line-height: 260px;
}

.el-container:nth-child(7) .el-aside {
  line-height: 320px;
}
</style>
