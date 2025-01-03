<template>
    <div>
        <el-row><el-col :span="22">
                <h1>参数图</h1>
            </el-col></el-row>
        <el-row type="flex" class="row-bg" justify="end"><el-col :span="16">

                <!-- distance_control: "{{ distance_control }}",
                LQR_u：" {{ LQR_u }} "，robot_speed：" {{ robot_speed }} ", motor1_shaft_velocity="
                {{ motor1_shaft_velocity }}
                ",motor1_shaft_angle=" {{ motor1_shaft_angle }} -->

                <span v-for="(value, key) in paramsJson.speed" :key="key">
                    {{ key }} : {{ value }}
                </span>
            </el-col><el-col :span="8"> <el-button type="primary" @click="getParams">{{ button_name }}</el-button>
            </el-col></el-row>
        <el-row><el-col :span="22">
                <div ref="speedChart" style="width: 100%; height: 300px"></div>
            </el-col>
            <el-col :span="22">
                <div ref="angleChart" style="width: 100%; height: 300px"></div>
            </el-col></el-row>
    </div>
</template>

<script>
import * as echarts from "echarts";

export default {
    data() {
        return {
            speedChart: null,
            angleChart: null,
            otherChart: null,
            websocket: null,
            websocketDataUpdate: 0,
            wsServerHost: "192.168.0.139",//机器人websocket服务器地址
            wsServerPort: "81",//机器人websocket服务器监听端口
            intervalId: null, //定时器id,
            websocketFlag: false, //socket返回结果状态
            button_name: "开始",
            robot_speed: "",
            motor1_shaft_velocity: "",
            motor1_shaft_angle: "",
            motor1_shaft_angle_pre: "",
            LQR_u: "",
            distance_control: "",
            maxValue: 50,
            paramsJson: { "speed": { "LQR_angle": "1.43", "LQR_distance": "137.36", "LQR_gyro": "-18.84", "LQR_speed": "0.28", "LQR_u": "-0.34", "YAW_angle": "-6.89", "YAW_angle_last": "-6.89", "YAW_angle_total": "-0.60", "YAW_angle_zero_point": "-365.18", "YAW_gyro": "1.79", "YAW_output": "-0.53", "angle_control": "0.79", "angle_zeropoint": "0.63", "distance_control": "-0.13", "distance_zeropoint": "137.62", "gyro_control": "-1.13", "leg_position_add": "20.81", "motor1_shaft_velocity": "0.65", "motor2_shaft_velocity": "-1.18", "robot_speed": "0.28", "robot_speed_last": "0.29", "speed_control": "0.19" }, "angle": { "motor1_shaft_angle": "40.03", "motor2_shaft_angle": "-314.75" } },
        };

    },
    created() { },
    mounted() {
        this.initCharts();
    },
    beforeDestroy() {
        // 组件销毁前清除定时器
        clearInterval(this.intervalId);
    },
    destroyed() {
        if (this.websocket) {
            this.webSocket.close();
        }
    },
    methods: {
        initCharts() {
            this.speedChart = echarts.init(this.$refs.speedChart);
            this.angleChart = echarts.init(this.$refs.angleChart);
            this.setOptions(this.speedChart, this.getSpeedOption());
            this.setOptions(this.angleChart, this.getAngleOption());
        },
        setOptions(chart, option) {
            chart.setOption(option);
            window.addEventListener("resize", () => {
                chart.resize();
            });
        },
        websocket_init() {
            // 初始化websocket客户端
            let that = this;
            try {
                this.websocket = new WebSocket("ws://" + this.wsServerHost + ":" + this.wsServerPort + "/"); // 注意：这里的ws://前缀是WebSocket协议的前缀，不是http://
                this.websocket.onmessage = function (event) {
                    that.websocketDataUpdate++;
                    // console.log("socket接收到的data：", event.data);
                    try {
                        that.updateDynamicCharts(JSON.parse(event.data));
                        that.paramsJson = JSON.parse(event.data)
                    } catch (error) {
                        console.log("解析json数据出错", error);
                        if (that.websocketFlag) {
                            alert("解析数据出错:" + error);
                        }
                        that.websocketFlag = false;
                    }
                };

                // 监听WebSocket打开事件
                this.websocket.onopen = () => {
                    console.log("WebSocket connected");
                    this.websocketFlag = true;
                    this.intervalId = setInterval(this.refreshData, 2000);
                    this.button_name = "结束"
                };

                // 监听WebSocket错误事件
                this.websocket.onerror = (error) => {
                    this.websocketFlag = false;
                    console.error("WebSocket error:", error);
                    alert("WebSocket conection error");
                    this.button_name = "开始"
                };

                // 监听WebSocket关闭事件
                this.websocket.onclose = () => {
                    this.websocketFlag = false;
                    console.log("WebSocket disconnected");
                    this.button_name = "开始"
                };
            } catch (error) {
                console.error("WebSocket init error:", error);
                alert("WebSocket init error:" + error);
                this.button_name = "开始"
            }
        },
        getSpeedOption() {
            return {
                title: {
                    text: "速度图",
                },
                legend: {
                    type: "scroll",
                    orient: "vertical",
                    right: 10,
                    top: 20,
                    bottom: 20,
                    data: [
                        "robot_speed",
                        "motor1_shaft_velocity",
                        "motor2_shaft_velocity",
                    ],
                },
                // 速度图表的配置
                xAxis: {
                    type: "category", data: [], axisLabel: {
                        show: true
                    }
                },
                yAxis: { type: "value" },
                series: [
                    {
                        name: "robot_speed",
                        type: "line",
                        data: [],
                        smooth: true,
                    },
                    {
                        name: "motor1_shaft_velocity",
                        type: "line",
                        data: [],
                        smooth: true,
                    },
                    {
                        name: "motor2_shaft_velocity",
                        type: "line",
                        data: [],
                        smooth: true,
                    },
                ],
            };
        },
        getAngleOption() {
            return {
                title: {
                    text: "角度图",
                },
                legend: {
                    type: "scroll",
                    orient: "vertical",
                    right: 10,
                    top: 20,
                    bottom: 20,
                    data: [
                        "motor1_shaft_angle",
                        "motor2_shaft_angle",
                    ],
                },
                // 角度图表的配置
                xAxis: { type: "category", data: [] },
                yAxis: { type: "value" },
                series: [
                    {
                        name: "motor1_shaft_angle",
                        type: "line",
                        data: [],
                        smooth: true,
                    },
                    {
                        name: "motor2_shaft_angle",
                        type: "line",
                        data: [],
                        smooth: true,
                    },
                ],
            };
        },
        updateDynamicCharts(data) {
            let speedObject = data["speed"];
            this.robot_speed = Math.abs(speedObject["robot_speed"] * 1)
            this.motor1_shaft_velocity = speedObject["motor1_shaft_velocity"]
            this.LQR_u = speedObject["LQR_u"]
            this.distance_control = speedObject["distance_control"]

            this.speedChart.setOption({
                xAxis: {
                    data: this.addXData(this.speedChart.getOption().xAxis[0].data)
                },
                series: [
                    {
                        name: "robot_speed",
                        tooltip: {
                            trigger: 'item', // 数据项图形触发
                        },
                        data: this.addYData(
                            this.speedChart.getOption().series[0].data,
                            speedObject["robot_speed"]
                        ),
                    },
                    {
                        name: "motor1_shaft_velocity",
                        data: this.addYData(
                            this.speedChart.getOption().series[1].data,
                            speedObject["motor1_shaft_velocity"]
                        ),
                    },
                    {
                        name: "motor2_shaft_velocity",
                        data: this.addYData(
                            this.speedChart.getOption().series[2].data,
                            speedObject["motor2_shaft_velocity"]
                        ),
                    },
                ],
            });
            let angleObject = data["angle"];

            this.motor1_shaft_angle = (angleObject["motor1_shaft_angle"] * 1 - this.motor1_shaft_angle_pre * 1).toFixed(3)
            this.motor1_shaft_angle_pre = angleObject["motor1_shaft_angle"]

            this.angleChart.setOption({
                xAxis: {
                    data: this.addXData(this.angleChart.getOption().xAxis[0].data)
                },
                series: [
                    {
                        name: "motor1_shaft_angle",
                        data: this.addYData(
                            this.angleChart.getOption().series[0].data,
                            angleObject["motor1_shaft_angle"]
                        ),
                    },
                    {
                        name: "motor2_shaft_angle",
                        data: this.addYData(
                            this.angleChart.getOption().series[1].data,
                            angleObject["motor2_shaft_angle"]
                        ),
                    }
                ],
            });

        },


        // 生成x轴数据
        addXData(xAxis) {
            xAxis.push(this.websocketDataUpdate);
            if (this.websocketDataUpdate > this.maxValue) {
                xAxis.shift();
            }
            // console.log("xAxis:", xAxis)
            return xAxis;
        },
        // 生成y轴数据
        addYData(y_axis, value) {
            y_axis.push(value);
            if (this.websocketDataUpdate > this.maxValue) {
                y_axis.shift();
            }
            return y_axis;
        },
        refreshData() {
            // console.log("this.websocketFlag:", this.websocketFlag, "this.websocket.readyState:", this.websocket.readyState);
            // 通过WebSocket发送请求到服务器，请求最新的曲线数据
            if (this.websocket && this.websocketFlag && this.websocket.readyState === WebSocket.OPEN) {
                // console.log("发送请求到服务器，请求最新的曲线数据");
                this.websocket.send(JSON.stringify({ mode: "params" }));
            }
        },
        getParams() {
            console.log(this.button_name);
            if (this.button_name === "开始") {
                //发起获取
                this.button_name = "获取中..."
                // 组件创建时连接WebSocket服务器
                this.websocket_init();
            } else {
                //停止获取
                this.button_name = "开始";
                // 组件销毁前清除定时器
                clearInterval(this.intervalId);
                if (this.websocket) {
                    this.websocket.close();
                }

            }
        }
    },
};
</script>
