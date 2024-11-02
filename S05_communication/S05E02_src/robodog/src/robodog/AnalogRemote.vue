<template>
  <div id="app">
    <div
      class="knob-container"
      @mousedown="handleClick"
      @mouseup="handleMouseUp"
    >
      <div class="knob" :style="knobStyle"></div>
    </div>
  </div>
</template>

<script>
export default {
  name: "AnalogRemote",
  components: {},
  props:{
  },
  data() {
    return {
      knobStyle: {
        top: "37.5%",
        left: "37.5%",
      },
    };
  },
  methods: {
    handleClick(event) {
      const container = this.$el.querySelector(".knob-container");
      const rect = container.getBoundingClientRect();
      const x = event.clientX - rect.left;
      const y = event.clientY - rect.top;
      const centerX = (rect.width - 25) / 2;
      const centerY = (rect.height - 25) / 2;
      const radius = Math.min(centerX, centerY);

      // 计算摇杆位置的百分比
      const dx = (x - centerX) / radius;
      const dy = (y - centerY) / radius;

      // 限制摇杆移动不超过大圆的范围
      const angle = Math.atan2(dy, dx);
      const distance = Math.min(Math.sqrt(dx * dx + dy * dy), 1);
      console.log("angle:" + angle + " distance:" + distance);

      // 更新摇杆位置
      this.knobStyle = {
        top: y - 25 + "px",
        left: x - 25 + "px",
      };
      //   this.knobStyle = {
      //     top: centerY - distance * Math.cos(angle) * (radius / centerX) + "px",
      //     left: centerX + distance * Math.sin(angle) * (radius / centerX) + "px",
      //   };
      console.log(
        "knobStyle:" + this.knobStyle.top + "knobStyle:" + this.knobStyle.left
      );

      // 输出摇杆的位置坐标
      console.log("Knob position: x: " + dx + ", y:" + dy);
    },
    handleMouseDown(event) {
      // 开始拖动时的逻辑
      this.isDragging = true;
      console.log("Mouse down");
    },
    handleMouseUp(event) {
      // 鼠标左键松开时的逻辑
        console.log("Mouse up");
        
      this.$nextTick(() => {
        this. knobStyle.top="37.5%";
        this. knobStyle.left="37.5%";
      })
    },
    handleMouseLeave(event) {
      // 鼠标移出knob-container区域时的逻辑

      if (this.isDragging) {
        console.log("Mouse leave");
        this.isDragging = false;
        // 可以在这里重置knob的位置或执行其他操作      
        this.$nextTick(() => {
        this. knobStyle.top="37.5%";
        this. knobStyle.left="37.5%";
      })
      }
    },
  },
};
</script>
<style scoped>
.knob-container {
  position: relative;
  width: 200px; /* 大圆的直径 */
  height: 200px;
  border-radius: 50%;
  background-color: #eee;
  cursor: pointer;
}

.knob {
  position: absolute;
  width: 50px; /* 小圆的直径 */
  height: 50px;
  border-radius: 50%;
  background-color: #333;
}
</style>
