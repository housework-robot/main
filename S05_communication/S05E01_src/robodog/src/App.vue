<template>
  <div id="app">
    <RoboDog></RoboDog>
  </div>
</template>

<script>
import RoboDog from "./robodog/robodog.vue";

export default {
  name: "App",
  components: {
    RoboDog,
  },
};
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
}
</style>
