<template>
  <div id="app">
    <ShowParams />
    <img alt="Vue logo" src="./assets/logo.png" v-if="false">
    <HelloWorld msg="Welcome to Your Vue.js App" v-if="false"/>
  </div>
</template>

<script>
import HelloWorld from './components/HelloWorld.vue' 
import ShowParams from './robotparams/ShowParams.vue' 

export default {
  name: 'App',
  components: {
    HelloWorld,ShowParams,
  }
}
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
</style>
