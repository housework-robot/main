<template>
	<view style="position: relative;">
		<view class='toucharea' @touchstart="onTouchStart" @touchmove="onTouchMove" @touchcancel="onTouchEnd" @touchend="onTouchEnd">
			<view style="border-radius: 50%;" :style="{width:touchRadius*2+'px',height:touchRadius*2+'px'}"></view>
		</view>
		<view class="ball" :style="{left:left+'px',top:top+'px'}" :class="{animation:inDraging===false&&transition}">
			<slot name="ball">
				<view style="width: 60px;height: 60px;border-radius: 50%;"></view>
			</slot>
		</view>
		<view class='stick' :class="{animation:inDraging===false&&transition}" :style="{height: stickHeight+'px',transform:'translateX(-50%)'}">
			<view :style="{transform:'rotate('+(angle/(3.14159/180)-90)+'deg)'}" style="transform-origin: 50% 0%;width: 100%;height: 100%;">
				<slot name="stick">
				</slot>
			</view>
		</view>
		<view class="bottom">
			<slot name="bottom">
			</slot>
		</view>
	</view>
</template>

<script>
	var startLeft,startTop;
	var timer;
	var getDistance=function(x1, y1, x2, y2) {
		var _x = Math.abs(x1 - x2); 
		var _y = Math.abs(y1 - y2); 
		return Math.sqrt(_x * _x + _y * _y);
	}
	export default {
        name:"ezjoystick",
		props:{
			//触摸识别区域的半径
			touchRadius:{
				type:Number,
				default:100
			},
			//杆头的移动范围半径，也就是杆体的长度
			ballMoveRadius:{
				type:Number,
				default:50
			},
			transition:{
				type:Boolean,
				default:false
			}
		},
		data() {
			return {
				left:0,
				top:0,
				stickHeight:0,
				angle:0,
				inDraging:false
			};
		},
		mounted() {
			this.loop();
		},
		methods:{
			onTouchStart(e){
				var curTouch=e.touches[0];
				startLeft=curTouch.clientX-this.left;
				startTop=curTouch.clientY-this.top;
				
				this.inDraging=true;
			},
			onTouchMove(e){
				var curTouch=e.touches[0];
				var tleft=curTouch.clientX-startLeft;
				var ttop=curTouch.clientY-startTop;
				
				var distance = getDistance(tleft,ttop,0,0);
				
				if(distance>=this.ballMoveRadius)distance = this.ballMoveRadius;
				
				var angle = Math.atan2((ttop-0), (tleft-0));
				this.left=Math.cos(angle)*distance;
				this.top=Math.sin(angle)*distance;
				
				this.stickHeight = distance;
				this.angle = angle;
			},
			onTouchEnd(e){
				this.stickHeight=this.left=this.top=0;
				
				this.inDraging=false;
				
				this.$emit("onJoyStickCancel");
			},
			loop(){
				requestAnimationFrame(this.loop); 
				
				if(this.inDraging){
					this.$emit("onJoyStickUpdate",{angle:this.angle,direction:{x:this.left,y:this.top},power:this.stickHeight/this.ballMoveRadius});
				}
			}
		}
	}
</script>

<style>
	.toucharea{
		position: absolute;
		z-index: 4;
		transform: translate(-50%,-50%);
		border-radius: 50%;
	}
	.ball{
		position: absolute;
		z-index: 3;
		transform: translate(-50%,-50%);
	}
	.stick
	{
		position: absolute;
		z-index: 2;
	}
	.ball.animation
	{
		transition: left .1s ease-out,top .1s ease-out;
	}
	.stick.animation
	{
		transition: all .2s ease-out;
	}
	.bottom
	{
		position: absolute;
		z-index: 1;
		transform: translate(-50%,-50%);
	}
</style>
