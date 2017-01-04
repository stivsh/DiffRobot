function draw_lane(x1,y1,x2,y2,context){
	context.beginPath();
	context.moveTo(x1, y1);
	context.lineTo(x2, y2);
	context.stroke();
}

function draw_arc(x,y,r,from,to,context){
	context.beginPath();
  context.arc(x,y,r,from,to);
	context.stroke();
}

function get_ctx(){
  var canvas = document.getElementById("myCanvas");
  var ctx = canvas.getContext("2d");
  return ctx;
}

function draw_back(){
  var ctx =  get_ctx();

  ctx.clearRect(0, 0, 400, 400);

  ctx.fillStyle = '#202020';
  ctx.fillRect(0,0,400,400);

  ctx.lineWidth="2";
  ctx.strokeStyle="#006600";

  draw_arc(200,200,50,Math.PI,0,ctx);
  draw_arc(200,200,100,Math.PI,0,ctx);
  draw_arc(200,200,200,Math.PI,0,ctx);
  draw_lane(0,200,400,200,ctx);
  draw_lane(200,200,200,0,ctx);
  draw_lane(200,200,400-58,58,ctx);
  draw_lane(200,200,58,58,ctx);
}

function Obstacle(x,y){
  this.time_of_life = 0;
  this.r = 1;
  this.grow_sign = 1
	this.x=x;
	this.y=y;

	this.draw=function (context){

    this.r += 1.1*this.grow_sign;
    if(this.r > 10)
      this.grow_sign = -1;
    if(this.r < 3)
      this.grow_sign = 1;

    var alpha = 1 - this.time_of_life*0.01;
    alpha = (alpha>0.01)?alpha:0.01;
    context.globalAlpha = alpha;

    context.beginPath();
    context.arc(this.x, this.y, this.r, 0, 2 * Math.PI, false);
    context.lineWidth = 	1;
    context.fillStyle = "#006600";
    context.fill();
    context.stroke();
    this.time_of_life += 1;
    context.globalAlpha = 1;
  }
}
