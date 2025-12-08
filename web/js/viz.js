var ros = new ROSLIB.Ros({
    url: 'ws://' + location.hostname + ':9090'
});

ros.on('connection', function() { console.log('ROS connected'); });
ros.on('error', function(error) { console.log('ROS connection error', error); });
ros.on('close', function() { console.log('ROS disconnected'); });

const canvas = document.getElementById('mapCanvas');
canvas.width = 1200;
canvas.height = 1200;
const ctx = canvas.getContext('2d');
const scale = 110;
function drawPoints(points, color, size=6, shape='rect') {
    ctx.fillStyle = color;
    points.forEach(p => {
        let x = 0, y = 0;

        if('x' in p && 'y' in p){
            x = p.x * scale;
            y = canvas.height - p.y * scale;
        } else if(Array.isArray(p)) {
            x = p[0] * scale;
            y = canvas.height - p[1] * scale;
        } else {
            return;
        }

        if(shape === 'circle') {
            ctx.beginPath();
            ctx.arc(x, y, size/2, 0, 2*Math.PI);
            ctx.fill();
        } else {
            ctx.fillRect(x - size/2, y - size/2, size, size);
        }
    });
}

var maskSub = new ROSLIB.Topic({
    ros: ros,
    name: '/pipeline_global_mask',
    messageType: 'visualization_msgs/Marker'
});

var junctionsSub = new ROSLIB.Topic({
    ros: ros,
    name: '/pipeline_global_junctions',
    messageType: 'visualization_msgs/Marker'
});

let maskPoints = [];
let junctionPoints = [];

maskSub.subscribe(function(msg) {
    maskPoints = msg.points;
    render();
});

junctionsSub.subscribe(function(msg) {
    junctionPoints = msg.points;
    render();
});

function render() {
    ctx.fillStyle = 'black';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    if(maskPoints.length>0) drawPoints(maskPoints, 'white', 6, 'rect');

    if(junctionPoints.length>0) drawPoints(junctionPoints, 'red', 50, 'circle');
}
