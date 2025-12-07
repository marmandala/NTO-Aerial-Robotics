var ros = new ROSLIB.Ros({ url: 'ws://' + location.hostname + ':9090' });

ros.on('connection', function() { console.log('ROS connected'); });
ros.on('error', function(error) { console.log('ROS connection error', error); });
ros.on('close', function() { console.log('ROS disconnected'); });

const missionParam = new ROSLIB.Param({
    ros: ros,
    name: '/mission_command'
});

const startBtn = document.getElementById('startBtn');
const stopBtn = document.getElementById('stopBtn');
const killBtn = document.getElementById('killBtn');

function setMissionCommand(cmd) {
    missionParam.set(cmd);
    console.log(`Mission command set: ${cmd}`);
}

startBtn.onclick = () => setMissionCommand('start');
stopBtn.onclick = () => setMissionCommand('stop');
killBtn.onclick = () => setMissionCommand('kill');

const junctionsUl = document.getElementById('junctionsUl');
var junctionsSub = new ROSLIB.Topic({
    ros: ros,
    name: '/pipeline_global_junctions',
    messageType: 'visualization_msgs/Marker'
});

junctionsSub.subscribe(function(msg) {
    const points = msg.points;
    junctionsUl.innerHTML = '';
    points.forEach((p,i) => {
        const li = document.createElement('li');
        li.innerText = `J${i}: x=${p.x.toFixed(2)}, y=${p.y.toFixed(2)}, z=${p.z.toFixed(2)}`;
        junctionsUl.appendChild(li);
    });
});