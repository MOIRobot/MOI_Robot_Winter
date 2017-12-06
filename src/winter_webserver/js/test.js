// Set the rosbridge and mjpeg_server port
var rosbridgePort = "9090";
var mjpegPort = "8080";

// Get the current hostname
thisHostName = document.location.hostname;

// If the rosbridge server is running on the webserver host, then this will
// work.
var rosbridgeHost = thisHostName;
var mjpegHost = thisHostName;
var serverURL = "ws://" + rosbridgeHost + ":" + rosbridgePort;
var ros = new ROSLIB.Ros();
var connection = null;
// Create a connection to the rosbridge WebSocket server.
try {
    connection = ros.connect(serverURL);
    //connection.publish(moveBaseTopic, 'geometry_msgs/PoseStamped', moveBaseMsg(0, 0, 0, 0));
    console.log('Successfully to connect to rosbridge!');
} catch (err) {
    console.log('Failed to connect to rosbridge!');
}
function init_ros() {
    
}
// Wait until a connection is made before continuing
ros.on('connection', function() {
    console.log('Rosbridge connected.');
});
// Create the main Navigation viewer.
var navViewer = new ROS2D.Viewer({
	divID : 'navCanvas',
	width : 600,
	height : 600
 });

// Setup the Navigation client.
 var navGridClient = new NAV2D.OccupancyGridClientNav({
	ros : ros,
	connection: connection,
	rootObject : navViewer.scene,
	viewer : navViewer,
	withOrientation: true,
	serverName : '/move_base',
	topic: 'map'
    });
    // Scale the canvas to fit to the map
    navGridClient.on('change', function() {
      viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
      viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });    
function moveBaseMsg(x, y, qz, qw) {
    return '{"header":{"frame_id":"/map"},"pose":{"position":{"x":' + x + ',"y":' + y + ',"z":0},"orientation":{"x":0,"y":0,"z":' + qz + ',"w":' + qw + '}}}';
}

function poseMsg(x, y, qz, qw) {
    // NOTE: The time stamp *must* be set to 0,0 for this to work.
    return '{"header":{"frame_id":"/map","stamp":{"secs":0, "nsecs":0}},"pose":{"pose":{"position":{"x":' + x + ',"y":' + y + ',"z":0},"orientation":{"x":0,"y":0,"z":' + qz +',"w":' + qw + '}}}}';
}
function getPoseMsg(qx,qy,qz,qw)
{
	var PoseMsg = new ROSLIB.Message({
	header : {
	     frame_id : '/map',
	    stamp : {
			secs: 0.0,
			nsecs: 0.0
		}
	},
	pose : {
			pose : 
			{
				position : {
					x : qx,
					y : qy,
					z : 0.0
				},
			orientation : {
				x : 0.0,
				y : 0.0,
				z : qz,
				w : qw
			}
		}
			
		}
    });
    return PoseMsg;
}
function getGoalMsg(qx,qy,qz,qw)
{
	var GoalMsg = new ROSLIB.Message({
	header : {
	     frame_id : '/map'
	},
	pose : {
			pose : 
			{
				position : {
					x : qx,
					y : qy,
					z : 0.0
				},
			orientation : {
				x : 0.0,
				y : 0.0,
				z : qz,
				w : qw
			}
		}
			
		}
    });
    return GoalMsg;
}
function setPose() {
	
    var posem=getPoseMsg(0.0,0.0,0.0,0.0);
    var goalm=getGoalMsg(0.0,0.0,0.0,0.0);
	posePub.publish(posem);
	initialposePub.publish(posem);
	initialposePub.publish(posem);
	goalPub.publish(goalm);
}


