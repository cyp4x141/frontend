import * as React from 'react';
import {
  Box,
  Button,
  Card,
  CardContent,
  Divider,
  TextField,
  Stack,
  Grid
} from '@material-ui/core';
import ArrowUpwardIcon from '@material-ui/icons/ArrowUpward';
import CloseIcon from '@material-ui/icons/Close';
import ROSLIB from 'roslib';
import RENDERER from 'renderer/index';
import { useResizeDetector } from 'react-resize-detector';
import Budget from 'components/dashboard/Budget';
import PowerBar from 'components/dashboard/PowerBar';
import TotalCustomers from 'components/dashboard/TotalCustomers';
import TotalProfit from 'components/dashboard/TotalProfit';


function Sales() {
  const [ip, setIp] = React.useState("ws://" + window.location.hostname + ":9090");
  const { width, height, ref } = useResizeDetector();
  const [connected, setConnected] = React.useState(false);
  const [power, setPower] = React.useState(0);
  const handleChange = (event) => {
    setIp(event.target.value);
  };

  React.useEffect(() => {
    console.log(width, height);
    if (width && height && connected) {
      RENDERER.updateDimension(width, height);
    }
  }, [width, height, connected]);

  const ros = new ROSLIB.Ros();
  ros.on("error", function (error) {
    console.log( error );
  });
  ros.on("connection", function (error) {
    setConnected(true);
    getHdMapAndVehiclePose(ros);
    getPower(ros);
  });
  ros.on("close", function (error) {
    setConnected(false);
  });

  const getPower = (ros) => {
    var powerListener = new ROSLIB.Topic({
      ros : ros,
      name : '/bms',
      messageType : 'std_msgs/Float64',
      throttle_rate: 10000
    });

    powerListener.subscribe(function(message) {
      setPower(message.data);
      console.log(power);
    });
  }

  const handleConnect = () => {
    RENDERER.initialize('canvas', width, height);
    if (connected) {
      ros.close();
      setConnected(false);
    } else {
      ros.connect(ip);
    }
  };
  const [start, setStart] = React.useState(null);
  const [stop, setStop] = React.useState(null);

  const getHdMapAndVehiclePose = (ros) => {
    var mapListener = new ROSLIB.Topic({
      ros : ros,
      name : '/lanelet2_map_viz',
      messageType : 'visualization_msgs/MarkerArray',
    });

    mapListener.subscribe(function(message) {
      RENDERER.updateHdmp(message.markers);
    });

    var poseListener = new ROSLIB.Topic({
      ros : ros,
      name : '/tracked_pose',
      messageType : 'geometry_msgs/PoseStamped',
      throttle_rate: 100
    });

    poseListener.subscribe(function(message) {
      const z = Math.atan2(2* (message.pose.orientation.w * message.pose.orientation.z + message.pose.orientation.x * message.pose.orientation.y), 
        1 - 2 * (message.pose.orientation.z * message.pose.orientation.z + message.pose.orientation.y * message.pose.orientation.y));
      RENDERER.updatePose(message.pose.position.x, message.pose.position.y, z);
    });

    setStart(new ROSLIB.Topic({
      ros : ros,
      name : '/move_base_simple/goal',
      messageType : 'geometry_msgs/PoseStamped',
    }));

    setStop(new ROSLIB.Topic({
      ros : ros,
      name : '/move_base/cancel',
      messageType : 'ctionlib_msgs::GoalID',
    }));
  }

  function startTask() {
    var currentTime = new Date();
    var secs = Math.floor(currentTime.getTime()/1000);
    var nsecs = Math.round(1000000000*(currentTime.getTime()/1000-secs));
    var twist = new ROSLIB.Message({
        header: {
          seq: 0,
          frame_id: "map",
          stamp: {
            secs: secs,
            nsecs: nsecs
          }
        },
        pose: {
          position: {
            x: Math.random(100),
            y: Math.random(100),
            z: Math.random(100)
          },
          orientation: {
            x: 0,
            y: 0,
            z: 0,
            w: 1
          }
        }
    });
    if (start) {
      start.publish(twist);
    }
  }

  function stopTask() {
    var msg = new ROSLIB.Message({
      stamp: {
        secs: 0,
        nsecs: 0
      },
      id: ``
    });
    if (stop) {
      stop.publish(msg);
    }
  }
  return (
    <Grid
      container
      spacing={3}
    >
      <Grid
        item
        lg={12}
        md={12}
        xl={12}
        xs={12}
      >         
        <Card>
          <CardContent>
            <Stack direction={{ xs: 'column', lg: 'row' }} divider={<Divider orientation="vertical" flexItem />} spacing={2}>
              <TextField id="outlined-basic" label="IP" variant="outlined" value={ip} onChange={handleChange}/>
              <Button variant="outlined" onClick={handleConnect} disabled={connected}>{connected ? "已连接" : "连接"}</Button>
              <Box sx={{ flexGrow: 1 }} />
              <Button
                disabled={!(connected === true && start !== null)}
                variant="contained"
                endIcon={<ArrowUpwardIcon />}
                onClick={startTask}
              >
                开始任务
              </Button>
              <Button
                disabled={!(connected === true && stop !== null)}
                variant="contained"
                endIcon={<CloseIcon />}
                onClick={stopTask}
              >
                停止任务
              </Button>
            </Stack>
          </CardContent>
          <Divider />
          <CardContent>
            <Box
              sx={{
                height: 500,
                position: 'relative'
              }}
            >
              <div
                  id="canvas"
                  className="dreamview-canvas"
                  style={{width: "100%", height: "100%"}}
                  ref={ref}
              >
              </div>
            </Box>
          </CardContent>
        </Card>
      </Grid>
      <Grid
        item
        lg={3}
        sm={6}
        xl={3}
        xs={12}
      >
        <PowerBar power={power}/>
      </Grid>
      <Grid
        item
        lg={3}
        sm={6}
        xl={3}
        xs={12}
      >
        <TotalCustomers />
      </Grid>
      <Grid
        item
        lg={3}
        sm={6}
        xl={3}
        xs={12}
      >
        <Budget />
      </Grid>
      <Grid
        item
        lg={3}
        sm={6}
        xl={3}
        xs={12}
      >
        <TotalProfit sx={{ height: '100%' }} />
      </Grid>
    </Grid>
  );
};

export default Sales;
