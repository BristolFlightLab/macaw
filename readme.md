# Macaw

A ROS 2/Mavlink interface, using ROS 2 standard messages to minimize need to learn MAVLINK.  Macaw is influenced by good experiences with the [ardrone_autonomy](https://ardrone-autonomy.readthedocs.io/en/latest/#) package.

Macaw will only talk to the Mavlink system ID set by the `mavlink_sysid` parameter, as read at start-up.

| For multi-drone systems, run multiple instances of Macaw.

## Published topics

* `/macaw/sysid[id]/altitude_asl` (std_msgs/Float64) altitude above sea level in metres
* `/macaw/sysid[id]/altitude_rel_home` (std_msgs/Float64) altitude relative to home location in metres
* `/macaw/sysid[id]/angular_velocity` (geometry_msgs/Vector3) angular velocity in radians/second, x: roll, y: pitch and z: yaw
* `/macaw/sysid[id]/attitude` (geometry_msgs/Quaternion) attitude 
* `/macaw/sysid[id]/battery_state` (sensor_msgs/BatteryState) battery information including voltage, current and percentage
* `/macaw/sysid[id]/global_pos` (sensor_msgs/NavSatFix) global position from the EKF, without altitude (ROS requires WGS84 altitude but not available from MAVLINK GLOBAL_POSITION message)
* `/macaw/sysid[id]/global_target` (sensor_msgs/NavSatFix) the autopilot's current working target point
* `/macaw/sysid[id]/gps_raw` (sensor_msgs/NavSatFix) global position straight from the GPS
* `/macaw/sysid[id]/heading` (std_msgs/Float64) heading angle in degrees CW from north
* `/macaw/sysid[id]/heartbeat_count` (std_msgs/UInt64) number of heartbeats received since start-up
* `/macaw/sysid[id]/is_armed` (std_msgs/Bool) TRUE if drone armed (i.e. motors turning), false otherwise
* `/macaw/sysid[id]/local_position` (geometry_msgs/Point) position from the EKF in the local reference frame, x: North, y: East, z: Down
* `/macaw/sysid[id]/local_target` (geometry_msgs/Point) the autopilot's current working target in the local reference frame, x: North, y: East, z: Down
* `/macaw/sysid[id]/local_velocity` (geometry_msgs/Vector3) velocity from the EKF in the local reference frame, x: North, y: East, z: Down
* `/macaw/sysid[id]/mode` (std_msgs/UInt8) the current mode as a number
* `/macaw/sysid[id]/status` (std_msgs/UInt8) the current status (see [MAVSTATE](https://mavlink.io/en/messages/common.html#MAV_STATE))
* `/macaw/sysid[id]/vertical_speed_fpm` (std_msgs/Float64) the vertical speed in feet per minute, positive if climbing

## Transforms

Macaw publishes two transforms relative to the `home[id]_ned` reference frame:
* `macaw[id]_ned` the reference frame embedded in the drone, derived from local position and attitude messages
* `target[id]_ned` reference frame of the current local target, including its yaw orientation

## Subscribed topics

The following topics can be used to send commands to the drone.  Macaw reports command acknowledgments via the ROS logger.

* `/macaw/sysid[id]/arm` (std_msgs/Empty) publish to attempt to arm the drone
* `/macaw/sysid[id]/cmd_mode` (std_msgs/String) if the drone can identify a mode number associated with the string data, will attempt to change to that mode
* `/macaw/sysid[id]/cmd_pos_global` (sensor_msgs/NavSatFix) send a global target position to the drone, with altitude relative to WGS84 ellipsoid, for guided mode
* `/macaw/sysid[id]/cmd_pose_local` (geometry_msgs/Pose) send a target pose to the drone in the local frame of reference, for guided mode.  Rejected if target roll and pitch not zero.
* `/macaw/sysid[id]/cmd_vel` (geometry_msgs/Twist) send a target velocity in body frame, for guided mode.  Responds to left/right, forward/backward, up/down, and yaw.
* `/macaw/sysid[id]/cmd_vel_global` (geometry_msgs/Twist) send a target velocity in local frame, for guided mode.  Responds to east/west, north/south, up/down, and yaw.
* `/macaw/sysid[id]/land` (std_msgs/Empty) publish to land the drone at its current location
* `/macaw/sysid[id]/takeoff` (std_msgs/Float64) publish to takeoff the drone and climb to specified altitude

## Docker

Macaw comes with some Docker files for testing

### Tidy

```docker-compose up```

This will bring up a SITL container and a Macaw node talking to it.  You can connect a ground station directly to the SITL on localhost 5762.  Open `localhost:8080` and select datasource `ws://localhost:9090` to fly and view in Foxglove.

### Hack

With a SITL running through MissionPlanner, open a Mavlink server port (`Ctrl+F` then `Mavlink`) on TCP port 14550 and then run
```
docker run -it macaw ros2 launch macaw macaw.launch.xml connect_str:="tcp:192.168.0.195:14550"
```
replacing the 192.168.0.195 with your local IP address.

(It ought to work in reverse by exposing the relevant port from the container and then connecting as client from MissionPlanner - but have yet to get that working.)