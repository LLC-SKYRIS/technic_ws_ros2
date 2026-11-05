# ArUco markers and VPE

`technic_aruco` is a port of the ROS 1 `aruco_pose` package and
`technic/vpe_publisher`. Behaviour, parameters, map files and defaults match
`technic/launch/aruco.launch`.

## TF tree

```text
map ──(flight_manager, 50 Hz)──> base_link ──(main_camera.launch.py)──> main_camera_optical
 │                                                                     │
 └─(vpe_publisher, static)─> aruco_map ─(static)─> aruco_<id>          └─(aruco_map)─> aruco_map_detected
```

With `aruco_vpe:=false`, `aruco_map` publishes `main_camera_optical -> aruco_map`
instead, and `map -> aruco_map` is resolved through `base_link`.

`navigate_to` accepts `frame_id: aruco_map` (see `NavigateTo.action`).

## Launch

Hardware (camera up, markers on the ceiling as in ROS 1):

```bash
ros2 launch technic_bringup hardware_flight.launch.py enable_flight:=true \
  aruco_placement:=ceiling aruco_map_file:=map1.txt
```

Simulation (downward camera, markers on the floor):

```bash
ros2 launch technic_simulation simulation.launch.py aruco:=true
```

The Gazebo world contains no markers yet, so no map is detected there.

## Vision position estimate

`vpe_publisher` publishes the base_link pose in `map` on `/vision_pose/pose`;
the PX4 backend forwards it to `/fmu/in/vehicle_visual_odometry` (NED/FRD).
For PX4 to fuse it, configure EKF2 external vision, e.g.:

- `EKF2_EV_CTRL`: horizontal and vertical position and yaw (bits 0, 1, 3);
- `EKF2_HGT_REF`: vision if the rangefinder/baro should not be the height reference;
- `EKF2_EVP_NOISE`, `EKF2_EVA_NOISE`: vision position and angle noise.

The flight manager must run (`enable_flight:=true`): it publishes `map -> base_link`
and forwards the vision pose.

## force_init

As in ROS 1, `force_init:=true` makes `vpe_publisher` publish a zero pose to
`/vision_pose/pose` at 10 Hz while there is no vision pose, so PX4 initializes
its local position without markers (e.g. to take off and find the map). Once
the flight manager telemetry reports a valid local position, zero is still
published for `force_init_duration` (5 s) and then stops; it resumes if the
local position is lost for `force_init_timeout` (5 s). A real vision pose
younger than `force_init_timeout` always suppresses it.

```bash
ros2 launch technic_bringup hardware_flight.launch.py enable_flight:=true force_init:=true
# force init only, without marker detection:
ros2 launch technic_bringup hardware_flight.launch.py enable_flight:=true force_init:=true aruco:=false
```
