# ROS 1 to ROS 2 migration map

Source workspace: `/Users/skyris/technic_ws/technic_ws/src`.

## Package disposition

| ROS 1 package | ROS 2 destination | Strategy | Phase |
|---|---|---|---|
| `technic/technic` | `technic_interfaces`, `technic_flight`, `technic_bringup`, later `technic_api` | Split by responsibility; do not line-port the monolith | 1-5 |
| `technic_description` | `technic_description` | Port xacro/URDF without Gazebo Classic plugins first | 5 |
| `technic_simulation` | `technic_simulation` | Rebuild for PX4 SITL + Gazebo Harmonic | 6+ |
| `cv_camera` | external `v4l2_camera` plus configuration | Replace | 7 |
| `aruco_pose` | `technic_aruco` (`aruco_detect`, `aruco_map`, `vpe_publisher` components) | Ported, vendored ArUco kept | 8 |
| optical flow nodelet | `technic_localization` component | Rewrite nodelet and parameters | 8 |
| `vl53l1x` | `technic_sensors` | Port driver after flight baseline | 10 |
| `opi_ws2812_rt` | `technic_led` | Port hardware adapter | 10 |
| `yolo_face` | `technic_perception` | Preserve RKNN inference behind ROS 2 interfaces | 10 |
| `genmap_run` | localization tooling | Replace shell orchestration | 10 |
| `roswww`, `roswww_static`, `technic_blocks` | `technic_web` | Reassess API around rosbridge/Foxglove | 10 |

## `simple_offboard.cpp` decomposition

| Existing responsibility | New owner |
|---|---|
| MAVROS publishers/subscribers/services | PX4 backend |
| setpoint heartbeat timer | PX4 backend |
| arm and mode switching | flight manager + PX4 backend |
| `navigate` interpolation | trajectory generator |
| TF lookup and user frames | frame resolver |
| MAVROS/PX4 coordinate conventions | PX4 frame converter |
| stale telemetry checks | safety manager |
| `get_telemetry` | telemetry publisher + compatibility service |
| ROS services | action/service adapters |
| `State.msg` publication | `FlightState.msg` |

## ROS 1 API compatibility

The old service names and request semantics are useful compatibility targets, but
not all are suitable as the canonical ROS 2 API.

| ROS 1 API | Canonical ROS 2 API | Compatibility |
|---|---|---|
| `navigate` | `NavigateTo.action` | facade service later |
| `navigate_global` | future global navigation action | after local flight |
| `land` Trigger | `Land.action` | Trigger facade later |
| `get_telemetry` | `Telemetry.msg` topic | service facade retained |
| `set_position` | internal setpoint command/service | phase 4 |
| `set_velocity` | `flight/set_velocity` (`SetVelocity.srv`) with expiry | done; `/set_velocity` facade in technic_web |
| `set_attitude`, `set_rates` | expert API, disabled by default | later |
| `set_altitude`, `set_yaw`, `set_yaw_rate` | partial-goal command API | later |

## Deferred risks

- ROS 1 nodelets must become ROS 2 components or separate processes.
- `dynamic_reconfigure` becomes declared parameters with validation callbacks.
- legacy `tf` calls become pure `tf2`.
- Gazebo Classic models/plugins are not binary-portable to Gazebo Harmonic.
- the current camera pipeline contains board-specific V4L2 and `LD_PRELOAD`
  workarounds that require validation on the production Orange Pi image.
