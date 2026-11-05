# Technic ROS 2

ROS 2 migration of the Technic autonomous-drone platform.

This repository is intentionally separate from the ROS 1 workspace. The migration
is performed as tested vertical slices; the ROS 1 implementation remains the
behavioral reference until each slice reaches parity.

## Baseline

- Ubuntu 24.04
- ROS 2 Jazzy
- PX4 v1.17
- `px4_msgs` branch `release/1.17`
- Micro XRCE-DDS Agent 2.4.3
- Gazebo Harmonic

The exact PX4 firmware and `px4_msgs` definitions must match. Message translation
is a compatibility fallback, not the default development setup.

## Architecture

```text
technic_api / navigation / missions
                 |
        technic_interfaces
                 |
          technic_flight
       +---------+---------+
       |                   |
 flight core         PX4 backend
                           |
                    /fmu/in, /fmu/out
                           |
                     uXRCE-DDS
                           |
                          PX4
```

Only the PX4 backend may depend on `px4_msgs`. Stabilization, attitude/rate
control, EKF, mixer, and motor control remain in PX4.

## First milestone

The first deliverable is deliberately small:

1. establish PX4 telemetry over uXRCE-DDS;
2. arm and enter Offboard safely;
3. take off to 2 m;
4. hover for 20 seconds;
5. land using the PX4 land command.

SLAM, Nav2, perception, web UI, and hardware peripheral drivers are excluded
until this milestone is repeatable in SITL.

## Workspace

```text
src/
  technic_interfaces/  Stable public ROS 2 API
  technic_flight/      Flight state machine and PX4 adapter
  technic_aruco/       ArUco markers map detection, map -> aruco_map TF and VPE
  technic_bringup/     Launch and configuration
  technic_description/ Gazebo model and replaceable Clover CAD meshes
  technic_simulation/  Gazebo world, PX4 SITL launch and smoke mission
  technic_web/         Web UI (pages, rosbridge, web_video_server, web_api)
  technic_docs/        Documentation for the web UI (/technic/docs/)
```

External repositories are declared in `technic_ros2.repos`.

## Build (on Ubuntu 24.04 with ROS 2 Jazzy)

```bash
vcs import src < technic_ros2.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Do not build this workspace over the ROS 1 `technic_ws` environment.

## Simulation

After building PX4 v1.17 SITL:

```bash
ros2 launch technic_simulation simulation.launch.py
```

Run the takeoff, hover, square-flight and landing check:

```bash
ros2 run technic_simulation smoke_mission
```

See `docs/simulation.md` for installation, readiness checks, headless mode and
CAD replacement instructions.

## Hardware: mavros and QGroundControl

On hardware `hardware_flight.launch.py` starts mavros (as ROS 1 technic): PX4
MAVLink on UART `/dev/ttyS0` at 921600 serves both ROS and QGroundControl
(`gcs_url tcp-l://0.0.0.0:5760`). The flight manager uses the mavros backend
(`autopilot.backend: mavros`); SITL keeps uXRCE-DDS.

PX4 parameters for the UART port connected to the Orange Pi (TELEM1 here):
`MAV_0_CONFIG = TELEM1`, `MAV_0_MODE = Onboard`, `SER_TEL1_BAUD = 921600`,
`UXRCE_DDS_CFG = Disabled`.

mavros needs GeographicLib datasets once:

```bash
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

In QGroundControl add a TCP link: host is the drone IP in its access point,
port `5760`. Launch arguments: `serial_device`, `baudrate`,
`gcs_bridge:=tcp|udp|udp-b|none`, `mavros:=false`.

## DDS: Cyclone DDS on hardware

`hardware_flight.launch.py` and `main_camera.launch.py` run all nodes with
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (launch argument `rmw`). With Fast DDS
the 640x480 camera stream stalled for seconds: large samples are sent as 64 KB
UDP datagrams that the kernel has to reassemble from IP fragments. Cyclone DDS
fragments samples itself below the MTU.

Every ROS process that talks to these nodes must use the same RMW, including
`ros2` CLI tools:

DDS discovery is limited to the drone itself (`discovery_range:=LOCALHOST`,
i.e. `ROS_AUTOMATIC_DISCOVERY_RANGE`); use `discovery_range:=SUBNET` to reach the
nodes from a laptop over Wi-Fi.

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc
echo 'export CYCLONEDDS_URI='"'"'<CycloneDDS><Domain id="any"><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'"'" >> ~/.bashrc
ros2 daemon stop
```

With `LOCALHOST` discovery rmw_cyclonedds gives every ROS process a participant
index and allows only 32 of them (about 25 are used by `hardware_flight.launch.py`).
A second launch, or nodes left over from a launch that was not stopped with
Ctrl+C, then fail with `Failed to find a free participant index for domain 0`.
The launch files raise the limit with `CYCLONEDDS_URI` (unless it is already
set); shells need the same value, otherwise `ros2` tools do not see the nodes.

## Velocity control: flight/set_velocity

`flight_manager` service `flight/set_velocity` (`technic_interfaces/srv/SetVelocity`),
the ROS 2 form of ROS 1 `set_velocity`:

```bash
ros2 service call /flight/set_velocity technic_interfaces/srv/SetVelocity \
  "{vx: 0.5, vy: 0.0, vz: 0.0, yaw: .nan, yaw_rate: .nan, frame_id: body, valid_for: {sec: 1}}"
```

- needs armed Offboard flight (take off with `flight/navigate_to` first) and is
  rejected while an action (takeoff, navigate_to, land) runs; a new action
  replaces the velocity;
- `frame_id`: `map`/empty, `body`, `base_link`, `navigate_target` (resolved at
  request time, like navigate goals) or a TF frame such as `aruco_map` (followed
  continuously);
- `yaw` (heading in `frame_id`, NaN keeps the current one) or `yaw_rate`
  (rad/s, counterclockwise), not both;
- `valid_for` must be positive: when it expires the vehicle holds its position,
  so a client must repeat the call to keep moving;
- speeds above `velocity.max_speed` (3 m/s, `config/flight.yaml`) are rejected.

PX4 gets `OffboardControlMode.velocity` with a `TrajectorySetpoint` velocity
(uXRCE-DDS) or a velocity `setpoint_raw/local` (mavros).

## Web UI

`technic_web` is the ROS 1 web interface (`technic/www`) on ROS 2. It starts with
`hardware_flight.launch.py` (`web:=false` disables it) or alone:

```bash
sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-web-video-server ffmpeg pipx
pipx install butterfly          # web terminal, not packaged in apt/rosdep
ros2 launch technic_web web.launch.py
```

| URL / node | What |
|---|---|
| `http://<drone>:8000/technic/` | pages (`web_server`, ROS 1 roswww) |
| `http://<drone>:8000/technic/docs/` | documentation (`technic_docs`, `docs:=false` disables) |
| `http://<drone>:8000/technic_blocks/` | blocks programming (`technic_blocks` node, `blocks:=false` disables) |
| `ws://<drone>:9090` | rosbridge + rosapi for roslib.js |
| `http://<drone>:8080` | web_video_server: streams and `/snapshot` |
| `http://<drone>:57575` | Butterfly web terminal ("Open Web Terminal" tile) |
| `web_api` | ROS 1 style services for the pages |

The pages keep calling the ROS 1 service names; `web_api` maps them onto the
ROS 2 flight API:

| Service (`technic_web/srv/...`) | ROS 2 |
|---|---|
| `/navigate` (`Navigate`) | `flight/navigate_to` action; returns when accepted, preempts the previous web command |
| `/land` (`std_srvs/srv/Trigger`) | `flight/land` action |
| `/get_telemetry` (`GetTelemetry`) | `flight/telemetry`, `flight/state`; other frames through TF |
| `/set_velocity`, `/set_yaw_rate` | `flight/set_velocity` with `valid_for` = `velocity_valid_for` (0.5 s): repeat the call to keep moving (teleop sends 40 Hz); cancels the running web navigate |
| `/exec` (`Execute`) | shell command for console.html (recording, Wi-Fi, servo) |
| `/genmap` (`GenMap`) | ArUco map file in `technic_aruco/map` (ROS 1 genmap.py) |

### Blocks programming

The ROS 1 `technic_blocks` (Blockly) lives in `technic_web`: the page in
`blocks/www`, the examples in `blocks/programs/examples`, the backend node
`technic_blocks` and the program API `technic_blocks.api`.

ROS 1 programs used `rospy` service proxies of simple_offboard. The generated
Python now imports `technic_blocks.api`, which calls the `web_api` services:

```python
from technic_blocks.api import navigate, land, sleep

navigate(z=1.5, frame_id='body', auto_arm=True)
sleep(5)
land()
```

The code of the Python tab also runs as a plain script (`python3 program.py`
in a sourced shell). Node `technic_blocks`: `~/run`, `~/stop`, `~/load`,
`~/store`, `~/get_config` (the page parameters as JSON, since ROS 2 has no
global parameters; URL parameters such as `?navigate_tolerance=0.5` still
override them), topics `~/running`, `~/block`, `~/print`, `~/error`, `~/prompt`.
User programs are saved to `~/technic_blocks_programs` (`blocks_programs_dir:=`).

| Blocks | ROS 2 |
|---|---|
| take off, navigate (body, map, ArUco map, marker, navigate target), land, set yaw, wait, state, position, distance | work through `web_api` |
| rangefinder | `/rangefinder/range` (vl53l1x) |
| RC channel | `/mavros/mavros/rc/in` (mavros backend only) |
| GPIO, servo, PWM | `technic_blocks.gpio`: the `gpio` utility (wiringOP), as in ROS 1 |
| set velocity | `flight/set_velocity`; the API repeats the last velocity while the program runs, so it lasts until the next command as in ROS 1; after the end, Stop or an error the vehicle stops within 0.5 s and holds |
| global navigation, attitude and rate setpoints, LED strip | not ported: the runner refuses such programs before takeoff (examples `flip.xml`, `led*.xml`, `flight-led.xml`) |

Flight commands need `enable_flight:=true`. Differences from ROS 1:

- mavros topics are `/mavros/mavros/...` (`TECHNIC_MAVROS` in `www/js/ros2_compat.js`);
- the pages' UI parameters (`/navigate_ui/...`, `/land_ui/...`) live in the
  browser's localStorage: ROS 2 has no global parameter server;
- console.html records with `ros2 bag record` (a directory in `~/screens`) and
  converts with `ros2 run technic_web bag_to_mp4 <bag directory>`;
- video comes straight from `/main_camera/image_raw` (no `image_web` topics);
- the Butterfly terminal is started by the launch file (`butterfly_terminal`
  wrapper) instead of `butterfly.service`. It asks for the user's password like
  ROS 1; `butterfly_login:=false` opens a shell without a password for everyone
  on the network, `butterfly:=false` disables it. On another port
  (`butterfly_port:=`), open the index page as `index.html?butterfly_port=<port>`.
  If the port is taken (e.g. `butterfly.service` of an old image) the wrapper
  waits and that terminal keeps working;
- the documentation (GitBook Markdown of ROS 1 `roswww_static`) is the
  `technic_docs` package and is rendered in the browser, without a GitBook
  build or Internet; its texts still describe the ROS 1 commands;
- extra static directories are mounted with `mounts:="url_path=/directory"`;
- not ported: the 3D views `viz.html` and `aruco_map.html` (need
  tf2_web_republisher) and the `/emergency` service of tandl.html (it falls back
  to disarming through mavros, as before).
# technic_ws_ros2