# PX4 SITL + Gazebo Harmonic simulation

The simulation runs the same `technic_flight` node and ROS 2 actions used on
hardware. Only PX4 and the vehicle are simulated.

## Components

```text
Gazebo Harmonic (technic model + sensors)
                  |
            PX4 v1.17 SITL
                  |
        uXRCE-DDS, UDP port 8888
                  |
          MicroXRCEAgent
                  |
           technic_flight
```

`ros_gz_bridge` additionally exposes `/clock`, the downward camera, and the
rangefinder to ROS 2.

## Install simulation dependencies

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake ninja-build \
  python3-jinja2 python3-jsonschema python3-yaml \
  ros-jazzy-ros-gz
```

Install PX4 build dependencies and clone the release matching
`px4_msgs release/1.17`:

```bash
cd ~
git clone --recursive --branch v1.17.0 \
  https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh
```

Log out/reboot if requested by the PX4 setup script, then build SITL:

```bash
cd ~/PX4-Autopilot
make px4_sitl_default
```

## Build the ROS 2 workspace

```bash
cd ~/technic_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Start the complete simulation

```bash
ros2 launch technic_simulation simulation.launch.py
```

Useful launch arguments:

```bash
# Explicit PX4 checkout
ros2 launch technic_simulation simulation.launch.py \
  px4_dir:=$HOME/PX4-Autopilot

# Server-only Gazebo
ros2 launch technic_simulation simulation.launch.py gui:=false

# Use an already running MicroXRCEAgent
ros2 launch technic_simulation simulation.launch.py start_agent:=false
```

The launch fails immediately with a clear error if the PX4 SITL binary is
missing.

## Verify readiness

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/technic_ros2_ws/install/setup.bash

ros2 topic echo /flight/state --once
ros2 topic hz /fmu/out/vehicle_status_v1
ros2 topic hz /main_camera/image_raw
ros2 topic echo /rangefinder --once
```

Do not send a takeoff goal until `/flight/state` reports:

```text
connected: true
local_position_valid: true
```

## Automated flight smoke mission

The mission performs:

1. takeoff to 2 m;
2. hover for 10 seconds;
3. a 1 x 1 m square;
4. PX4 landing and disarm.

```bash
ros2 run technic_simulation smoke_mission
```

Run this only after the readiness checks pass.

## CAD model replacement

The files are under:

```text
src/technic_description/models/technic/meshes/clover4/
```

Names retained from Clover/Technic:

- `clover_body_solid.dae`
- `clover_guards_transparent.dae`
- `prop_ccw.dae`
- `prop_cw.dae`

You can replace their contents without editing `model.sdf`. Keep metres as the
model unit, preserve the model origin and axes, and keep referenced textures in
the same directory.

Visual CAD is deliberately independent from collision geometry and flight
dynamics. The initial physics and motor constants follow the stable PX4 x500
baseline. Tune mass, inertia, rotor positions and motor constants only when the
replacement hardware geometry is known.

## Troubleshooting

List native Gazebo topics:

```bash
gz topic -l | sort
```

If PX4 waits for Gazebo, verify:

```bash
echo "$GZ_SIM_RESOURCE_PATH"
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

If `/fmu/*` is absent, inspect the Micro XRCE-DDS Agent log. It must establish a
client session on UDP port 8888. Do not run the hardware serial agent at the
same time as this simulation launch.
