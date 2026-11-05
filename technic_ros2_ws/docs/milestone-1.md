# Milestone 1: takeoff, hover, land

Run this milestone in SITL before enabling hardware.

## Preconditions

1. Ubuntu 24.04 and ROS 2 Jazzy are installed.
2. PX4 v1.17 and `px4_msgs` `release/1.17` are used together.
3. Micro XRCE-DDS Agent 2.4.3 is installed.
4. PX4 SITL is started with its uXRCE-DDS client on UDP port 8888.
5. QGroundControl is connected and PX4 reports a valid local position.

## Start

```bash
ros2 launch technic_bringup sitl_flight.launch.py
```

Verify the bridge before sending commands:

```bash
ros2 topic hz /fmu/out/vehicle_status_v1
ros2 topic echo /flight/state --once
```

The state must report `connected: true` and `local_position_valid: true`.

## Take off to 2 m

```bash
ros2 action send_goal /flight/takeoff technic_interfaces/action/Takeoff \
  "{height: 2.0, speed: 0.5, frame_id: map}" --feedback
```

Expected result:

- PX4 receives at least one second of Offboard heartbeat before mode switching;
- the vehicle arms and enters Offboard;
- the action succeeds within 30 seconds;
- `/flight/state` becomes `HOVERING`.

## Hover for 20 seconds

The PX4 backend keeps publishing the last position setpoint after the takeoff
action completes.

```bash
timeout 20 ros2 topic echo /flight/telemetry
```

Acceptance:

- mode remains `OFFBOARD`;
- position error remains within the configured SITL tolerance;
- `/fmu/in/offboard_control_mode` remains at approximately 10 Hz;
- no PX4 failsafe is raised.

## Land

```bash
ros2 action send_goal /flight/land technic_interfaces/action/Land "{}" --feedback
```

Landing uses `VEHICLE_CMD_NAV_LAND`. The Offboard stream is stopped after PX4
leaves Offboard mode. The action succeeds only after PX4 reports disarmed.

## Hardware gate

Hardware commands are not exposed by default:

```bash
ros2 launch technic_bringup hardware_flight.launch.py \
  serial_device:=/dev/ttyS0 baudrate:=921600 enable_flight:=true
```

Before setting `enable_flight:=true`, validate props-off:

- UART wiring and PX4 serial-port assignment;
- Agent and firmware message compatibility;
- RC/manual override;
- `COM_OF_LOSS_T` and `COM_OBL_RC_ACT`;
- QGroundControl arming checks;
- correct local-position source and coordinate direction.
