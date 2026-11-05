# Architecture decisions

## AD-001: ROS 2 baseline

Use Ubuntu 24.04 and ROS 2 Jazzy for the primary development and deployment
profile. Humble is a compatibility profile only.

Reasons:

- Jazzy is the current PX4 recommendation.
- PX4 message service translation is supported on Jazzy but not Humble.
- Ubuntu 24.04 provides a longer maintenance runway for a new platform.
- Gazebo Harmonic integrates with this generation of the ROS 2 stack.

## AD-002: PX4 transport and versioning

The primary transport is PX4 uXRCE-DDS over serial on hardware and UDP in SITL.
Pin PX4 firmware, `px4_msgs`, and the Micro XRCE-DDS Agent as one tested bundle.

Initial bundle:

- PX4: v1.17
- `px4_msgs`: `release/1.17`
- Micro XRCE-DDS Agent: 2.4.3 for ROS 2 Jazzy/Fast DDS 2.14

The firmware's `dds_topics.yaml` is part of the platform contract: a
`px4_msgs` type existing in the workspace does not imply that PX4 exposes it.

## AD-003: Public API boundary

No package above `technic_flight` may publish or subscribe to `/fmu/*` or depend
on `px4_msgs`.

Long-running commands use actions:

- takeoff;
- navigate;
- land.

Immediate state changes and setpoint updates may use services:

- arm/disarm;
- set velocity;
- release control.

Telemetry and flight state use topics. A telemetry service is retained only as a
compatibility facade for ROS 1 user programs.

## AD-004: Flight layers

`technic_flight` is split into:

1. command/action adapters;
2. flight manager state machine;
3. safety manager;
4. frame conversion;
5. autopilot backend.

The PX4 backend owns:

- PX4 QoS profiles;
- PX4 message timestamps and IDs;
- Offboard heartbeat;
- vehicle command/ack exchange;
- ENU/FLU to NED/FRD conversion;
- PX4 firmware compatibility details.

The flight core uses ROS REP-103 frames and SI units.

## AD-005: Interface Library adoption

The PX4 ROS 2 Interface Library is valuable, but parts of it and several setpoint
types remain experimental. Therefore the public Technic API must not expose its
types.

The first vertical slice uses a small direct `px4_msgs` backend. A second backend
using the Interface Library can be added behind the same internal interface once
the pinned PX4 release is validated. This avoids coupling the platform API to an
experimental dependency while preserving a migration path to registered PX4
flight modes.

## AD-006: Coordinate frames

The public API follows ROS conventions:

- world: ENU;
- body: FLU;
- angles: radians;
- distances: metres.

PX4 wire messages use NED/FRD where specified. Conversion happens exactly once,
inside the PX4 backend. Raw PX4 coordinates must never leak into
`technic_interfaces`.

Required transform policy:

```text
map (ENU) -> odom (ENU) -> base_link (FLU)
                              |
                         PX4 backend
                              |
                    local_ned / body_frd
```

## AD-007: Migration strategy

Port behavior, not files. The ROS 1 `simple_offboard.cpp` combines transport,
flight state, TF handling, interpolation, safety, and public services; those
responsibilities must be separated.

Order:

1. interfaces and simulated backend tests;
2. direct PX4 backend telemetry;
3. SITL takeoff-hover-land;
4. `navigate` and velocity control;
5. Python API and CLI;
6. manual override/teleoperation;
7. camera;
8. localization and ENU/NED validation;
9. GPS-denied flight;
10. Nav2, perception, and missions.

## Safety invariants for milestone 1

- Offboard heartbeat starts before requesting Offboard mode.
- Setpoints are valid before arming.
- Loss of heartbeat, PX4 connection, valid local position, or action owner stops
  external control and lets PX4 execute its configured failsafe.
- Land is sent as the PX4 navigation command, not emulated by descending an
  Offboard setpoint.
- Action cancellation transitions to a deterministic hover or PX4 land policy.
- Real-hardware launch defaults to disarmed and requires an explicit opt-in.
