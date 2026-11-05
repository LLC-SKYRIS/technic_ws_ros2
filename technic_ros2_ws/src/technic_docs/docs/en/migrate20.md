# Migration to version 0.20

[Image](image.md) version v0.20 includes significant changes in comparison with the version 0.19. When transitioning please note the changes presented below.

## ROS package `clever` is renamed to `technic`

All the imports in Python scripts should be changed.

Before:

```python
import rospy
from clever import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Take off 1 m
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
```

After:

```python
import rospy
from technic import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Take off 1 m
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
```

## systemd service `clever` is renamed to `technic`

For restarting the platform instead of:

```bash
sudo systemctl restart clever
```

use command:

```bash
sudo systemctl restart technic
```

## Path to platform's files changed

The `~/catkin_ws/src/clever/` directory is renamed to `~/catkin_ws/src/technic`. Thus, configuration files (`*.launch`) are to be edited using the new path.

For example, `~/catkin_ws/src/clever/clever/launch/clever.launch` file is now `~/catkin_ws/src/technic/technic/launch/technic.launch`.

## Wi-Fi network configuration

Wi-Fi networks' SSID is changed to `technic-XXXX` (where X is a random number), password is changed to `technicwifi`.

## The camera orientation configuration changed

See details in the "[Camera setup](camera_setup.md#frame)" article.
