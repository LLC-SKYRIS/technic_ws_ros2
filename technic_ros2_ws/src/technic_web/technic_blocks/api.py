"""Synchronous API for Technic programs on ROS 2 (blocks and plain Python scripts).

The ROS 1 programs called simple_offboard through rospy service proxies. Here the
same functions call the ROS 1 style services of technic_web/web_api (/navigate,
/get_telemetry, /land, /set_velocity), which drive the ROS 2 flight_manager:

    from technic_blocks.api import navigate, get_telemetry, land, sleep

    navigate(z=1.5, frame_id='body', auto_arm=True)
    sleep(5)
    land()

A hidden node "technic_program_<pid>" is created on the first call and spun in a
background thread, so a program needs no rclpy boilerplate. When the program is
started from the blocks page, technic_blocks/stop makes every waiting call
(sleep, service calls, wait_for_message) raise Stop.
"""

import math
import os
import threading
import time

__all__ = [
    'Stop', 'NotSupported', 'get_telemetry', 'navigate', 'land', 'set_velocity', 'set_yaw',
    'navigate_global', 'set_attitude', 'set_rates', 'sleep', 'get_time', 'is_shutdown',
    'wait_for_message', 'get_range', 'get_rc_channel', 'set_effect', 'set_leds', 'LEDState',
    'get_led_count', 'check_stop', 'stop_velocity',
]

SERVICE_TIMEOUT = 5.0   # seconds to wait for web_api services
POLL = 0.1              # how often waiting calls check the stop request
VELOCITY_REPEAT = 0.2   # web_api velocities expire after 0.5 s: repeat the last one
RANGEFINDER_TOPIC = '/rangefinder/range'
RC_TOPIC = '/mavros/mavros/rc/in'

# Functions of the ROS 1 API that have no ROS 2 backend yet. The blocks runner
# refuses programs that use them before takeoff, not in the middle of a flight.
UNSUPPORTED = {
    'navigate_global': 'navigation in global coordinates (flight/navigate_to has no GPS goal yet)',
    'set_attitude': 'attitude setpoints (no flight_manager service yet)',
    'set_rates': 'angular rate setpoints (no flight_manager service yet)',
    'set_effect': 'LED strip effects (the LED driver is not ported to technic_ros2_ws)',
    'set_leds': 'LED strip control (the LED driver is not ported to technic_ros2_ws)',
    'get_led_count': 'LED strip state (the LED driver is not ported to technic_ros2_ws)',
}


class Stop(Exception):
    """The program was stopped from the blocks page."""


class NotSupported(RuntimeError):
    pass


_lock = threading.Lock()
_stop = threading.Event()
_ros = None


def request_stop():
    _stop.set()


def reset_stop():
    _stop.clear()


def check_stop():
    if _stop.is_set():
        raise Stop()


class _Ros:
    """The hidden node, its executor thread and the web_api clients."""

    def __init__(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_srvs.srv import Trigger
        from technic_web.srv import GetTelemetry, Navigate, SetVelocity

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('technic_program_%d' % os.getpid())
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self._spin, daemon=True).start()

        self.types = {'get_telemetry': GetTelemetry, 'navigate': Navigate,
                      'set_velocity': SetVelocity, 'land': Trigger}
        self.clients = {name: self.node.create_client(t, '/' + name) for name, t in self.types.items()}

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:  # shutdown while spinning
            pass

    def call(self, name, **fields):
        client = self.clients[name]
        deadline = time.monotonic() + SERVICE_TIMEOUT
        while not client.service_is_ready():
            check_stop()
            if time.monotonic() > deadline:
                raise RuntimeError('service /%s is not available: start technic_web '
                                   '(web.launch.py) with flight_manager' % name)
            time.sleep(POLL)
        request = self.types[name].Request()
        for key, value in fields.items():
            setattr(request, key, value)
        future = client.call_async(request)
        deadline = time.monotonic() + SERVICE_TIMEOUT
        while not future.done():
            check_stop()
            if time.monotonic() > deadline:
                raise RuntimeError('service /%s did not answer in %.0f s' % (name, SERVICE_TIMEOUT))
            time.sleep(0.01)
        return future.result()


def _get_ros():
    global _ros
    with _lock:
        if _ros is None:
            _ros = _Ros()
        return _ros


# ------------------------------------------------------------------ flight

def get_telemetry(frame_id=''):
    """Telemetry as ROS 1 simple_offboard: x, y, z, yaw in frame_id, vx..., armed, mode, voltage...

    When frame_id is not available in TF (e.g. the marker map is not visible),
    x, y, z and yaw are NaN, as in ROS 1.
    """
    res = _get_ros().call('get_telemetry', frame_id=str(frame_id))
    if frame_id and not res.frame_id:
        res.x = res.y = res.z = res.yaw = math.nan
    return res


def navigate(x=0.0, y=0.0, z=0.0, yaw=0.0, speed=0.5, frame_id='', auto_arm=False):
    """Fly to a point (returns once the flight manager accepted the goal)."""
    stop_velocity()
    res = _get_ros().call('navigate', x=float(x), y=float(y), z=float(z), yaw=float(yaw),
                          speed=float(speed), frame_id=str(frame_id), auto_arm=bool(auto_arm))
    if res.success:
        # Let the navigate_target TF (50 Hz) catch up, so that a following
        # get_telemetry(frame_id='navigate_target') does not see the old target.
        sleep(0.1)
    return res


def land():
    stop_velocity()
    return _get_ros().call('land')


# The last velocity command, repeated by a background thread: flight_manager keeps a
# velocity only for a short time (valid_for), ROS 1 kept it until the next command.
# When the program ends or is stopped the repetition stops and the vehicle holds.
_velocity = None
_velocity_thread = None


def set_velocity(vx=0.0, vy=0.0, vz=0.0, yaw=math.nan, frame_id='body', auto_arm=False):
    """Fly with a velocity (m/s) in frame_id until another command or the program end."""
    global _velocity, _velocity_thread
    fields = dict(vx=float(vx), vy=float(vy), vz=float(vz), yaw=float(yaw),
                  frame_id=str(frame_id), auto_arm=bool(auto_arm))
    res = _get_ros().call('set_velocity', **fields)
    with _lock:
        if not res.success:
            _velocity = None
            return res
        # body yaw is relative to the current heading: repeat it as "keep heading"
        _velocity = dict(fields, yaw=math.nan) if frame_id == 'body' and math.isfinite(fields['yaw']) \
            else fields
        if _velocity_thread is None:
            _velocity_thread = threading.Thread(target=_repeat_velocity, daemon=True)
            _velocity_thread.start()
    return res


def stop_velocity():
    """Stop repeating the last set_velocity (the vehicle holds once it expires)."""
    global _velocity
    with _lock:
        _velocity = None


def _repeat_velocity():
    global _velocity, _velocity_thread
    while True:
        time.sleep(VELOCITY_REPEAT)
        with _lock:
            fields = _velocity
            if fields is None or _stop.is_set():
                # under the lock, so set_velocity starts a new thread if needed
                _velocity_thread = None
                return
        try:
            res = _get_ros().call('set_velocity', **fields)
        except Exception:  # Stop, service gone: let the velocity expire
            res = None
        if res is None or not res.success:
            with _lock:
                if _velocity is fields:
                    _velocity = None


# ------------------------------------------------------------------ time

def sleep(duration):
    """time.sleep that can be interrupted by the stop request."""
    until = time.monotonic() + float(duration)
    while True:
        check_stop()
        left = until - time.monotonic()
        if left <= 0:
            return
        time.sleep(min(left, POLL))


def get_time():
    return time.time()


def is_shutdown():
    """rospy.is_shutdown(). A stop request is not a shutdown: sleep() raises Stop instead."""
    import rclpy
    return _ros is not None and not rclpy.ok()


# ------------------------------------------------------------------ topics

def wait_for_message(topic, msg_type, timeout=None):
    """Return the next message of a topic (rospy.wait_for_message)."""
    from rclpy.qos import qos_profile_sensor_data

    ros = _get_ros()
    received = []
    event = threading.Event()

    def callback(msg):
        if not received:
            received.append(msg)
            event.set()

    sub = ros.node.create_subscription(msg_type, topic, callback, qos_profile_sensor_data)
    try:
        deadline = None if timeout is None else time.monotonic() + timeout
        while not event.wait(POLL):
            check_stop()
            if deadline is not None and time.monotonic() > deadline:
                raise RuntimeError('no message on %s in %.0f s' % (topic, timeout))
    finally:
        ros.node.destroy_subscription(sub)
    return received[0]


def get_range(timeout=5.0):
    """Distance of the VL53L1X rangefinder (vl53l1x node "rangefinder"), metres."""
    from sensor_msgs.msg import Range
    return wait_for_message(RANGEFINDER_TOPIC, Range, timeout).range


def get_rc_channel(channel, timeout=5.0):
    """Raw RC channel value from mavros (needs the mavros backend)."""
    try:
        from mavros_msgs.msg import RCIn
    except ImportError:
        raise NotSupported('RC input needs mavros_msgs (ros-jazzy-mavros-msgs)')
    return wait_for_message(RC_TOPIC, RCIn, timeout).channels[int(channel)]


# ------------------------------------------------------------------ LED (not ported)

class LEDState:
    def __init__(self, index=0, r=0, g=0, b=0):
        self.index, self.r, self.g, self.b = index, r, g, b


def set_effect(*args, **kwargs):
    raise NotSupported('set_effect: ' + UNSUPPORTED['set_effect'])


def set_leds(*args, **kwargs):
    raise NotSupported('set_leds: ' + UNSUPPORTED['set_leds'])


def get_led_count():
    raise NotSupported('get_led_count: ' + UNSUPPORTED['get_led_count'])
