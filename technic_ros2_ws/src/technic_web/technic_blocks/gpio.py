"""GPIO/PWM for programs (ROS 1 technic.gpio, unchanged): the `gpio` utility of WiringPi / wiringOP."""

import subprocess
import threading

# GPIO/PWM via the `gpio` command line utility (WiringPi / wiringOP).
# Pin numbers are passed to `gpio` as is (wPi numbering by default).
#
# PWM is configured so that one PWM unit equals 1 us:
# range 20000 with clock 24 gives a 20 ms (50 Hz) period, suitable for servos.
PWM_RANGE = 20000
PWM_CLOCK = 24


class GPIOError(RuntimeError):
    pass


_lock = threading.Lock()
_modes = {}  # pin -> 'in' | 'out' | 'pwm'


def run_gpio(*args):
    try:
        return subprocess.run(['gpio', *map(str, args)], check=False,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               universal_newlines=True)
    except FileNotFoundError:
        raise GPIOError("'gpio' utility not found. Install WiringPi / wiringOP.")


def _set_mode(pin, mode):
    pin = int(pin)
    with _lock:
        if _modes.get(pin) == mode:
            return
        run_gpio('mode', pin, mode)
        if mode == 'pwm':
            run_gpio('pwmr', pin, PWM_RANGE)
            run_gpio('pwmc', pin, PWM_CLOCK)
        _modes[pin] = mode


def gpio_read(pin: int) -> int:
    """Read GPIO pin value (0/1)."""
    _set_mode(pin, 'in')
    res = run_gpio('read', int(pin))
    try:
        return 1 if int(res.stdout.strip()) else 0
    except ValueError:
        raise GPIOError('Cannot read GPIO pin {}: {}'.format(pin, (res.stderr or res.stdout).strip()))


def gpio_write(pin: int, level: int) -> None:
    """Set GPIO pin value (0/1)."""
    _set_mode(pin, 'out')
    run_gpio('write', int(pin), 1 if int(level) else 0)


def servo_init(pin: int) -> None:
    """Switch pin to PWM mode and set range/clock for servo control."""
    _set_mode(pin, 'pwm')


def servo_write(pin: int, value: int) -> None:
    """Write raw PWM value (0..PWM_RANGE) to the pin."""
    value = max(0, min(PWM_RANGE, int(value)))
    run_gpio('pwm', int(pin), value)


def set_servo(pin: int, pwm_us: int) -> None:
    """Set servo pulse width in microseconds (typically 500..2500)."""
    servo_init(pin)
    servo_write(pin, pwm_us)


def set_duty_cycle(pin: int, duty_cycle: float) -> None:
    """Set PWM duty cycle 0..1."""
    duty_cycle = max(0.0, min(1.0, float(duty_cycle)))
    servo_init(pin)
    servo_write(pin, round(duty_cycle * PWM_RANGE))
