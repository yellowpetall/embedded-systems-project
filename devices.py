"""Small building blocks: debounced sensors, spot LEDs, lamps, buzzers, gates."""


class DebouncedInput:
    """A sensor reading only counts once it has been stable for hold_s seconds."""

    def __init__(self, pin, hold_s, active_low=True):
        self.pin, self.hold_s, self.active_low = pin, hold_s, active_low
        self.state = self._raw()
        self._changing_since = None

    def _raw(self):
        return self.pin.read() != self.active_low   # True = something detected

    def update(self, now):
        """Returns True only on the tick where the stable state changes."""
        raw = self._raw()
        if raw == self.state:
            self._changing_since = None
            return False
        if self._changing_since is None:
            self._changing_since = now
        if now - self._changing_since >= self.hold_s:
            self.state = raw
            self._changing_since = None
            return True
        return False


class BicolorOnePin:
    """Red + green LED on ONE pin:  3.3V -> GREEN -> R -> PIN -> R -> RED -> GND.
    HIGH = red, LOW = green, high impedance = both off."""

    def __init__(self, out):
        self.out, self.color = out, None
        self.set("off")

    def set(self, color):
        if color != self.color:
            self.out.set({"red": True, "green": False, "off": None}[color])
            self.color = color


class BicolorTwoPin:
    """Red and green LEDs on separate pins (easy on expanders with spare pins)."""

    def __init__(self, red, green):
        self.red, self.green, self.color = red, green, None
        self.set("off")

    def set(self, color):
        if color != self.color:
            self.red.set(color == "red")
            self.green.set(color == "green")
            self.color = color


def make_indicator(io, led_cfg):
    if "pin" in led_cfg:
        return BicolorOnePin(io.output(led_cfg["pin"]))
    return BicolorTwoPin(io.output(led_cfg["red"]), io.output(led_cfg["green"]))


class Lamp:
    """A plain on/off LED that only touches the hardware when it changes."""

    def __init__(self, out):
        self.out, self.lit = out, None
        self.set(False)

    def set(self, on):
        on = bool(on)
        if on != self.lit:
            self.out.set(on)
            self.lit = on


class Beeper:
    """Active buzzer playing a pattern without blocking: [(on_s, off_s), ...]."""

    def __init__(self, out):
        self.lamp = Lamp(out)
        self.steps = []         # list of (until_time, on)

    def play(self, pattern, now):
        t, self.steps = now, []
        for on_s, off_s in pattern:
            self.steps += [(t + on_s, True), (t + on_s + off_s, False)]
            t += on_s + off_s

    def tick(self, now):
        while self.steps and now >= self.steps[0][0]:
            self.steps.pop(0)
        self.lamp.set(self.steps[0][1] if self.steps else False)

    @property
    def active(self):
        return bool(self.lamp.lit)


class Gate:
    """A servo barrier on a Pi GPIO pin (servos need PWM, so they stay on GPIO).
    After moving, the signal is stopped so the servo doesn't hum or jitter."""

    def __init__(self, servo_gpio, settle_s, closed=0, opened=90):
        self.settle_s, self.closed_angle, self.open_angle = settle_s, closed, opened
        self.servo = None
        if servo_gpio is not None:
            from gpiozero import AngularServo
            self.servo = AngularServo(servo_gpio, min_angle=0, max_angle=180,
                                      min_pulse_width=0.0005,
                                      max_pulse_width=0.0024)
        self.is_open = True
        self.moved_at = 0.0
        self.close(0.0)

    def _move(self, angle, now):
        if self.servo:
            self.servo.angle = angle
        self.moved_at = now

    def open(self, now):
        if not self.is_open:
            self.is_open = True
            self._move(self.open_angle, now)

    def close(self, now):
        if self.is_open:
            self.is_open = False
            self._move(self.closed_angle, now)

    def tick(self, now):
        if (self.servo and self.servo.value is not None
                and now - self.moved_at > self.settle_s):
            self.servo.detach()

    def release(self):
        if self.servo:
            self.servo.close()
