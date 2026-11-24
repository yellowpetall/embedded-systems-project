#!/usr/bin/env python3
"""
Smart Parking Lot with LED guidance, gates and LCD - Raspberry Pi 2 B
---------------------------------------------------------------------
- 6 slots, each with an IR sensor and a red/green LED pair driven from ONE pin
  (HIGH = red/occupied, LOW = green/vacant, input = both off).
- Entrance (IR sensor or push button): if there is space, a buzzer beeps, the
  entrance gate (servo) opens and the driver is guided to the nearest vacant
  slot with running lane LEDs + an arrow LED + the target slot blinking green.
  If the lot is full, the gate stays closed and a long alarm sounds.
- Exit gate opens automatically when a slot becomes vacant (a car is leaving).
- A 16x2 LCD shows free / occupied counts and where to park.
- A live dashboard is served on http://<pi-ip>:5000 (open it on the laptop).

Run on the Pi:      python3 parking.py
Test on a laptop:   PARKING_MOCK=1 python3 parking.py   (no hardware needed)
"""

import os
import threading
import time
from collections import deque

from flask import Flask, jsonify, Response
from gpiozero import Device, DigitalInputDevice, LED, AngularServo, Buzzer

MOCK = os.environ.get("PARKING_MOCK") == "1"
if MOCK:
    from gpiozero.pins.mock import MockFactory, MockPWMPin
    Device.pin_factory = MockFactory(pin_class=MockPWMPin)

# ============================ CONFIG (BCM numbers) ============================
#            [A1]   [A2]   [A3]        <- row A (arrow LED "A")
#   IN > L0 - L1 --- L2 --- L3
#            [B1]   [B2]   [B3]        <- row B (arrow LED "B")
#
#  name: (sensor_pin, red/green_led_pin, lane_position, side)
SLOTS = {
    "A1": (17, 18, 1, "A"),
    "A2": (27, 23, 2, "A"),
    "A3": (22, 24, 3, "A"),
    "B1": (5, 25, 1, "B"),
    "B2": (6, 8, 2, "B"),
    "B3": (26, 7, 3, "B"),
}
ENTRANCE_PIN = 4                      # 7th IR sensor OR push button to GND
LANE_PINS = [14, 20, 21, 19]          # L0 (entrance), L1, L2, L3
ARROW_PINS = {"A": 10, "B": 9}
ENTRANCE_SERVO_PIN = 12
EXIT_SERVO_PIN = 13
ENTRANCE_BUZZER_PIN = 16
EXIT_BUZZER_PIN = 11

SENSOR_ACTIVE_LOW = True   # IR modules (and buttons to GND) read LOW = detected
SLOT_DEBOUNCE_S = 0.5
ENTRANCE_DEBOUNCE_S = 0.2
GUIDE_TIMEOUT_S = 30
CHASE_STEP_S = 0.15
TICK_S = 0.02

GATE_CLOSED_ANGLE = 0
GATE_OPEN_ANGLE = 90
SERVO_SETTLE_S = 0.6        # time to reach the angle, then stop the signal
ENTRANCE_CLOSE_AFTER_S = 2  # close once the entrance has been clear this long
EXIT_OPEN_S = 6             # exit gate stays open this long

# --- 16x2 character LCD ---
LCD_MODE = "i2c"            # "i2c" | "gpio" | "none"
LCD_I2C_ADDRESS = 0x27      # check with: i2cdetect -y 1
LCD_COLS, LCD_ROWS = 16, 2
LCD_GPIO_PINS = {"rs": 15, "e": 2, "data": [3, 14, 20, 21]}  # only if no backpack
LCD_REFRESH_S = 0.3
# ==============================================================================


class DebouncedSensor:
    """Ignores flickers: a new state only counts after it is stable for hold_s."""

    def __init__(self, pin, hold_s):
        self.dev = DigitalInputDevice(pin, pull_up=SENSOR_ACTIVE_LOW)
        self.hold_s = hold_s
        self.state = self.dev.is_active
        self._changing_since = None

    def update(self, now):
        raw = self.dev.is_active
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


class BicolorLED:
    """Red + green LED on ONE pin.

    Wiring:  Pi 3.3V -> GREEN -> 220R -> PIN -> 220R -> RED -> GND
      pin HIGH  -> red on      pin LOW -> green on      pin input -> both off
    """

    def __init__(self, pin_number):
        self.pin = Device.pin_factory.pin(pin_number)
        self.color = None
        self.set("off")

    def set(self, color):
        if color == self.color:
            return                      # don't touch the pin if nothing changed
        if color == "off":
            self.pin.function = "input"
            self.pin.pull = "floating"
        else:
            self.pin.function = "output"
            self.pin.state = 1 if color == "red" else 0
        self.color = color

    def close(self):
        self.set("off")
        self.pin.close()


class Gate:
    """A servo barrier. Moves, then stops sending pulses so it doesn't jitter."""

    def __init__(self, pin):
        self.servo = AngularServo(pin, min_angle=0, max_angle=180,
                                  min_pulse_width=0.0005,
                                  max_pulse_width=0.0024)
        self.is_open = True             # force the first close() to move it
        self.moved_at = 0.0
        self.close(time.monotonic())

    def _move(self, angle, now):
        self.servo.angle = angle
        self.moved_at = now

    def open(self, now):
        if not self.is_open:
            self.is_open = True
            self._move(GATE_OPEN_ANGLE, now)

    def close(self, now):
        if self.is_open:
            self.is_open = False
            self._move(GATE_CLOSED_ANGLE, now)

    def tick(self, now):
        if self.servo.value is not None and now - self.moved_at > SERVO_SETTLE_S:
            self.servo.detach()        # stop the pulses -> no humming/jitter


class Slot:
    def __init__(self, name, sensor_pin, led_pin, position, side):
        self.name, self.position, self.side = name, position, side
        self.sensor = DebouncedSensor(sensor_pin, SLOT_DEBOUNCE_S)
        self.led = BicolorLED(led_pin)

    @property
    def occupied(self):
        return self.sensor.state


class LcdDisplay:
    """Writes to the LCD from its own thread (LCD writes are slow)."""

    def __init__(self):
        self.lines = ["", ""]
        self.lcd = None
        if MOCK or LCD_MODE == "none":
            return
        try:
            if LCD_MODE == "i2c":
                from RPLCD.i2c import CharLCD
                self.lcd = CharLCD("PCF8574", LCD_I2C_ADDRESS,
                                   cols=LCD_COLS, rows=LCD_ROWS)
            elif LCD_MODE == "gpio":
                from RPLCD.gpio import CharLCD
                import RPi.GPIO as GPIO
                p = LCD_GPIO_PINS
                self.lcd = CharLCD(numbering_mode=GPIO.BCM, cols=LCD_COLS,
                                   rows=LCD_ROWS, pin_rs=p["rs"], pin_e=p["e"],
                                   pins_data=p["data"])
            self.lcd.clear()
        except Exception as e:
            print(f"[LCD] disabled: {e}")
            self.lcd = None

    def show(self, line1, line2):
        new = [line1.ljust(LCD_COLS)[:LCD_COLS], line2.ljust(LCD_COLS)[:LCD_COLS]]
        if new == self.lines:
            return
        self.lines = new
        if self.lcd:
            try:
                for row, text in enumerate(new):
                    self.lcd.cursor_pos = (row, 0)
                    self.lcd.write_string(text)
            except OSError as e:
                print(f"[LCD] write failed: {e}")
        elif MOCK:
            print(f"[LCD] |{new[0]}|  |{new[1]}|")

    def close(self):
        if self.lcd:
            try:
                self.lcd.close(clear=True)
                if LCD_MODE == "i2c":
                    self.lcd.backlight_enabled = False
            except OSError:
                pass


class ParkingController:
    def __init__(self):
        self.slots = {n: Slot(n, *cfg) for n, cfg in SLOTS.items()}
        self.entrance = DebouncedSensor(ENTRANCE_PIN, ENTRANCE_DEBOUNCE_S)
        self.lane = [LED(p) for p in LANE_PINS]
        self.arrows = {side: LED(p) for side, p in ARROW_PINS.items()}
        self.entrance_gate = Gate(ENTRANCE_SERVO_PIN)
        self.exit_gate = Gate(EXIT_SERVO_PIN)
        self.entrance_buzzer = Buzzer(ENTRANCE_BUZZER_PIN)
        self.exit_buzzer = Buzzer(EXIT_BUZZER_PIN)

        self.lock = threading.Lock()
        self.now = 0.0
        self.target = None
        self.guide_started = 0.0
        self.full_until = 0.0
        self.entrance_last_seen = 0.0
        self.exit_close_at = 0.0
        self.log = deque(maxlen=15)
        self._log("System started")

    # ---------------------------------------------------------------- logic
    def _log(self, msg):
        self.log.appendleft(f"{time.strftime('%H:%M:%S')}  {msg}")

    def free_slots(self):
        return [s for s in self.slots.values() if not s.occupied]

    def pick_slot(self):
        free = [s for s in self.free_slots() if s.name != self.target]
        if not free:
            return None
        return min(free, key=lambda s: (s.position, s.side))

    def car_at_entrance(self, now):
        slot = self.pick_slot()
        if slot is None:
            self.full_until = now + 5
            self.entrance_buzzer.beep(on_time=1.0, off_time=0.2, n=1)
            self._log("Car at entrance - LOT FULL, gate stays closed")
            return
        self.target = slot.name
        self.guide_started = now
        self.entrance_gate.open(now)
        self.entrance_buzzer.beep(on_time=0.1, off_time=0.1, n=2)
        self._log(f"Car at entrance - gate open, guiding to {slot.name}")

    def car_leaving(self, slot, now):
        self.exit_gate.open(now)
        self.exit_close_at = now + EXIT_OPEN_S
        self.exit_buzzer.beep(on_time=0.1, off_time=0.1, n=1)
        self._log(f"Car left {slot.name} - exit gate open")

    def tick(self, now):
        with self.lock:
            self.now = now

            # 1) slot sensors
            for slot in self.slots.values():
                if slot.sensor.update(now):
                    if slot.occupied:
                        self._log(f"{slot.name} is now occupied")
                        if slot.name == self.target:
                            self._log(f"Car parked in {slot.name} - guidance done")
                            self.target = None
                    else:
                        self._log(f"{slot.name} is now vacant")
                        self.car_leaving(slot, now)

            # 2) entrance
            if self.entrance.update(now) and self.entrance.state:
                self.car_at_entrance(now)
            if self.entrance.state:
                self.entrance_last_seen = now
            if (self.entrance_gate.is_open and not self.entrance.state
                    and now - self.entrance_last_seen > ENTRANCE_CLOSE_AFTER_S):
                self.entrance_gate.close(now)
                self._log("Entrance gate closed")

            # 3) exit gate timer
            if self.exit_gate.is_open and now >= self.exit_close_at:
                self.exit_gate.close(now)
                self._log("Exit gate closed")

            # 4) guidance timeout
            if self.target and now - self.guide_started > GUIDE_TIMEOUT_S:
                self._log(f"Guidance to {self.target} timed out")
                self.target = None

            self.entrance_gate.tick(now)
            self.exit_gate.tick(now)
            self._render(now)

    def _render(self, now):
        blink = int(now * 6) % 2 == 0

        for slot in self.slots.values():
            if slot.name == self.target:
                slot.led.set("green" if blink else "off")
            else:
                slot.led.set("red" if slot.occupied else "green")

        if self.target:
            t = self.slots[self.target]
            step = int((now - self.guide_started) / CHASE_STEP_S) % (t.position + 1)
            for i, led in enumerate(self.lane):
                led.value = (i == step)
            for side, led in self.arrows.items():
                led.value = (side == t.side)
        elif now < self.full_until:
            for led in self.lane:
                led.value = blink
            for led in self.arrows.values():
                led.off()
        else:
            for led in self.lane + list(self.arrows.values()):
                led.off()

    def run(self, stop):
        while not stop.is_set():
            self.tick(time.monotonic())
            time.sleep(TICK_S)

    # ------------------------------------------------------------- LCD text
    def lcd_text(self):
        with self.lock:
            free = len(self.free_slots())
            occupied = len(self.slots) - free
            line1 = f"Free:{free} Occ:{occupied}"
            if self.now < self.full_until:
                line2 = "  LOT FULL :(  "
            elif self.target:
                line2 = f"Go to {self.target} -->"
            elif free == 0:
                line2 = "No free spaces"
            else:
                line2 = f"Total: {len(self.slots)} spaces"
            return line1, line2

    def run_lcd(self, lcd, stop):
        while not stop.is_set():
            lcd.show(*self.lcd_text())
            time.sleep(LCD_REFRESH_S)

    # ------------------------------------------------------------ dashboard
    def snapshot(self):
        with self.lock:
            return {
                "slots": {n: {"occupied": s.occupied, "position": s.position,
                              "side": s.side, "led": s.led.color}
                          for n, s in self.slots.items()},
                "free": len(self.free_slots()),
                "total": len(self.slots),
                "target": self.target,
                "lane": [led.is_lit for led in self.lane],
                "arrows": {k: v.is_lit for k, v in self.arrows.items()},
                "gates": {"entrance": self.entrance_gate.is_open,
                          "exit": self.exit_gate.is_open},
                "full": self.now < self.full_until,
                "log": list(self.log),
                "lcd": list(lcd.lines),
                "mock": MOCK,
            }


# ================================ WEB SERVER =================================
app = Flask(__name__)
ctrl = ParkingController()
lcd = LcdDisplay()


@app.route("/")
def index():
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "dashboard.html"), encoding="utf-8") as f:
        return Response(f.read(), mimetype="text/html")


@app.route("/api/status")
def status():
    return jsonify(ctrl.snapshot())


if MOCK:
    def _pin(n):
        return Device.pin_factory.pin(n)

    @app.route("/api/mock/slot/<name>", methods=["POST"])
    def mock_slot(name):
        pin = _pin(SLOTS[name][0])
        pin.drive_high() if pin.state == 0 else pin.drive_low()
        return jsonify(ok=True)

    @app.route("/api/mock/arrive", methods=["POST"])
    def mock_arrive():
        pin = _pin(ENTRANCE_PIN)
        pin.drive_low()
        threading.Timer(1.5, pin.drive_high).start()
        return jsonify(ok=True)


if __name__ == "__main__":
    stop = threading.Event()
    threading.Thread(target=ctrl.run, args=(stop,), daemon=True).start()
    threading.Thread(target=ctrl.run_lcd, args=(lcd, stop), daemon=True).start()
    print("Dashboard: http://<this-device-ip>:5000" + ("  [MOCK MODE]" if MOCK else ""))
    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        stop.set()
        time.sleep(LCD_REFRESH_S)
        lcd.close()
