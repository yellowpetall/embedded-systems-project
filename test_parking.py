"""Run with:  python3 -m pytest -q test_parking.py   (no hardware needed)"""
import json
import os
import time

import pytest
from gpiozero import Device
from gpiozero.pins.mock import MockFactory, MockPWMPin

from controller import ParkingController
from io_backends import IO, Mcp23017Backend
from make_lot_config import generate

HERE = os.path.dirname(os.path.abspath(__file__))


def make(cfg):
    Device.pin_factory = MockFactory(pin_class=MockPWMPin)   # fresh pins per test
    return ParkingController(cfg, IO(cfg.get("expanders", []), mock=True))


class Sim:
    """Drives a controller with a fake clock."""

    def __init__(self, ctrl):
        self.c, self.t = ctrl, 100.0

    def run(self, seconds, dt=0.02):
        end = self.t + seconds
        while self.t < end:
            self.c.tick(self.t)
            self.t += dt

    def car(self, ref, present):
        self.c.io.drive(ref, not present)       # sensors are active LOW

    def park(self, sid, present=True):
        self.car(self.c.by_id[sid].spec.sensor, present)

    def arrive(self):
        ref = self.c.cfg["entrance"]["sensor"]
        self.car(ref, True); self.run(0.4); self.car(ref, False); self.run(0.3)

    def colors(self):
        return {s.id: s.led.color for s in self.c.slots}


@pytest.fixture
def small():
    with open(os.path.join(HERE, "lot.json")) as f:
        ctrl = make(json.load(f))
    sim = Sim(ctrl)
    sim.run(0.5)
    yield sim
    ctrl.close()


# ---------------------------------------------------------------- 4 spots
def test_all_green_at_start(small):
    assert set(small.colors().values()) == {"green"}
    assert small.c.lcd_text() == ("Free:4 Occ:0", "Total: 4 spaces")


def test_parked_car_turns_red_after_debounce(small):
    small.park("A1"); small.run(0.3)
    assert small.colors()["A1"] == "green"          # too short: still debouncing
    small.run(0.4)
    assert small.colors()["A1"] == "red"
    assert small.c.free_count == 3


def test_flicker_is_ignored(small):
    small.park("A2"); small.run(0.1); small.park("A2", False); small.run(1)
    assert not small.c.by_id["A2"].occupied


def test_guidance_to_nearest_free_spot(small):
    small.park("A1"); small.park("B1"); small.run(1)
    small.arrive()
    assert small.c.reservations and list(small.c.reservations) == ["A2"]
    assert small.c.entrance.gate.is_open
    assert small.c.signs["A"].lit and not small.c.signs["B"].lit
    seen_route, seen_led = set(), set()
    for _ in range(20):
        small.run(0.05)
        seen_route |= {n for n, l in small.c.lane.items() if l.lit}
        seen_led.add(small.c.by_id["A2"].led.color)
    assert seen_route == {"L0", "L1", "L2"}          # lights run along the route
    assert seen_led == {"green", "off"}              # target blinks


def test_car_parks_and_gate_closes(small):
    small.arrive()
    small.run(2.5)
    assert not small.c.entrance.gate.is_open
    small.park("A1"); small.run(1)
    assert not small.c.reservations
    assert small.colors()["A1"] == "red"


def test_two_cars_get_different_spots(small):
    small.arrive(); small.run(0.5); small.arrive()
    assert list(small.c.reservations) == ["A1", "B1"]


def test_full_lot_keeps_gate_closed(small):
    for sid in ("A1", "A2", "B1", "B2"):
        small.park(sid)
    small.run(1)
    small.arrive()
    assert not small.c.entrance.gate.is_open
    assert small.c.lcd_text() == ("Free:0 Occ:4", "  LOT FULL :(  ")
    assert small.c.entrance.beeper.active


def test_exit_sensor_opens_and_closes_exit_gate(small):
    ref = small.c.cfg["exit"]["sensor"]
    small.car(ref, True); small.run(0.4)
    assert small.c.exit.gate.is_open
    small.car(ref, False); small.run(2.5)
    assert not small.c.exit.gate.is_open


def test_servo_signal_stops_after_move(small):
    small.arrive(); small.run(1)
    assert small.c.entrance.gate.servo.value is None


def test_bicolor_pin_levels(small):
    backend = small.c.io.backends["gpio"]
    assert backend.outputs["18"] is False              # A1 green = LOW
    small.park("A1"); small.run(1)
    assert backend.outputs["18"] is True               # red = HIGH
    small.park("A1", False); small.run(1); small.arrive()
    states = set()
    for _ in range(10):
        small.run(0.05); states.add(backend.outputs["18"])
    assert None in states                              # blinking uses high impedance


# ------------------------------------------------------------- big lots
def test_big_lot_nearest_spot_and_speed():
    cfg = generate(aisles=5, per_row=20)               # 200 spots, 32 expanders
    ctrl = make(cfg)
    sim = Sim(ctrl)
    sim.run(0.5)
    assert len(ctrl.slots) == 200 and len(cfg["expanders"]) == 32

    sim.arrive()
    assert list(ctrl.reservations) == ["AD01"]         # closest to the entrance
    sim.park("AD01"); sim.park("AU01"); sim.run(1)
    sim.arrive()
    assert list(ctrl.reservations) == ["AD02"]

    # the route really follows the lane graph
    assert ctrl.by_id["CU05"].spec.route == ["C0", "C1", "C2", "N2_1", "N2_2",
                                            "N2_3", "N2_4", "N2_5"]

    t0 = time.perf_counter()
    for i in range(200):
        ctrl.tick(sim.t + i * 0.05)
    per_tick_ms = (time.perf_counter() - t0) / 200 * 1000
    print(f"\n200-slot tick: {per_tick_ms:.2f} ms")
    assert per_tick_ms < 20                            # far below the 50 ms budget at 20 Hz
    ctrl.close()


def test_layout_rejects_bad_config():
    cfg = generate(1, 2)
    cfg["slots"][0]["node"] = "nope"
    with pytest.raises(ValueError, match="unknown node"):
        make(cfg)


# --------------------------------------------------------- MCP23017 driver
class FakeBus:
    def __init__(self):
        self.regs = {}
        self.writes = 0
        self.gpio = [0xFF, 0xFF]

    def write_i2c_block_data(self, addr, reg, data):
        self.regs[reg], self.regs[reg + 1] = data
        self.writes += 1

    def read_i2c_block_data(self, addr, reg, n):
        assert reg == Mcp23017Backend.GPIOA and n == 2
        return list(self.gpio)


def test_mcp23017_registers():
    Mcp23017Backend._buses.clear()
    bus = FakeBus()
    m = Mcp23017Backend(99, 0x20, smbus_factory=lambda n: bus)
    m.setup_input("A0")
    m.setup_output("B3")
    m.write("B3", True)
    m.end_tick()
    assert bus.regs[0x00] == 0xFF and bus.regs[0x01] == 0xF7     # IODIR: B3 output
    assert bus.regs[0x0C] == 0x01                                 # pull-up on A0
    assert bus.regs[0x15] == 0x08                                 # OLATB bit 3 high
    writes = bus.writes
    m.end_tick()
    assert bus.writes == writes                                   # unchanged: no traffic
    bus.gpio = [0xFE, 0xFF]
    m.begin_tick()
    assert m.read("A0") is False and m.read("A1") is True
    m.write("B3", None)                                           # tri-state
    m.end_tick()
    assert bus.regs[0x01] == 0xFF
    Mcp23017Backend._buses.clear()
