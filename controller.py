"""
The parking logic. Knows nothing about wires: it only uses the devices built
from lot.json, so the same code runs 4 spots or 400.
"""
import threading
import time
from collections import deque

from devices import (DebouncedInput, Lamp, Beeper, Gate, make_indicator)
from layout import Layout

DEFAULT_TIMING = {
    "slot_debounce_s": 0.5, "gate_debounce_s": 0.2, "guide_timeout_s": 30,
    "chase_step_s": 0.15, "gate_close_after_s": 2, "servo_settle_s": 0.6,
    "exit_open_s": 6, "full_alarm_s": 5,
}


class Slot:
    __slots__ = ("spec", "sensor", "led")

    def __init__(self, spec, sensor, led):
        self.spec, self.sensor, self.led = spec, sensor, led

    @property
    def id(self):
        return self.spec.id

    @property
    def occupied(self):
        return self.sensor.state


class GatePoint:
    """An entrance or exit: optional sensor, servo gate and buzzer."""

    def __init__(self, io, cfg, timing, active_low):
        self.sensor = (DebouncedInput(io.input(cfg["sensor"]), timing["gate_debounce_s"],
                                      active_low) if cfg.get("sensor") else None)
        self.gate = Gate(cfg.get("servo_gpio"), timing["servo_settle_s"])
        self.beeper = Beeper(io.output(cfg["buzzer"])) if cfg.get("buzzer") else None
        self.last_seen = 0.0
        self.close_at = None

    def beep(self, pattern, now):
        if self.beeper:
            self.beeper.play(pattern, now)


class ParkingController:
    def __init__(self, cfg, io):
        self.cfg, self.io = cfg, io
        self.t = {**DEFAULT_TIMING, **cfg.get("timing", {})}
        active_low = cfg.get("sensor_active_low", True)
        self.layout = Layout(cfg)

        # slots, already sorted nearest-first by the layout
        self.slots = [Slot(spec,
                           DebouncedInput(io.input(spec.sensor), self.t["slot_debounce_s"],
                                          active_low),
                           make_indicator(io, spec.led))
                      for spec in self.layout.slots]
        self.by_id = {s.id: s for s in self.slots}
        self.occupied_count = sum(s.occupied for s in self.slots)   # kept up to date O(1)

        self.lane = {nid: Lamp(io.output(n["led"]))
                     for nid, n in self.layout.nodes.items() if n.get("led")}
        self.signs = {side: Lamp(io.output(ref))
                      for side, ref in cfg.get("side_signs", {}).items()}

        self.entrance = GatePoint(io, cfg["entrance"], self.t, active_low)
        self.exit = GatePoint(io, cfg.get("exit", {}), self.t, active_low)

        self.reservations = {}        # slot id -> time the car was sent there
        self.full_until = 0.0
        self.now = 0.0
        self.lock = threading.Lock()
        self.log = deque(maxlen=20)
        self._log(f"Started: {len(self.slots)} slots, {len(self.lane)} lane LEDs")

    # ------------------------------------------------------------- helpers
    def _log(self, msg):
        self.log.appendleft(f"{time.strftime('%H:%M:%S')}  {msg}")

    @property
    def free_count(self):
        return len(self.slots) - self.occupied_count

    def pick_slot(self):
        """Nearest free slot that no other car is already heading to."""
        for slot in self.slots:                      # nearest first
            if not slot.occupied and slot.id not in self.reservations:
                return slot
        return None

    # --------------------------------------------------------------- events
    def _car_at_entrance(self, now):
        slot = self.pick_slot()
        if slot is None:
            self.full_until = now + self.t["full_alarm_s"]
            self.entrance.beep([(1.0, 0.2)], now)
            self._log("Car at entrance - LOT FULL, gate stays closed")
            return
        self.reservations[slot.id] = now
        self.entrance.gate.open(now)
        self.entrance.beep([(0.1, 0.1), (0.1, 0.1)], now)
        self._log(f"Car at entrance - gate open, guiding to {slot.id}")

    def _slot_changed(self, slot, now):
        if slot.occupied:
            self.occupied_count += 1
            if self.reservations.pop(slot.id, None) is not None:
                self._log(f"Car parked in {slot.id} - guidance done")
            else:
                self._log(f"{slot.id} is now occupied")
        else:
            self.occupied_count -= 1
            self._log(f"{slot.id} is now vacant")
            if self.exit.sensor is None:            # no exit sensor: open on vacancy
                self._open_exit(now, auto_close_at=now + self.t["exit_open_s"])

    def _open_exit(self, now, auto_close_at=None):
        if not self.exit.gate.is_open:
            self.exit.gate.open(now)
            self.exit.beep([(0.1, 0.1)], now)
            self._log("Exit gate open")
        self.exit.close_at = auto_close_at

    def _update_gate_point(self, point, now, on_arrive):
        if point.sensor is None:
            return
        if point.sensor.update(now) and point.sensor.state:
            on_arrive(now)
        if point.sensor.state:
            point.last_seen = now
        elif (point.gate.is_open and point.close_at is None
              and now - point.last_seen > self.t["gate_close_after_s"]):
            point.gate.close(now)
            self._log(f"{'Entrance' if point is self.entrance else 'Exit'} gate closed")

    # ---------------------------------------------------------------- tick
    def tick(self, now):
        with self.lock:
            self.now = now
            self.io.begin_tick()                     # one bulk read per expander

            for slot in self.slots:
                if slot.sensor.update(now):
                    self._slot_changed(slot, now)

            self._update_gate_point(self.entrance, now, self._car_at_entrance)
            self._update_gate_point(self.exit, now, lambda t: self._open_exit(t))
            if self.exit.close_at is not None and now >= self.exit.close_at:
                self.exit.gate.close(now)
                self.exit.close_at = None
                self._log("Exit gate closed")

            for sid, started in list(self.reservations.items()):
                if now - started > self.t["guide_timeout_s"]:
                    del self.reservations[sid]
                    self._log(f"Guidance to {sid} timed out")

            for point in (self.entrance, self.exit):
                point.gate.tick(now)
                if point.beeper:
                    point.beeper.tick(now)

            self._render(now)
            self.io.end_tick()                       # write only what changed

    def _render(self, now):
        blink = int(now * 6) % 2 == 0
        for slot in self.slots:
            if slot.id in self.reservations:
                slot.led.set("green" if blink else "off")
            else:
                slot.led.set("red" if slot.occupied else "green")

        lit, side = set(), None
        if self.reservations:
            # lane LEDs show the route of the most recent car
            sid, started = next(reversed(self.reservations.items()))
            spec = self.by_id[sid].spec
            route = [n for n in spec.route if n in self.lane]
            if route:
                step = int((now - started) / self.t["chase_step_s"]) % len(route)
                lit = {route[step]}
            side = spec.side
        elif now < self.full_until and blink:
            lit = set(self.lane)

        for nid, lamp in self.lane.items():
            lamp.set(nid in lit)
        for s, lamp in self.signs.items():
            lamp.set(s == side)

    def run(self, stop):
        period = 1.0 / self.cfg.get("tick_hz", 50)
        while not stop.is_set():
            self.tick(time.monotonic())
            time.sleep(period)

    # ------------------------------------------------------- LCD / dashboard
    def lcd_text(self):
        with self.lock:
            free = self.free_count
            line1 = f"Free:{free} Occ:{self.occupied_count}"
            if self.now < self.full_until:
                line2 = "  LOT FULL :(  "
            elif self.reservations:
                line2 = f"Go to {next(reversed(self.reservations))} -->"
            elif free == 0:
                line2 = "No free spaces"
            else:
                line2 = f"Total: {len(self.slots)} spaces"
            return line1, line2

    def layout_json(self):
        L = self.layout
        return {
            "name": self.cfg.get("name", "Parking lot"),
            "entrance": L.entrance,
            "nodes": [{"id": n["id"], "x": n.get("x", 0), "y": n.get("y", 0),
                       "led": bool(n.get("led"))} for n in L.nodes.values()],
            "edges": [[a, b] for a in L.adj for b, _ in L.adj[a] if a < b],
            "slots": [{"id": s.id, "x": s.x, "y": s.y, "side": s.side}
                      for s in L.slots],
        }

    def snapshot(self):
        with self.lock:
            return {
                "slots": {s.id: ("r" if s.id in self.reservations else
                                 "o" if s.occupied else "f") for s in self.slots},
                "free": self.free_count, "total": len(self.slots),
                "reservations": list(self.reservations),
                "lane_lit": [n for n, l in self.lane.items() if l.lit],
                "gates": {"entrance": self.entrance.gate.is_open,
                          "exit": self.exit.gate.is_open},
                "full": self.now < self.full_until,
                "log": list(self.log),
            }

    def close(self):
        for point in (self.entrance, self.exit):
            point.gate.release()
        self.io.close()
