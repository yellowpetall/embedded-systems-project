#!/usr/bin/env python3
"""
Generate a lot.json for a big parking lot wired through MCP23017 expanders.

    python3 make_lot_config.py --aisles 5 --per-row 20 --out big_lot.json
    -> 5 aisles x 2 rows x 20 spots = 200 spots

Shape (top view):          entrance
                              |
    C0 ── N0_1 ── N0_2 ── ...    aisle 0: row "U" above, row "D" below
    |
    C1 ── N1_1 ── N1_2 ── ...    aisle 1
    |
    ...
Every spot gets a sensor pin and one red/green LED pin; every lane point gets a
guidance LED pin. Pins are handed out 16 per MCP23017 chip: chips 0-7 on I2C
bus 1 (addresses 0x20-0x27), further chips on TCA9548A mux buses 11, 12, ...
"""
import argparse
import json


class PinAllocator:
    """Hands out expander pins in order: mcp0:A0, mcp0:A1, ... mcp0:B7, mcp1:A0 ..."""

    def __init__(self):
        self.count = 0
        self.chips = []

    def next(self):
        chip, idx = divmod(self.count, 16)
        if chip == len(self.chips):
            bus = 1 if chip < 8 else 11 + (chip - 8) // 8
            self.chips.append({"name": f"mcp{chip}", "type": "mcp23017",
                               "bus": bus, "address": hex(0x20 + chip % 8)})
        self.count += 1
        return f"mcp{chip}:{'AB'[idx // 8]}{idx % 8}"


def generate(aisles, per_row):
    pins = PinAllocator()
    nodes, edges, slots = [], [], []
    for k in range(aisles):
        y = 3 * k + 1
        nodes.append({"id": f"C{k}", "led": pins.next(), "x": 0, "y": y})
        if k:
            edges.append([f"C{k - 1}", f"C{k}", 3])          # 3 cells between aisles
        prev = f"C{k}"
        for c in range(1, per_row + 1):
            nid = f"N{k}_{c}"
            nodes.append({"id": nid, "led": pins.next(), "x": c, "y": y})
            edges.append([prev, nid])
            prev = nid
            letter = chr(ord("A") + k)
            for side, dy in (("U", -1), ("D", 1)):
                slots.append({"id": f"{letter}{side}{c:02d}", "node": nid, "side": side,
                              "x": c, "y": y + dy,
                              "sensor": pins.next(), "led": {"pin": pins.next()}})
    return {
        "name": f"Generated lot - {len(slots)} spots",
        "tick_hz": 20,                         # bulk I2C polling: 20 Hz is plenty
        "sensor_active_low": True,
        "expanders": pins.chips,
        "lane": {"entrance": "C0", "nodes": nodes, "edges": edges},
        "slots": slots,
        "entrance": {"sensor": "gpio:4", "servo_gpio": 12, "buzzer": "gpio:16"},
        "exit": {"sensor": "gpio:22", "servo_gpio": 13, "buzzer": "gpio:11"},
        "timing": {"slot_debounce_s": 0.5, "gate_debounce_s": 0.2, "guide_timeout_s": 120,
                   "chase_step_s": 0.12, "gate_close_after_s": 2, "servo_settle_s": 0.6},
        "lcd": {"enabled": True, "i2c_bus": 1, "address": "0x27", "cols": 16, "rows": 2},
    }


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aisles", type=int, default=5)
    ap.add_argument("--per-row", type=int, default=20)
    ap.add_argument("--out", default="big_lot.json")
    a = ap.parse_args()
    cfg = generate(a.aisles, a.per_row)
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=1)
    print(f"{a.out}: {len(cfg['slots'])} spots, {len(cfg['lane']['nodes'])} lane LEDs, "
          f"{len(cfg['expanders'])} MCP23017 chips")
