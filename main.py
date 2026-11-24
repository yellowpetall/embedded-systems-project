#!/usr/bin/env python3
"""
Smart Parking - entry point.

    python3 main.py                    # real hardware, uses lot.json
    python3 main.py big_lot.json       # any other lot description
    PARKING_MOCK=1 python3 main.py     # no hardware: simulate on a laptop

Dashboard: http://<pi-ip>:5000
"""
import json
import os
import sys
import threading
import time

from flask import Flask, jsonify, Response

from controller import ParkingController
from io_backends import IO
from lcd import LcdDisplay

HERE = os.path.dirname(os.path.abspath(__file__))
MOCK = os.environ.get("PARKING_MOCK") == "1"


def build(config_path, mock=MOCK):
    with open(config_path, encoding="utf-8") as f:
        cfg = json.load(f)
    if mock:
        from gpiozero import Device
        from gpiozero.pins.mock import MockFactory, MockPWMPin
        Device.pin_factory = MockFactory(pin_class=MockPWMPin)
    io = IO(cfg.get("expanders", []), mock=mock)
    ctrl = ParkingController(cfg, io)
    lcd = LcdDisplay(cfg.get("lcd", {"enabled": False}), mock=mock)
    return cfg, ctrl, lcd


def make_app(cfg, ctrl, lcd):
    app = Flask(__name__)

    @app.route("/")
    def index():
        with open(os.path.join(HERE, "dashboard.html"), encoding="utf-8") as f:
            return Response(f.read(), mimetype="text/html")

    @app.route("/api/layout")
    def layout():
        return jsonify(ctrl.layout_json())

    @app.route("/api/status")
    def status():
        snap = ctrl.snapshot()
        snap["lcd"] = lcd.lines
        snap["mock"] = ctrl.io.mock
        return jsonify(snap)

    if ctrl.io.mock:
        def pulse(ref, seconds=1.5):
            ctrl.io.drive(ref, False)                     # LOW = car detected
            threading.Timer(seconds, ctrl.io.drive, (ref, True)).start()

        @app.route("/api/mock/slot/<sid>", methods=["POST"])
        def mock_slot(sid):
            ref = ctrl.by_id[sid].spec.sensor
            backend, pin = ctrl.io._resolve(ref)
            ctrl.io.drive(ref, not backend.read(pin))     # toggle
            return jsonify(ok=True)

        @app.route("/api/mock/entrance", methods=["POST"])
        def mock_entrance():
            pulse(cfg["entrance"]["sensor"])
            return jsonify(ok=True)

        @app.route("/api/mock/exit", methods=["POST"])
        def mock_exit():
            if cfg.get("exit", {}).get("sensor"):
                pulse(cfg["exit"]["sensor"])
            return jsonify(ok=True)

    return app


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "lot.json")
    cfg, ctrl, lcd = build(path)
    stop = threading.Event()
    threading.Thread(target=ctrl.run, args=(stop,), daemon=True).start()
    threading.Thread(target=lcd.run, args=(ctrl, stop), daemon=True).start()
    print(f"{cfg.get('name', 'Parking lot')}: {len(ctrl.slots)} slots. "
          f"Dashboard on http://<this-device-ip>:5000" + ("  [MOCK]" if MOCK else ""))
    try:
        make_app(cfg, ctrl, lcd).run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        stop.set()
        time.sleep(0.4)
        lcd.close()
        ctrl.close()


if __name__ == "__main__":
    main()
