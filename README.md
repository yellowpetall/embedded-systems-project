# Smart Parking (scalable)

LED-guided parking system for a Raspberry Pi. The lot is described in a JSON
file, so the same code runs 4 spots on bare GPIO or hundreds through MCP23017
I/O expanders.

## Run

```bash
sudo apt install python3-flask python3-gpiozero
pip install RPLCD smbus2 --break-system-packages   # LCD + expanders

python3 main.py                   # uses lot.json (the 4-spot model)
python3 main.py big_lot.json      # any other lot
PARKING_MOCK=1 python3 main.py    # laptop simulation, no hardware
```

Dashboard: `http://<pi-ip>:5000`

## Files

| File | What it does |
| --- | --- |
| `lot.json` | The lot: spots, lane graph, pins, gates, timings |
| `main.py` | Starts everything + the web dashboard |
| `controller.py` | Parking logic (guidance, reservations, gates, LCD text) |
| `layout.py` | Lane graph + Dijkstra: nearest spot and the route to it |
| `devices.py` | Debounced sensors, red/green LEDs, lamps, buzzers, servo gates |
| `io_backends.py` | Pin refs `gpio:17` / `mcp0:A3` -> real or simulated hardware |
| `lcd.py` | I2C character LCD |
| `dashboard.html` | Live view that draws any lot from `/api/layout` |
| `make_lot_config.py` | Generates big lots, e.g. `--aisles 5 --per-row 20` = 200 spots |
| `test_parking.py` | `python3 -m pytest -q test_parking.py` |

## Adding a spot

Add one line to `slots` in `lot.json`:

```json
{"id": "A3", "node": "L3", "side": "A", "x": 3, "y": 0, "sensor": "gpio:22", "led": {"pin": "gpio:24"}}
```

and, if it needs a new lane point, a node + an edge in `lane`.

## Scaling past the Pi's pins

Add MCP23017 chips (16 pins each, up to 8 per I2C bus) to `expanders` and use
`mcpN:A0`...`mcpN:B7` refs. More than 8 chips: TCA9548A I2C multiplexer +
`dtoverlay=i2c-mux,pca9548` in `/boot/firmware/config.txt`; each mux channel
becomes its own bus number. Servos stay on Pi GPIO (they need PWM).
