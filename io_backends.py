"""
Hardware I/O layer.

Every input/output in lot.json is a "pin ref" string:
    gpio:17     -> pin BCM 17 directly on the Raspberry Pi
    mcp0:A3     -> pin A3 of the MCP23017 expander named "mcp0" in lot.json
                   (each MCP23017 has 16 pins: A0-A7 and B0-B7)

The parking logic never talks to hardware directly; it asks this layer for an
InputPin / OutputPin and calls .read() / .set(). That's what lets the same
program drive 4 spots on bare GPIO or hundreds of spots on expander chips.

Scalability trick: expander backends don't talk to the chip on every read or
write. Once per tick, begin_tick() reads ALL inputs of a chip in one I2C
transaction, and end_tick() writes only the chips whose outputs changed.
"""


class Backend:
    """Interface every backend implements."""

    def setup_input(self, pin, pull_up=True): ...
    def setup_output(self, pin): ...
    def read(self, pin) -> bool: ...
    def write(self, pin, value): ...      # True / False / None (= high impedance)
    def begin_tick(self): pass            # bulk-read inputs
    def end_tick(self): pass              # flush changed outputs
    def close(self): pass


# --------------------------------------------------------------------------
class GpioBackend(Backend):
    """Pins directly on the Pi, through gpiozero's pin factory (lgpio on Pi OS)."""

    def __init__(self):
        from gpiozero import Device
        if Device.pin_factory is None:
            Device.ensure_pin_factory()
        self.factory = Device.pin_factory
        self.pins = {}
        self.last = {}

    def _pin(self, pin):
        n = int(pin)
        if n not in self.pins:
            self.pins[n] = self.factory.pin(n)
        return self.pins[n]

    def setup_input(self, pin, pull_up=True):
        p = self._pin(pin)
        p.function = "input"
        p.pull = "up" if pull_up else "down"

    def setup_output(self, pin):
        self.write(pin, False)

    def read(self, pin):
        return bool(self._pin(pin).state)

    def write(self, pin, value):
        if self.last.get(pin, "unset") == value:
            return                                 # unchanged: skip the syscall
        p = self._pin(pin)
        if value is None:                          # high impedance (tri-state)
            p.function = "input"
            p.pull = "floating"
        else:
            if p.function != "output":
                p.function = "output"
            p.state = 1 if value else 0
        self.last[pin] = value

    def close(self):
        for p in self.pins.values():
            try:
                p.close()
            except Exception:
                pass


# --------------------------------------------------------------------------
class Mcp23017Backend(Backend):
    """MCP23017 16-pin I2C expander. Up to 8 per I2C bus (addresses 0x20-0x27).

    For more than 8 chips, add a TCA9548A I2C multiplexer and the Linux overlay
    `dtoverlay=i2c-mux,pca9548` - each mux channel then appears as its own bus
    number (e.g. 11..18), so each chip in lot.json just names its bus.
    """
    IODIRA, GPPUA, GPIOA, OLATA = 0x00, 0x0C, 0x12, 0x14
    _buses = {}

    def __init__(self, bus, address, smbus_factory=None):
        if bus not in Mcp23017Backend._buses:
            if smbus_factory is None:
                from smbus2 import SMBus
                smbus_factory = SMBus
            Mcp23017Backend._buses[bus] = smbus_factory(bus)
        self.bus = Mcp23017Backend._buses[bus]
        self.addr = address
        self.iodir = 0xFFFF       # 1 = input (chip's power-on default)
        self.gppu = 0x0000        # 1 = pull-up on
        self.olat = 0x0000        # output latch
        self.inputs = 0xFFFF
        self.input_mask = 0
        self._written = None      # (iodir, gppu, olat) last sent to the chip

    @staticmethod
    def bit(pin):
        """'A3' -> bit 3, 'B7' -> bit 15."""
        port, num = pin[0].upper(), int(pin[1:])
        if port not in "AB" or not 0 <= num <= 7:
            raise ValueError(f"MCP23017 pin must be A0-A7 or B0-B7, got {pin!r}")
        return 1 << (num + (8 if port == "B" else 0))

    def setup_input(self, pin, pull_up=True):
        b = self.bit(pin)
        self.iodir |= b
        self.gppu = self.gppu | b if pull_up else self.gppu & ~b
        self.input_mask |= b

    def setup_output(self, pin):
        self.write(pin, False)

    def read(self, pin):
        return bool(self.inputs & self.bit(pin))

    def write(self, pin, value):
        b = self.bit(pin)
        if value is None:
            self.iodir |= b
            self.gppu &= ~b
        else:
            self.iodir &= ~b
            self.olat = self.olat | b if value else self.olat & ~b

    def _write16(self, reg, value):
        self.bus.write_i2c_block_data(self.addr, reg, [value & 0xFF, value >> 8])

    def begin_tick(self):
        if self.input_mask:
            lo, hi = self.bus.read_i2c_block_data(self.addr, self.GPIOA, 2)
            self.inputs = lo | (hi << 8)

    def end_tick(self):
        state = (self.iodir, self.gppu, self.olat)
        if state == self._written:
            return                                  # nothing changed: no I2C traffic
        old = self._written or (None, None, None)
        # order matters: set the latch before turning a pin into an output
        if state[2] != old[2]:
            self._write16(self.OLATA, self.olat)
        if state[1] != old[1]:
            self._write16(self.GPPUA, self.gppu)
        if state[0] != old[0]:
            self._write16(self.IODIRA, self.iodir)
        self._written = state


# --------------------------------------------------------------------------
class MockBackend(Backend):
    """No hardware: for testing on a laptop. Inputs idle HIGH (pulled up)."""

    def __init__(self):
        self.inputs = {}
        self.outputs = {}

    def setup_input(self, pin, pull_up=True):
        self.inputs.setdefault(pin, pull_up)

    def setup_output(self, pin):
        self.outputs[pin] = False

    def read(self, pin):
        return self.inputs.get(pin, True)

    def write(self, pin, value):
        self.outputs[pin] = value

    # used by tests / the dashboard's simulation buttons
    def drive(self, pin, value):
        self.inputs[pin] = value


# --------------------------------------------------------------------------
class InputPin:
    def __init__(self, backend, pin):
        self.backend, self.pin = backend, pin

    def read(self):
        return self.backend.read(self.pin)


class OutputPin:
    def __init__(self, backend, pin):
        self.backend, self.pin = backend, pin

    def set(self, value):
        self.backend.write(self.pin, value)


class IO:
    """Turns 'gpio:17' / 'mcp0:A3' strings into pins on the right backend."""

    def __init__(self, expanders, mock=False):
        self.mock = mock
        self.backends = {}
        if mock:
            self.backends["gpio"] = MockBackend()
            for e in expanders:
                self.backends[e["name"]] = MockBackend()
        else:
            self.backends["gpio"] = GpioBackend()
            for e in expanders:
                if e.get("type", "mcp23017") != "mcp23017":
                    raise ValueError(f"Unknown expander type {e['type']!r}")
                self.backends[e["name"]] = Mcp23017Backend(
                    e.get("bus", 1), int(str(e["address"]), 0))

    def _resolve(self, ref):
        try:
            name, pin = ref.split(":", 1)
            return self.backends[name], pin
        except (ValueError, KeyError):
            raise ValueError(f"Bad pin ref {ref!r} (expected e.g. 'gpio:17' or "
                             f"'mcp0:A3', with the expander listed in lot.json)")

    def input(self, ref, pull_up=True):
        backend, pin = self._resolve(ref)
        backend.setup_input(pin, pull_up)
        return InputPin(backend, pin)

    def output(self, ref):
        backend, pin = self._resolve(ref)
        backend.setup_output(pin)
        return OutputPin(backend, pin)

    def drive(self, ref, value):
        """Mock mode only: pretend a sensor reads `value`."""
        backend, pin = self._resolve(ref)
        backend.drive(pin, value)

    def begin_tick(self):
        for b in self.backends.values():
            b.begin_tick()

    def end_tick(self):
        for b in self.backends.values():
            b.end_tick()

    def close(self):
        for b in self.backends.values():
            b.close()
