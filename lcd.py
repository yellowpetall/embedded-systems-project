"""16x2 (or 20x4) character LCD with an I2C backpack. Runs in its own thread
because LCD writes are slow and must not stutter the LED animation."""
import time


class LcdDisplay:
    def __init__(self, cfg, mock=False):
        self.cols = cfg.get("cols", 16)
        self.rows = cfg.get("rows", 2)
        self.lines = [""] * self.rows
        self.mock = mock
        self.lcd = None
        if mock or not cfg.get("enabled", True):
            return
        try:
            from RPLCD.i2c import CharLCD
            self.lcd = CharLCD("PCF8574", int(str(cfg.get("address", "0x27")), 0),
                               port=cfg.get("i2c_bus", 1),
                               cols=self.cols, rows=self.rows)
            self.lcd.clear()
        except Exception as e:            # no LCD? keep the lot running anyway
            print(f"[LCD] disabled: {e}")

    def show(self, *lines):
        padded = list(lines)[:self.rows] + [""] * (self.rows - len(lines))
        new = [text.ljust(self.cols)[:self.cols] for text in padded]
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
        elif self.mock:
            print("[LCD] " + "  ".join(f"|{l}|" for l in new))

    def run(self, controller, stop, period=0.3):
        while not stop.is_set():
            self.show(*controller.lcd_text())
            time.sleep(period)

    def close(self):
        if self.lcd:
            try:
                self.lcd.close(clear=True)
                self.lcd.backlight_enabled = False
            except OSError:
                pass
