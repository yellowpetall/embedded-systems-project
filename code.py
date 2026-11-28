import RPi.GPIO as GPIO
import time
import os
import fcntl

# =====================================================================
# 1. KONFİGÜRASYON VE PIN TANIMLARI
# =====================================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# I2C LCD Ayarları (0x27 çalışmazsa 0x3f yapın)
I2C_ADDR = 0x27
I2C_BUS = 1
I2C_SLAVE = 0x0703

LCD_CHR = 1
LCD_CMD = 0
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_BACKLIGHT = 0x08
ENABLE = 0b00000100

# Kapı Sensörü ve Servo Pinleri
GATE_TRIG_PIN = 23   # Fiziksel Pin 16
GATE_ECHO_PIN = 24   # Fiziksel Pin 18
SERVO_PIN = 18       # Fiziksel Pin 12

# Park Sensörleri Ortak Trigger Pini
PARK_SHARED_TRIG = 25  # Fiziksel Pin 22

# Ölçeklenebilir Park Yerleri Listesi (4 Yeşil + 4 Kırmızı LED)
# Yeni park yeri eklemek için bu listeye yeni bir sözlük eklemeniz yeterlidir.
parking_spots = [
    {
        "id": "SOL_1",
        "distance": 1,         # Kapıya en yakın sol alt park yeri
        "echo_pin": 17,        # Fiziksel Pin 11
        "red_led_pin": 4,      # Fiziksel Pin 7
        "green_led_pin": 5,    # Fiziksel Pin 29 (SOL_1 önündeki yeşil LED)
        "is_occupied": False,
        "detect_start_time": None
    },
    {
        "id": "SAG_1",
        "distance": 2,         # Sağ alt park yeri
        "echo_pin": 27,        # Fiziksel Pin 13
        "red_led_pin": 14,     # Fiziksel Pin 8
        "green_led_pin": 6,    # Fiziksel Pin 31 (SAG_1 önündeki yeşil LED)
        "is_occupied": False,
        "detect_start_time": None
    },
    {
        "id": "SOL_2",
        "distance": 3,         # Sol üst park yeri
        "echo_pin": 22,        # Fiziksel Pin 15
        "red_led_pin": 15,     # Fiziksel Pin 10
        "green_led_pin": 13,   # Fiziksel Pin 33 (SOL_2 önündeki yeşil LED)
        "is_occupied": False,
        "detect_start_time": None
    },
    {
        "id": "SAG_2",
        "distance": 4,         # Sağ üst park yeri
        "echo_pin": 10,        # Fiziksel Pin 19
        "red_led_pin": 12,     # Fiziksel Pin 32
        "green_led_pin": 19,   # Fiziksel Pin 35 (SAG_2 önündeki yeşil LED)
        "is_occupied": False,
        "detect_start_time": None
    }
]

# Eşik Değerleri
GATE_DETECT_DISTANCE_CM = 12.0   # Kapıda araç algılama mesafesi
SPOT_OCCUPIED_DIST_CM = 10.0     # Park yerinde araç var sayılma mesafesi
PARK_CONFIRM_SECONDS = 5.0       # Park edildi sayılması için gereken bekleme süresi

# Durum Değişkenleri
gate_is_open = False
guiding_spot_id = None
last_lcd_text = ("", "")
i2c_fd = None


# =====================================================================
# 2. PAKETSİZ I2C LCD FONKSİYONLARI
# =====================================================================

def i2c_write_byte(data):
    os.write(i2c_fd, bytes([data]))

def lcd_toggle_enable(bits):
    time.sleep(0.0005)
    i2c_write_byte(bits | ENABLE)
    time.sleep(0.0005)
    i2c_write_byte(bits & ~ENABLE)
    time.sleep(0.0005)

def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT
    i2c_write_byte(bits_high)
    lcd_toggle_enable(bits_high)
    i2c_write_byte(bits_low)
    lcd_toggle_enable(bits_low)

def lcd_init():
    global i2c_fd
    i2c_fd = os.open(f"/dev/i2c-{I2C_BUS}", os.O_RDWR)
    fcntl.ioctl(i2c_fd, I2C_SLAVE, I2C_ADDR)
    lcd_byte(0x33, LCD_CMD)
    lcd_byte(0x32, LCD_CMD)
    lcd_byte(0x06, LCD_CMD)
    lcd_byte(0x0C, LCD_CMD)
    lcd_byte(0x28, LCD_CMD)
    lcd_byte(0x01, LCD_CMD)
    time.sleep(0.005)

def lcd_clear():
    lcd_byte(0x01, LCD_CMD)
    time.sleep(0.005)

def update_lcd(line1, line2):
    global last_lcd_text
    if (line1, line2) != last_lcd_text:
        msg1 = line1.ljust(16, " ")[:16]
        msg2 = line2.ljust(16, " ")[:16]
        lcd_byte(LCD_LINE_1, LCD_CMD)
        for char in msg1:
            lcd_byte(ord(char), LCD_CHR)
        lcd_byte(LCD_LINE_2, LCD_CMD)
        for char in msg2:
            lcd_byte(ord(char), LCD_CHR)
        last_lcd_text = (line1, line2)


# =====================================================================
# 3. DONANIM VE ALGORİTMA FONKSİYONLARI
# =====================================================================

def setup_hardware():
    lcd_init()

    GPIO.setup(GATE_TRIG_PIN, GPIO.OUT)
    GPIO.setup(GATE_ECHO_PIN, GPIO.IN)
    GPIO.setup(PARK_SHARED_TRIG, GPIO.OUT)
    GPIO.output(GATE_TRIG_PIN, GPIO.LOW)
    GPIO.output(PARK_SHARED_TRIG, GPIO.LOW)

    for spot in parking_spots:
        GPIO.setup(spot["echo_pin"], GPIO.IN)
        GPIO.setup(spot["red_led_pin"], GPIO.OUT)
        GPIO.setup(spot["green_led_pin"], GPIO.OUT)
        GPIO.output(spot["red_led_pin"], GPIO.LOW)
        GPIO.output(spot["green_led_pin"], GPIO.LOW)

    GPIO.setup(SERVO_PIN, GPIO.OUT)
    servo = GPIO.PWM(SERVO_PIN, 50)
    servo.start(0)
    return servo


def measure_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    timeout = time.time() + 0.04
    pulse_start = time.time()
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return 999.0

    pulse_end = time.time()
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return 999.0

    distance = (pulse_end - pulse_start) * 17150
    return round(distance, 2)


def set_servo_angle(servo, angle):
    duty = 2.5 + (angle / 18.0)
    GPIO.output(SERVO_PIN, True)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.4)
    GPIO.output(SERVO_PIN, False)
    servo.ChangeDutyCycle(0)


def open_gate(servo):
    global gate_is_open
    if not gate_is_open:
        set_servo_angle(servo, 90)
        gate_is_open = True
        print("[KAPI] Açıldı.")


def close_gate(servo):
    global gate_is_open
    if gate_is_open:
        set_servo_angle(servo, 0)
        gate_is_open = False
        print("[KAPI] Kapandı.")


def turn_off_all_green_leds():
    for spot in parking_spots:
        GPIO.output(spot["green_led_pin"], GPIO.LOW)


def guide_to_spot(spot):
    """Sadece hedef park yerinin önündeki yeşil LED'i yakar."""
    turn_off_all_green_leds()
    GPIO.output(spot["green_led_pin"], GPIO.HIGH)
    print(f"[YONLENDIRME] {spot['id']} önündeki yeşil LED yakıldı.")


def update_spot_status(spot, current_distance):
    status_changed = False
    car_present = (0 < current_distance <= SPOT_OCCUPIED_DIST_CM)

    if car_present:
        if not spot["is_occupied"]:
            if spot["detect_start_time"] is None:
                spot["detect_start_time"] = time.time()
            elif (time.time() - spot["detect_start_time"]) >= PARK_CONFIRM_SECONDS:
                spot["is_occupied"] = True
                GPIO.output(spot["green_led_pin"], GPIO.LOW)  # Yeşil söner
                GPIO.output(spot["red_led_pin"], GPIO.HIGH)   # Kırmızı yanar
                status_changed = True
                print(f"[{spot['id']}] Araç park etti. Yeşil söndü, Kırmızı yandı.")
    else:
        spot["detect_start_time"] = None
        if spot["is_occupied"]:
            spot["is_occupied"] = False
            GPIO.output(spot["red_led_pin"], GPIO.LOW)
            status_changed = True
            print(f"[{spot['id']}] Araç çıkış yaptı. Park yeri boşaldı.")

    return status_changed


def get_occupancy_stats():
    total = len(parking_spots)
    occupied = sum(1 for spot in parking_spots if spot["is_occupied"])
    ratio = int((occupied / total) * 100) if total > 0 else 0
    return occupied, total, ratio


def find_nearest_available_spot():
    nearest_spot = None
    for spot in parking_spots:
        if not spot["is_occupied"]:
            if nearest_spot is None or spot["distance"] < nearest_spot["distance"]:
                nearest_spot = spot
    return nearest_spot


# =====================================================================
# 4. ANA DÖNGÜ
# =====================================================================

servo = setup_hardware()
set_servo_angle(servo, 0)
print("Akıllı Otopark Sistemi Başlatıldı...")

try:
    while True:
        # 1. Park yerlerini kontrol et
        for spot in parking_spots:
            dist = measure_distance(PARK_SHARED_TRIG, spot["echo_pin"])
            changed = update_spot_status(spot, dist)
            time.sleep(0.02)

            if changed and spot["is_occupied"] and guiding_spot_id == spot["id"]:
                turn_off_all_green_leds()
                guiding_spot_id = None

        # 2. Doluluk istatistiklerini hesapla
        occupied, total, ratio = get_occupancy_stats()
        is_full = (occupied == total)

        # 3. Kapı sensörünü kontrol et
        gate_dist = measure_distance(GATE_TRIG_PIN, GATE_ECHO_PIN)
        car_at_gate = (0 < gate_dist <= GATE_DETECT_DISTANCE_CM)

        if car_at_gate:
            if is_full:
                update_lcd("OTOPARK DOLU!", f"Doluluk: %{ratio}")
                close_gate(servo)
            else:
                nearest_spot = find_nearest_available_spot()
                update_lcd("HOSGELDINIZ!", f"Yer: {nearest_spot['id']}")
                open_gate(servo)

                if guiding_spot_id != nearest_spot["id"]:
                    guiding_spot_id = nearest_spot["id"]
                    guide_to_spot(nearest_spot)
        else:
            if gate_is_open:
                time.sleep(1.5)
                close_gate(servo)

            if is_full:
                update_lcd("OTOPARK DOLU!", f"Dolu: {occupied}/{total} (%{ratio})")
            else:
                update_lcd(f"Doluluk: %{ratio}", f"Bos Yer: {total - occupied}/{total}")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nSistem kapatılıyor...")
finally:
    turn_off_all_green_leds()
    for spot in parking_spots:
        GPIO.output(spot["red_led_pin"], GPIO.LOW)
    servo.stop()
    try:
        lcd_clear()
        os.close(i2c_fd)
    except Exception:
        pass
    GPIO.cleanup()
