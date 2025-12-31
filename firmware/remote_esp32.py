# main.py - REMOTE ESP32 with OLED Display (v4)
# Adafruit ESP32 Feather V2 + 1.3" 128x64 OLED
# Hold button 1.5s to enter temp set mode, click once to exit
# Temperature range: center +/-2F (was +/-3F)
#
# v4 Changes:
#  - Temp set mode: joystick right = increase, left = decrease (Y-axis for 90deg CCW rotation)
# v3 Changes:
#  - Added watchdog feed during temp set mode delays
#  - Added I2C error handling to prevent crashes
#  - More reliable button press transmission

import network
import espnow
from machine import Pin, ADC, WDT, I2C
import time
import struct
import ssd1306

print("REMOTE: Starting with OLED...")

# ==================== CONFIGURATION ====================
BASE_MAC = b'\x14+/\xaf=\xbc'
WIFI_CHANNEL = 1

# Joystick calibration
CENTER_X = 1800
CENTER_Y = 1765
JOY_THRESH = 100
MIN_SEND_INTERVAL_MS = 80

# Temp set mode
TEMP_SET_HOLD_MS = 1500  # Hold button 1.5 seconds to enter
TEMP_RANGE_HALF = 2      # +/-2 degrees from center (was 3)
TEMP_ADJUST_THRESH = 600 # Joystick threshold for temp adjustment

# Button transmission
BUTTON_SEND_COUNT = 2     # Send button press packet this many times
BUTTON_SEND_DELAY_MS = 30 # Delay between repeated sends

# ==================== WATCHDOG ====================
wdt = WDT(timeout=8000)

# ==================== NETWORK SETUP ====================
w0 = network.WLAN(network.STA_IF)
w0.active(True)
w0.config(channel=WIFI_CHANNEL)

e = espnow.ESPNow()
e.active(True)

try:
    e.add_peer(BASE_MAC)
except OSError:
    pass

print("REMOTE: ESP-NOW ready on channel", WIFI_CHANNEL)

# ==================== I2C & OLED SETUP ====================
# Enable STEMMA QT power
print("REMOTE: Enabling STEMMA QT power...")
qt_pwr = Pin(2, Pin.OUT, value=1)
time.sleep_ms(500)  # Increased delay for power stabilization after hard reset

i2c = I2C(0, scl=Pin(20), sda=Pin(22), freq=400000)
devices = i2c.scan()
print("REMOTE: I2C devices:", [hex(d) for d in devices])

oled = None
oled_available = False
oled_error_count = 0
MAX_OLED_ERRORS = 5

def init_oled():
    """Initialize or reinitialize the OLED display."""
    global oled, oled_available, oled_error_count
    try:
        devices = i2c.scan()
        if 0x3D in devices:
            oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3D)
            oled_available = True
            oled_error_count = 0
            print("REMOTE: OLED initialized at 0x3D")
            return True
        else:
            oled_available = False
            print("REMOTE: OLED not found on I2C bus")
            return False
    except Exception as ex:
        oled_available = False
        print("REMOTE: OLED init error:", ex)
        return False

# Initial OLED setup
if devices:
    init_oled()
else:
    print("REMOTE: No I2C devices found - using LEDs only")

# ==================== JOYSTICK SETUP ====================
adc_x = ADC(Pin(34))
adc_y = ADC(Pin(39))
adc_x.atten(ADC.ATTN_11DB)
adc_y.atten(ADC.ATTN_11DB)
adc_x.width(ADC.WIDTH_12BIT)
adc_y.width(ADC.WIDTH_12BIT)

btn = Pin(5, Pin.IN, Pin.PULL_UP)

# ==================== LED SETUP ====================
led_auto = Pin(19, Pin.OUT)  # Green
led_lock = Pin(4, Pin.OUT)   # Blue
led_auto.value(0)
led_lock.value(0)

print("REMOTE: Hardware initialized")

# ==================== STATE VARIABLES ====================
last_sent_x = None
last_sent_y = None
last_sent_btn = None
last_send_ms = time.ticks_ms()

auto_mode = True
servo_locked = False
indoor_temp_f = None
outdoor_temp_f = None
smoke_alert = False
noise_alert = False
temp_center = 70.0  # Default comfort zone center

# Button state tracking
button_pressed = False
button_press_start = 0
button_press_handled = False
temp_set_mode = False
blink_state = False
last_blink_ms = time.ticks_ms()

MAX_SEND_RETRIES = 3

# ==================== HELPER FUNCTIONS ====================
def send_button_press():
    """Send button press with redundancy for reliability."""
    x_now = adc_x.read()
    y_now = adc_y.read()
    temp_center_int = int(temp_center * 10)
    
    # Send button=0 (pressed) multiple times for reliability
    pkt_press = struct.pack('>HHBh', x_now, y_now, 0, temp_center_int)
    
    success = False
    for i in range(BUTTON_SEND_COUNT):
        wdt.feed()
        try:
            e.send(BASE_MAC, pkt_press)
            success = True
            if i < BUTTON_SEND_COUNT - 1:
                time.sleep_ms(BUTTON_SEND_DELAY_MS)
        except OSError:
            pass
    
    # Small delay then send button=1 (released) to complete the edge
    time.sleep_ms(50)
    pkt_release = struct.pack('>HHBh', x_now, y_now, 1, temp_center_int)
    try:
        e.send(BASE_MAC, pkt_release)
    except OSError:
        pass
    
    if success:
        print("REMOTE: Button press sent to BASE ({} packets)".format(BUTTON_SEND_COUNT))
    else:
        print("REMOTE: Button press FAILED to send")

# ==================== DISPLAY FUNCTIONS ====================
def update_display():
    """Update OLED display with current state. Handles I2C errors gracefully."""
    global oled_available, oled_error_count
    
    if not oled_available:
        return
    
    try:
        oled.fill(0)
        
        if temp_set_mode:
            # Temperature set mode with flashing
            oled.text("TEMP SET MODE", 8, 0, 1)
            # Draw horizontal line
            for x in range(128):
                oled.pixel(x, 10, 1)
            
            # Flashing center temperature
            if blink_state:
                temp_str = "{:.0f}F".format(temp_center)
                x_pos = (128 - len(temp_str) * 8) // 2
                oled.text(temp_str, x_pos, 20, 1)
                # Arrow indicators
                oled.text("<", x_pos - 16, 20, 1)
                oled.text(">", x_pos + len(temp_str) * 8 + 8, 20, 1)
            
            # Range display (shown in temp set mode)
            low = temp_center - TEMP_RANGE_HALF
            high = temp_center + TEMP_RANGE_HALF
            range_str = "Range: {:.0f}-{:.0f}F".format(low, high)
            x_pos = (128 - len(range_str) * 8) // 2
            oled.text(range_str, x_pos, 40, 1)
            
            # Instructions
            oled.text("< > Adjust", 24, 55, 1)
        else:
            # Normal mode with alerts
            
            # Alert banner at top (if applicable) - smoke takes priority
            if smoke_alert:
                # Smoke alert - flash it for urgency
                if int(time.ticks_ms() / 250) % 2:
                    # Draw inverted banner
                    for y in range(10):
                        for x in range(128):
                            oled.pixel(x, y, 1)
                    oled.text("SMOKE!", 44, 1, 0)
            elif noise_alert:
                # Noise alert - solid banner
                for y in range(10):
                    for x in range(128):
                        oled.pixel(x, y, 1)
                oled.text("NOISE!", 44, 1, 0)
            else:
                # Normal status line (no alerts)
                lock_txt = "LOCK" if servo_locked else "UNLOCK"
                oled.text(lock_txt, 0, 0, 1)
                
                if auto_mode:
                    oled.text("AUTO", 88, 0, 1)
                else:
                    oled.text("MANUAL", 72, 0, 1)
            
            # Temperature display (dual format) - always at fixed position
            if indoor_temp_f is not None and outdoor_temp_f is not None:
                temp_line = "{:.1f}/{:.1f}".format(indoor_temp_f, outdoor_temp_f)
                x_pos = (128 - len(temp_line) * 8) // 2
                oled.text(temp_line, x_pos, 20, 1)
                
                label_line = "Inside/Outside"
                x_pos = (128 - len(label_line) * 8) // 2
                oled.text(label_line, x_pos, 32, 1)
            elif indoor_temp_f is not None:
                # Only indoor temp available
                temp_str = "{:.1f}F".format(indoor_temp_f)
                x_pos = (128 - len(temp_str) * 8) // 2
                oled.text(temp_str, x_pos, 22, 1)
                oled.text("Indoor", 44, 38, 1)
            else:
                # No temp data
                oled.text("No Temp", 35, 25, 1)
                oled.text("Data", 48, 38, 1)
        
        oled.show()
        oled_error_count = 0
        
    except OSError as ex:
        oled_error_count += 1
        print("REMOTE: OLED error ({}/{}): {}".format(oled_error_count, MAX_OLED_ERRORS, ex))
        
        if oled_error_count >= MAX_OLED_ERRORS:
            print("REMOTE: Too many OLED errors - attempting reinit")
            oled_available = False
            time.sleep_ms(100)
            init_oled()

print("REMOTE: Main loop starting...")

# ==================== MAIN LOOP ====================
while True:
    wdt.feed()
    now_ms = time.ticks_ms()

    # ==================== BUTTON HANDLING ====================
    btn_val = btn.value()
    
    if btn_val == 0 and not button_pressed:
        # Button just pressed
        button_pressed = True
        button_press_start = now_ms
        button_press_handled = False
        
    elif btn_val == 1 and button_pressed:
        # Button just released
        if not button_press_handled:
            press_duration = time.ticks_diff(now_ms, button_press_start)
            
            if temp_set_mode:
                # Already in temp set mode - any press exits
                temp_set_mode = False
                low = temp_center - TEMP_RANGE_HALF
                high = temp_center + TEMP_RANGE_HALF
                print("REMOTE: Temp saved - Center: {:.0f}F, Range: {:.0f}-{:.0f}F".format(
                    temp_center, low, high))
                
            elif press_duration >= TEMP_SET_HOLD_MS:
                # Long press already handled, ignore release
                pass
                
            else:
                # Short press - send button press to BASE (will toggle relay)
                send_button_press()
        
        button_pressed = False
        button_press_handled = False
    
    # Check for long press while still held
    if button_pressed and not temp_set_mode and not button_press_handled:
        if time.ticks_diff(now_ms, button_press_start) >= TEMP_SET_HOLD_MS:
            temp_set_mode = True
            button_press_handled = True
            print("REMOTE: Entering temp set mode - Center: {:.0f}F".format(temp_center))

    # ==================== TEMP SET MODE ====================
    if temp_set_mode:
        # Use Y-axis for temp adjustment (joystick rotated 90deg CCW)
        # Y low value = joystick right (user perspective) = increase temp
        # Y high value = joystick left (user perspective) = decrease temp
        y = adc_y.read()
        diff_y = y - CENTER_Y
        
        # Adjust temperature center with joystick (1F increments)
        # Joystick right (high Y on your setup) = increase temp
        if diff_y > TEMP_ADJUST_THRESH:
            temp_center += 1.0
            if temp_center > 85:
                temp_center = 85
            print("REMOTE: Temp center: {:.0f}F (increased)".format(temp_center))
            # Split the delay into smaller chunks with watchdog feeds
            for _ in range(3):
                wdt.feed()
                time.sleep_ms(50)
        # Joystick left (low Y on your setup) = decrease temp
        elif diff_y < -TEMP_ADJUST_THRESH:
            temp_center -= 1.0
            if temp_center < 55:
                temp_center = 55
            print("REMOTE: Temp center: {:.0f}F (decreased)".format(temp_center))
            # Split the delay into smaller chunks with watchdog feeds
            for _ in range(3):
                wdt.feed()
                time.sleep_ms(50)
        
        # Blink temperature display
        if time.ticks_diff(now_ms, last_blink_ms) > 500:
            blink_state = not blink_state
            last_blink_ms = now_ms
        
        # Update LEDs - both blink alternately in temp set mode
        led_auto.value(1 if blink_state else 0)
        led_lock.value(0 if blink_state else 1)

    # ==================== NORMAL JOYSTICK TRANSMISSION ====================
    else:
        # Update LEDs to show current status
        led_auto.value(1 if auto_mode else 0)
        led_lock.value(1 if servo_locked else 0)
        
        x = adc_x.read()
        y = adc_y.read()
        b = 0 if btn_val == 0 else 1

        # Check if values changed
        changed = False
        if last_sent_x is None:
            changed = True
        else:
            if abs(x - last_sent_x) > JOY_THRESH:
                changed = True
            if abs(y - last_sent_y) > JOY_THRESH:
                changed = True

        # Send if changed
        if changed and time.ticks_diff(now_ms, last_send_ms) > MIN_SEND_INTERVAL_MS:
            temp_center_int = int(temp_center * 10)
            pkt = struct.pack('>HHBh', x, y, 1, temp_center_int)
            
            success = False
            for attempt in range(MAX_SEND_RETRIES):
                try:
                    e.send(BASE_MAC, pkt)
                    success = True
                    break
                except OSError:
                    time.sleep_ms(10)
            
            if success:
                last_sent_x = x
                last_sent_y = y
                last_sent_btn = 1
                last_send_ms = now_ms

    # ==================== RECEIVE STATUS FROM BASE ====================
    host, msg = e.recv(0)
    if msg and host == BASE_MAC and len(msg) >= 7:
        try:
            status_byte, temp_f10_indoor, temp_f10_outdoor, smoke, noise = struct.unpack('>BhhBB', msg[0:7])
            auto_mode = bool(status_byte & 0x01)
            servo_locked = bool(status_byte & 0x02)
            
            if temp_f10_indoor != -32768:
                indoor_temp_f = temp_f10_indoor / 10.0
            else:
                indoor_temp_f = None
            
            if temp_f10_outdoor != -32768:
                outdoor_temp_f = temp_f10_outdoor / 10.0
            else:
                outdoor_temp_f = None
            
            smoke_alert = bool(smoke)
            noise_alert = bool(noise)
        except:
            pass

    # ==================== UPDATE DISPLAY ====================
    update_display()

    time.sleep_ms(50)
