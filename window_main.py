# base_esp32_window.py v3.8
#
# BASE / WINDOW ESP32
#  - Talks to REMOTE ESP32 (joystick + LEDs) and OUTDOOR ESP32 (temp + noise)
#  - Controls window stepper with limit switches
#  - Servo lock mechanism with manual override
#  - AUTO vs MANUAL modes with smoke override
#  - Environmental monitoring (DHT11, PMS5003)
#
# v3.8 Changes:
#  - Reverted to continuous PWM for servo (keeps PWM on to hold position)
#  - Servo was not holding position with PWM off
# v3.7 Changes:
#  - Fixed servo lock grace period timing (timer starts BEFORE servo moves, not after)
#  - Increased grace period from 1s to 2s for more safety margin
#  - Prevents false unlocks from switch bounce or servo engagement forces
# v3.6 Changes:
#  - Servo settle time increased to 700ms before PWM off
#  - Added periodic servo reinforcement pulse every 10 seconds
# v3.5 Changes:
#  - Servo PWM turns off after movement to eliminate whine noise
# v3.4 Changes:
#  - Periodic state send for temperature updates (every 500ms)
#  - Manual Y input interrupts close-and-lock sequence
#  - Improved button debounce with hold-off timer

import network
import espnow
import struct
from machine import Pin, PWM, UART, WDT
import time
import dht

# ==================== CONFIGURATION ====================
# MAC addresses
BASE_MAC = b'\x14+/\xaf=\xbc'   # this board
REMOTE_MAC = b'\x14+/\xae\xb8l'
OUTDOOR_MAC = b'\xf4e\x0b3\x1bl'

# Wi-Fi settings
WIFI_CHANNEL = 1

# Stepper motor settings
MIN_DELAY_US = 500   # Much faster speed (was 800)
MAX_DELAY_US = 2000  # Faster slow speed (was 3000)
IDLE_EN_DELAY_MS = 100

# Joystick settings (calibrated to actual hardware)
CENTER_X = 1800
CENTER_Y = 1765
DEADZONE = 600  # Increased to prevent accidental inputs

# Window position limits
WINDOW_MIN_STEPS = 0    # fully open
WINDOW_MAX_STEPS = 765  # fully closed

# Direction definitions
DIR_TOWARD_CLOSED = 1
DIR_TOWARD_OPEN = 0

# Servo settings
LOCK_ANGLE = 90
UNLOCK_ANGLE = 0
AUTO_LOCK_DELAY_MS = 300  # Delay before auto-locking when closed

# Smoke detection settings
SMOKE_PM25_THRESHOLD = 80  # Increased threshold
SMOKE_PM25_CLEAR = 30
SMOKE_HOLD_MS = 2000
SMOKE_BEEP_INTERVAL_MS = 1250  # Increased beep rate

# Temperature control settings
TEMP_TARGET_LOW = 68.0   # Default comfort range (will be updated from remote)
TEMP_TARGET_HIGH = 72.0  # Default comfort range (will be updated from remote)
TEMP_HYSTERESIS = 1.5  # Hysteresis to prevent oscillation
TEMP_PROACTIVE_DIFF = 5.0  # Close window if outdoor is this many degrees different from comfort zone
TEMP_ACTION_MIN_DIFF = 2.0  # Minimum temp difference between indoor/outdoor to take action
TEMP_RANGE_HALF = 2.0  # +/- 2 degrees from center (was 3)

# DHT reading settings
DHT_INTERVAL_MS = 2000
DHT_MAX_RETRIES = 3

# PMS5003 settings
PMS_INTERVAL_MS = 1000

# Communication timeout
COMM_TIMEOUT_MS = 10000  # 10 seconds

# Retry settings
MAX_SEND_RETRIES = 3

# State send interval (NEW - for periodic temp updates)
STATE_SEND_INTERVAL_MS = 500

# Button debounce settings (NEW)
BUTTON_HOLDOFF_MS = 200  # Minimum time between button toggles

# ==================== WATCHDOG ====================
wdt = WDT(timeout=8000)  # 8 second watchdog

# ==================== NETWORK SETUP ====================
w0 = network.WLAN(network.STA_IF)
w0.active(True)
w0.config(channel=WIFI_CHANNEL)

e = espnow.ESPNow()
e.active(True)
e.add_peer(REMOTE_MAC)
e.add_peer(OUTDOOR_MAC)

print("BASE: ESP-NOW ready on channel", WIFI_CHANNEL)
print("BASE: Joystick center - X:", CENTER_X, "Y:", CENTER_Y, "Deadzone:", DEADZONE)

# ==================== STEPPER SETUP ====================
STEP = Pin(19, Pin.OUT)
DIR = Pin(33, Pin.OUT)
EN = Pin(32, Pin.OUT)   # active LOW
EN.value(1)  # disabled initially

last_step_time = time.ticks_us()
step_interval_us = None
motor_active = False
last_motor_active_ms = time.ticks_ms()
last_dir = DIR_TOWARD_CLOSED
driver_was_disabled = True  # Track if driver needs wake-up

# ==================== LIMIT SWITCHES ====================
left_limit_pin = Pin(5, Pin.IN, Pin.PULL_UP)   # OPEN
right_limit_pin = Pin(7, Pin.IN, Pin.PULL_UP)  # CLOSED

def left_limit_pressed():
    return left_limit_pin.value() == 0

def right_limit_pressed():
    return right_limit_pin.value() == 0

# ==================== POSITION TRACKING ====================
position_steps = (WINDOW_MIN_STEPS + WINDOW_MAX_STEPS) // 2
window_closed = False
last_closed_time_ms = 0

def clamp_position():
    global position_steps
    if position_steps < WINDOW_MIN_STEPS:
        position_steps = WINDOW_MIN_STEPS
    if position_steps > WINDOW_MAX_STEPS:
        position_steps = WINDOW_MAX_STEPS

# ==================== SERVO LOCK ====================
servo_pin = Pin(13)
servo_pwm = None
servo_locked = False
manual_lock_override = False  # User has manually controlled lock state

# Servo timing settings
SERVO_SETTLE_MS = 500           # Time for servo to reach position
SERVO_LOCK_GRACE_MS = 2000      # Grace period after lock before safety check can trigger
last_servo_lock_ms = 0          # Track when servo was last locked (for grace period)

def servo_on():
    global servo_pwm
    if servo_pwm is None:
        servo_pwm = PWM(servo_pin, freq=50)

def servo_off():
    global servo_pwm
    if servo_pwm is not None:
        servo_pwm.deinit()
        servo_pwm = None

def set_servo_angle(angle):
    """Set servo to specific angle and keep PWM active to hold position."""
    global servo_pwm
    angle = max(0, min(180, angle))
    us = 500 + int((angle / 180.0) * 2000)
    if servo_pwm is None:
        servo_on()
    servo_pwm.duty_ns(us * 1000)

def lock_servo(manual=False):
    """Lock the servo. If manual=True, set override flag."""
    global servo_locked, manual_lock_override, last_servo_lock_ms
    if not servo_locked and right_limit_pressed():
        print("BASE: Servo -> LOCK", "(manual)" if manual else "(auto)")
        last_servo_lock_ms = time.ticks_ms()  # Start grace period
        set_servo_angle(LOCK_ANGLE)
        servo_locked = True
        if manual:
            manual_lock_override = True
        # Feed watchdog during settle time
        for _ in range(SERVO_SETTLE_MS // 100):
            wdt.feed()
            time.sleep_ms(100)
        # PWM stays ON to hold position
        print("BASE: Servo at LOCK position - PWM holding")
        send_state()

def unlock_servo(manual=False):
    """Unlock the servo. If manual=True, set override flag."""
    global servo_locked, manual_lock_override
    if servo_locked:
        print("BASE: Servo -> UNLOCK", "(manual)" if manual else "(auto)")
        set_servo_angle(UNLOCK_ANGLE)
        servo_locked = False
        if manual:
            manual_lock_override = True
        # Feed watchdog during settle time
        for _ in range(SERVO_SETTLE_MS // 100):
            wdt.feed()
            time.sleep_ms(100)
        # PWM stays ON to hold position
        print("BASE: Servo at UNLOCK position - PWM holding")
        send_state()

def clear_lock_override():
    """Clear manual override when window moves away from closed position."""
    global manual_lock_override
    if manual_lock_override:
        print("BASE: Clearing manual lock override")
        manual_lock_override = False

# ==================== MODE CONTROL ====================
auto_mode = True

def set_auto_mode(new_state):
    global auto_mode
    if auto_mode != new_state:
        auto_mode = new_state
        print("BASE: AUTO mode:", auto_mode)
        send_state()

def send_state():
    """Send status byte + indoor temp + outdoor temp + alerts to remote"""
    global last_state_send_ms
    byte = (1 if auto_mode else 0) | ((1 if servo_locked else 0) << 1)
    
    # Pack indoor temperature (temp_f * 10 as int16)
    if indoor_temp_f is not None:
        temp_f10_indoor = int(indoor_temp_f * 10)
    else:
        temp_f10_indoor = -32768  # Sentinel for no data
    
    # Pack outdoor temperature
    if outdoor_temp_f is not None:
        temp_f10_outdoor = int(outdoor_temp_f * 10)
    else:
        temp_f10_outdoor = -32768
    
    # Pack alert flags
    smoke_flag = 1 if smoke_triggered else 0
    noise_flag = 1 if outdoor_loud else 0
    
    pkt = struct.pack('>BhhBB', byte, temp_f10_indoor, temp_f10_outdoor, smoke_flag, noise_flag)
    
    for attempt in range(MAX_SEND_RETRIES):
        try:
            e.send(REMOTE_MAC, pkt)
            last_state_send_ms = time.ticks_ms()  # Update timestamp on success
            return
        except OSError:
            time.sleep_ms(10)
    print("BASE: Failed to send state to remote")

# ==================== RELAY ====================
relay = Pin(27, Pin.OUT)
relay_state = 0  # 0 = privacy, 1 = clear
relay.value(relay_state)

def set_relay_state(new_state, reason=""):
    """Set relay state with optional reason logging."""
    global relay_state
    if relay_state != new_state:
        relay_state = new_state
        relay.value(relay_state)
        mode_str = "clear" if relay_state else "privacy"
        if reason:
            print("BASE: PDLC Film ->", mode_str.upper(), "-", reason)
        else:
            print("BASE: PDLC Film ->", mode_str.upper())

last_remote_button = 1
last_button_toggle_ms = 0  # NEW: Track last button toggle time for debounce
last_noise_close_state = False  # Track if we closed due to noise

# ==================== DHT11 (Indoor) ====================
dht_sensor = dht.DHT11(Pin(14))
last_dht_time = time.ticks_ms()
indoor_temp_f = None
indoor_hum = None

def read_dht_with_retry():
    global indoor_temp_f, indoor_hum
    
    for attempt in range(DHT_MAX_RETRIES):
        try:
            dht_sensor.measure()
            t = dht_sensor.temperature()
            h = dht_sensor.humidity()
            indoor_temp_f = t * 9.0 / 5.0 + 32.0
            indoor_hum = h
            return True
        except Exception as ex:
            if attempt == DHT_MAX_RETRIES - 1:
                print("BASE: DHT read failed:", ex)
            time.sleep_ms(100)
    return False

# ==================== PMS5003 ====================
uart = UART(2, baudrate=9600, rx=21, tx=22, timeout=200)
pm25 = None
last_pms_time = time.ticks_ms()

def read_pms_frame():
    b = uart.read(1)
    if not b or b[0] != 0x42:
        return None
    b2 = uart.read(1)
    if not b2 or b2[0] != 0x4D:
        return None
    rest = uart.read(30)
    if not rest or len(rest) != 30:
        return None
    frame = b'\x42\x4D' + rest
    checksum_calc = sum(frame[0:30]) & 0xFFFF
    checksum_frame = (frame[30] << 8) | frame[31]
    if checksum_calc != checksum_frame:
        return None
    data = struct.unpack('>HHHHHHHHHHHHHH', frame[2:32])
    return data[0], data[1], data[2]

def update_pms():
    global pm25
    frame = read_pms_frame()
    if frame is not None:
        _, pm2_5, _ = frame
        pm25 = pm2_5

# ==================== BUZZER ====================
buzzer_pin = Pin(4, Pin.OUT)
buzzer_pwm = None

def buzzer_on(freq=2000, duty=32768):
    global buzzer_pwm
    if buzzer_pwm is None:
        buzzer_pwm = PWM(buzzer_pin)
    buzzer_pwm.freq(freq)
    buzzer_pwm.duty_u16(duty)

def buzzer_off():
    global buzzer_pwm
    if buzzer_pwm is not None:
        buzzer_pwm.duty_u16(0)

def beep(ms=100, freq=2500):
    buzzer_on(freq=freq)
    time.sleep_ms(ms)
    buzzer_off()

# ==================== SMOKE DETECTION ====================
smoke_high = False
smoke_high_start = 0
smoke_triggered = False
smoke_manually_cancelled = False  # User manually overrode smoke opening
last_smoke_beep = time.ticks_ms()

# ==================== REMOTE CONTROL STATE ====================
remote_x = CENTER_X
remote_y = CENTER_Y
remote_button = 1
last_remote_recv_ms = time.ticks_ms()
remote_comm_ok = False

x_mode_triggered = False
x_lock_triggered = False
close_and_lock_requested = False  # Flag for auto-close-and-lock sequence
last_manual_y_time_ms = 0

# ==================== OUTDOOR SENSOR STATE ====================
outdoor_temp_f = None
outdoor_loud = False
last_outdoor_recv_ms = time.ticks_ms()
outdoor_comm_ok = False

# ==================== PERIODIC STATE SEND ====================
last_state_send_ms = time.ticks_ms()

# ==================== AUTO CONTROL LOGIC ====================
last_auto_direction = None  # Track last AUTO direction for hysteresis

def compute_auto_desired():
    """
    Returns 'open', 'close', or None.
    Priority: outdoor_loud > temperature control
    Includes hysteresis to prevent oscillation.
    
    Logic:
    - If indoor too cold (< 67°F) and outdoor warmer: OPEN to bring in warm air
    - If indoor too hot (> 73°F) and outdoor cooler: OPEN to bring in cool air
    - If indoor too hot (> 73°F) and outdoor hotter: CLOSE to keep hot air out
    - If indoor too cold (< 67°F) and outdoor colder: CLOSE to keep cold air out
    - If indoor comfortable (67-73°F) but outdoor significantly different (>5°F): CLOSE preemptively
    - Requires minimum 2°F difference between indoor/outdoor to take action (prevents cycling on equal temps)
    """
    global last_auto_direction
    
    # Priority 1: Noise (always close if outdoor is loud)
    if outdoor_loud:
        last_auto_direction = 'close'
        return 'close'

    # Priority 2: Temperature control
    if indoor_temp_f is None or outdoor_temp_f is None:
        return None

    Ti = indoor_temp_f
    To = outdoor_temp_f
    temp_diff = abs(To - Ti)

    # Apply hysteresis based on last direction
    if last_auto_direction == 'close':
        # If we were closing, need stronger signal to open
        low_thresh = TEMP_TARGET_LOW - TEMP_HYSTERESIS
        high_thresh = TEMP_TARGET_HIGH - TEMP_HYSTERESIS
    elif last_auto_direction == 'open':
        # If we were opening, need stronger signal to close
        low_thresh = TEMP_TARGET_LOW + TEMP_HYSTERESIS
        high_thresh = TEMP_TARGET_HIGH + TEMP_HYSTERESIS
    else:
        # No previous direction, use base thresholds
        low_thresh = TEMP_TARGET_LOW
        high_thresh = TEMP_TARGET_HIGH

    # Temperature-based decisions
    # Indoor too cold - want to warm up
    if Ti < low_thresh:
        # Check if temperature difference is significant enough to act
        if temp_diff < TEMP_ACTION_MIN_DIFF:
            # Temps too similar - maintain current state
            return last_auto_direction
        elif To > Ti:
            # Outdoor warmer - open to bring in warm air
            last_auto_direction = 'open'
            return 'open'
        else:  # To < Ti
            # Outdoor colder - close to keep cold air out
            last_auto_direction = 'close'
            return 'close'
    
    # Indoor too hot - want to cool down
    elif Ti > high_thresh:
        # Check if temperature difference is significant enough to act
        if temp_diff < TEMP_ACTION_MIN_DIFF:
            # Temps too similar - maintain current state
            return last_auto_direction
        elif To < Ti:
            # Outdoor cooler - open to bring in cool air
            last_auto_direction = 'open'
            return 'open'
        else:  # To > Ti
            # Outdoor hotter - close to keep hot air out
            last_auto_direction = 'close'
            return 'close'
    
    # Indoor in comfort zone (67-73°F) - check for proactive closing
    else:
        # Proactive protection: close if outdoor is significantly hotter or colder
        if To > TEMP_TARGET_HIGH + TEMP_PROACTIVE_DIFF:
            # Outdoor is too hot (> 78°F) - close to prevent heating up
            last_auto_direction = 'close'
            return 'close'
        elif To < TEMP_TARGET_LOW - TEMP_PROACTIVE_DIFF:
            # Outdoor is too cold (< 62°F) - close to prevent cooling down
            last_auto_direction = 'close'
            return 'close'
        else:
            # Outdoor is reasonable - maintain current state
            return last_auto_direction

# ==================== STATUS DISPLAY ====================
last_status_print = time.ticks_ms()

print("BASE: Ready. AUTO mode =", auto_mode)
print("BASE: PDLC Film:", "CLEAR" if relay_state else "PRIVACY")
send_state()

# ==================== MAIN LOOP ====================
while True:
    wdt.feed()  # Feed watchdog
    now_ms = time.ticks_ms()
    now_us = time.ticks_us()

    # ==================== LIMIT SWITCHES & POSITION ====================
    if left_limit_pressed():
        if position_steps != WINDOW_MIN_STEPS:
            print("BASE: LEFT limit hit -> OPEN position")
            # Clear override when window fully opens
            clear_lock_override()
        position_steps = WINDOW_MIN_STEPS

    if right_limit_pressed():
        if position_steps != WINDOW_MAX_STEPS:
            print("BASE: RIGHT limit hit -> CLOSED position")
        position_steps = WINDOW_MAX_STEPS
        if not window_closed:
            window_closed = True
            last_closed_time_ms = now_ms
    else:
        # Window moved away from closed position
        if window_closed:
            window_closed = False
            # Clear manual override when window starts moving from closed
            clear_lock_override()

    # ==================== ESP-NOW RECEIVE ====================
    # Process ESP-NOW with minimal blocking
    host, msg = e.recv(0)
    if msg:
        if host == REMOTE_MAC and len(msg) >= 7:
            try:
                remote_x, remote_y, remote_button, temp_center_int = struct.unpack('>HHBh', msg[0:7])
                last_remote_recv_ms = now_ms
                remote_comm_ok = True
                
                # Update temperature setpoints from remote
                temp_center = temp_center_int / 10.0
                TEMP_TARGET_LOW = temp_center - TEMP_RANGE_HALF
                TEMP_TARGET_HIGH = temp_center + TEMP_RANGE_HALF
                
                # Button toggles relay with debounce holdoff
                # Detect falling edge (1 -> 0) AND enforce minimum time between toggles
                if last_remote_button == 1 and remote_button == 0:
                    if time.ticks_diff(now_ms, last_button_toggle_ms) >= BUTTON_HOLDOFF_MS:
                        set_relay_state(1 - relay_state, reason="manual button press")
                        last_button_toggle_ms = now_ms
                        print("BASE: Button toggle accepted")
                    else:
                        print("BASE: Button press ignored (debounce holdoff)")
                last_remote_button = remote_button
                
            except Exception as ex:
                if not motor_active:
                    print("BASE: Bad joystick packet:", ex)

        elif host == OUTDOOR_MAC and len(msg) == 3:
            temp_f10, loud = struct.unpack('>hB', msg)
            outdoor_loud = bool(loud)
            if temp_f10 != -32768:
                outdoor_temp_f = temp_f10 / 10.0
            else:
                outdoor_temp_f = None
            last_outdoor_recv_ms = now_ms
            outdoor_comm_ok = True

    # ==================== PERIODIC STATE SEND (NEW) ====================
    # Send state periodically so REMOTE gets temperature updates
    if time.ticks_diff(now_ms, last_state_send_ms) >= STATE_SEND_INTERVAL_MS:
        send_state()

    # ==================== COMMUNICATION TIMEOUT CHECK ====================
    # Move timeout checks AFTER all processing to avoid blocking
    # Only check when motor is not active to avoid timing interference
    if not motor_active:
        if time.ticks_diff(now_ms, last_remote_recv_ms) > COMM_TIMEOUT_MS:
            if remote_comm_ok:
                print("BASE: WARNING - Remote communication timeout!")
                remote_comm_ok = False
        
        if time.ticks_diff(now_ms, last_outdoor_recv_ms) > COMM_TIMEOUT_MS:
            if outdoor_comm_ok:
                print("BASE: WARNING - Outdoor sensor communication timeout!")
                outdoor_temp_f = None  # Invalidate stale data
                outdoor_comm_ok = False

    # ==================== SMOKE DETECTION ====================
    if pm25 is not None:
        if pm25 >= SMOKE_PM25_THRESHOLD:
            if not smoke_high:
                smoke_high = True
                smoke_high_start = now_ms
                # Auto-clear film when smoke is detected
                if relay_state == 0:  # If in privacy mode
                    set_relay_state(1, reason="smoke detected - clearing for visibility")
            else:
                # Only trigger if not manually cancelled
                if (not smoke_triggered and not smoke_manually_cancelled and
                    time.ticks_diff(now_ms, smoke_high_start) > SMOKE_HOLD_MS):
                    smoke_triggered = True
                    print("BASE: SMOKE TRIGGER - Opening window!")
        elif pm25 <= SMOKE_PM25_CLEAR:
            # Clear smoke trigger when PM2.5 drops below clear threshold
            if smoke_high or smoke_triggered or smoke_manually_cancelled:
                print("BASE: Smoke cleared (PM2.5 =", pm25, "ug/m3) - Resetting smoke system")
            smoke_high = False
            smoke_triggered = False
            smoke_manually_cancelled = False  # Reset manual cancel flag

    # ==================== AUTO-LOCK WHEN CLOSED & IDLE ====================
    # Only auto-lock if no manual override is active
    if right_limit_pressed() and not motor_active and not servo_locked and not manual_lock_override:
        if time.ticks_diff(now_ms, last_closed_time_ms) > AUTO_LOCK_DELAY_MS:
            lock_servo(manual=False)

    # ==================== AUTO CONTROL LOGIC ====================
    # Check if we're closing due to outdoor noise
    current_noise_close = False
    if auto_mode and outdoor_loud:
        desired = compute_auto_desired()
        if desired == 'close':
            current_noise_close = True
            # If we just started closing due to noise and film is clear, auto-dim it
            if not last_noise_close_state and relay_state == 1:
                set_relay_state(0, reason="closing due to outdoor noise")
    
    # Track state for next iteration
    last_noise_close_state = current_noise_close
    
    # If noise stops, don't auto-clear (let user control manually)
    # Film only auto-changes for: 1) noise starts while clear, 2) smoke while privacy

    # ==================== X-AXIS GESTURES ====================
    diff_x = remote_x - CENTER_X

    # +X flick -> toggle auto/manual
    if diff_x > DEADZONE:
        if not x_mode_triggered:
            x_mode_triggered = True
            set_auto_mode(not auto_mode)
            print("BASE: Joystick +X detected, toggling AUTO mode")
    else:
        x_mode_triggered = False

    # -X flick -> toggle servo lock/unlock (MANUAL CONTROL)
    if diff_x < -DEADZONE:
        if not x_lock_triggered:
            x_lock_triggered = True
            print("BASE: Joystick -X detected - Manual lock toggle")
            if right_limit_pressed():
                # Manual toggle when at closed position
                if servo_locked:
                    unlock_servo(manual=True)
                else:
                    lock_servo(manual=True)
            else:
                # Window not closed - initiate auto-close and lock sequence
                print("BASE: Lock requested but window not closed - initiating close sequence")
                set_auto_mode(False)  # Switch to manual mode
                close_and_lock_requested = True
    else:
        x_lock_triggered = False
        # Don't reset close_and_lock_requested here - let it complete

    # Safety: if locked but not closed, unlock immediately
    # BUT respect grace period after locking (in case switch bounces or servo pushes window slightly)
    if not right_limit_pressed() and servo_locked:
        if time.ticks_diff(now_ms, last_servo_lock_ms) > SERVO_LOCK_GRACE_MS:
            print("BASE: SAFETY - Window not closed but servo locked -> emergency unlock")
            servo_locked = False
            set_servo_angle(UNLOCK_ANGLE)
            send_state()
        # else: within grace period, ignore - switch might be bouncing

    # ==================== Y-AXIS CONTROL ====================
    diff_y = remote_y - CENTER_Y
    manual_y_active = abs(diff_y) >= DEADZONE

    # CHANGED: Check for manual Y input FIRST - it should interrupt close-and-lock
    if manual_y_active:
        # Manual Y movement -> switch to MANUAL mode & cancel smoke override
        set_auto_mode(False)
        if smoke_triggered or smoke_high:
            print("BASE: Manual Y input -> smoke opening cancelled until smoke clears")
            smoke_triggered = False
            smoke_high = False
            smoke_manually_cancelled = True  # Block smoke opening until smoke clears

        # Manual Y input cancels close-and-lock sequence
        if close_and_lock_requested:
            print("BASE: Manual Y input -> cancelling close-and-lock sequence")
            close_and_lock_requested = False

        last_manual_y_time_ms = now_ms

        # -Y (joystick down/up on hardware) -> toward CLOSED, +Y -> toward OPEN
        # Y=0 is up, Y=4095 is down on this joystick
        desired_dir = DIR_TOWARD_OPEN if diff_y < 0 else DIR_TOWARD_CLOSED

        # CRITICAL: Unlock before ANY opening movement
        if desired_dir == DIR_TOWARD_OPEN:
            if servo_locked:
                print("BASE: Manual OPEN requested - Unlocking servo FIRST")
                unlock_servo(manual=False)  # Auto unlock, don't set override
                # Wait for servo to complete unlock before allowing movement
                time.sleep_ms(400)
            
            # Double-check servo is unlocked before proceeding
            if servo_locked:
                print("BASE: ERROR - Servo still locked, blocking movement")
                motor_active = False
                step_interval_us = None
                EN.value(1)
            elif left_limit_pressed():
                # Already at open limit
                motor_active = False
                step_interval_us = None
            else:
                # Safe to open
                DIR.value(DIR_TOWARD_OPEN)
                last_dir = DIR_TOWARD_OPEN
                mag = min(abs(diff_y) - DEADZONE, 2000)
                if mag < 0:
                    mag = 0
                norm = mag / 2000.0
                delay = int(MAX_DELAY_US - norm * (MAX_DELAY_US - MIN_DELAY_US))
                step_interval_us = max(MIN_DELAY_US, min(MAX_DELAY_US, delay))
                motor_active = True
                last_motor_active_ms = now_ms
                
        else:  # DIR_TOWARD_CLOSED
            if right_limit_pressed():
                # Already at closed limit
                motor_active = False
                step_interval_us = None
            else:
                # Safe to close
                DIR.value(DIR_TOWARD_CLOSED)
                last_dir = DIR_TOWARD_CLOSED
                mag = min(abs(diff_y) - DEADZONE, 2000)
                if mag < 0:
                    mag = 0
                norm = mag / 2000.0
                delay = int(MAX_DELAY_US - norm * (MAX_DELAY_US - MIN_DELAY_US))
                step_interval_us = max(MIN_DELAY_US, min(MAX_DELAY_US, delay))
                motor_active = True
                last_motor_active_ms = now_ms

    # CHANGED: Close-and-lock sequence now only runs if NO manual Y input
    elif close_and_lock_requested and not right_limit_pressed():
        # Override manual Y input to close window
        print("BASE: Executing close-and-lock sequence")
        set_auto_mode(False)  # Ensure manual mode
        
        # Command window to close
        DIR.value(DIR_TOWARD_CLOSED)
        last_dir = DIR_TOWARD_CLOSED
        step_interval_us = MIN_DELAY_US  # Use fast speed
        motor_active = True
        last_motor_active_ms = now_ms
        
    elif close_and_lock_requested and right_limit_pressed():
        # Window reached closed position - lock it
        print("BASE: Close-and-lock sequence complete - locking")
        motor_active = False
        step_interval_us = None
        lock_servo(manual=True)
        close_and_lock_requested = False

    else:
        # No manual Y, no close-and-lock -> check for smoke override or auto mode
        if smoke_triggered:
            # Priority 1: Smoke override (force open)
            if not left_limit_pressed():
                # CRITICAL: Unlock before smoke movement
                if servo_locked:
                    print("BASE: SMOKE - Unlocking servo before emergency open")
                    unlock_servo(manual=False)
                    time.sleep_ms(400)
                
                if not servo_locked:  # Verify unlock succeeded
                    DIR.value(DIR_TOWARD_OPEN)
                    last_dir = DIR_TOWARD_OPEN
                    step_interval_us = MIN_DELAY_US  # Use faster speed for emergencies
                    motor_active = True
                    last_motor_active_ms = now_ms
                else:
                    motor_active = False
                    step_interval_us = None
            else:
                motor_active = False
                step_interval_us = None
        elif auto_mode:
            # Priority 2: AUTO mode
            desired = compute_auto_desired()
            if desired == 'close':
                if not right_limit_pressed():
                    DIR.value(DIR_TOWARD_CLOSED)
                    last_dir = DIR_TOWARD_CLOSED
                    step_interval_us = MAX_DELAY_US
                    motor_active = True
                    last_motor_active_ms = now_ms
                else:
                    motor_active = False
                    step_interval_us = None
            elif desired == 'open':
                if not left_limit_pressed():
                    # CRITICAL: Unlock before auto movement
                    if servo_locked:
                        print("BASE: AUTO - Unlocking servo before auto open")
                        unlock_servo(manual=False)
                        time.sleep_ms(400)
                    
                    if not servo_locked:  # Verify unlock succeeded
                        DIR.value(DIR_TOWARD_OPEN)
                        last_dir = DIR_TOWARD_OPEN
                        step_interval_us = MAX_DELAY_US
                        motor_active = True
                        last_motor_active_ms = now_ms
                    else:
                        motor_active = False
                        step_interval_us = None
                else:
                    motor_active = False
                    step_interval_us = None
            else:
                motor_active = False
                step_interval_us = None
        else:
            motor_active = False
            step_interval_us = None

    # ==================== MOTOR ENABLE/DISABLE ====================
    if motor_active and step_interval_us is not None:
        # Enable driver - add wake-up delay if it was disabled
        if driver_was_disabled:
            EN.value(0)
            time.sleep_us(50)  # Give driver time to wake up
            driver_was_disabled = False
            print("BASE: Driver enabled after idle")
        else:
            EN.value(0)
    else:
        if time.ticks_diff(now_ms, last_motor_active_ms) > IDLE_EN_DELAY_MS:
            if EN.value() == 0:  # Was enabled
                EN.value(1)
                driver_was_disabled = True

    # ==================== STEP PULSES & POSITION UPDATE ====================
    if motor_active and step_interval_us is not None:
        if time.ticks_diff(now_us, last_step_time) >= step_interval_us:
            # Check limits before stepping
            if last_dir == DIR_TOWARD_CLOSED and right_limit_pressed():
                motor_active = False
                step_interval_us = None
                EN.value(1)
                beep(ms=80, freq=1800)
                print("BASE: Hit CLOSED limit while moving")
            elif last_dir == DIR_TOWARD_OPEN and left_limit_pressed():
                motor_active = False
                step_interval_us = None
                EN.value(1)
                beep(ms=80, freq=1800)
                print("BASE: Hit OPEN limit while moving")
            else:
                STEP.value(1)
                time.sleep_us(5)
                STEP.value(0)
                last_step_time = now_us  # Use current time, not function call

                # Update position
                if last_dir == DIR_TOWARD_CLOSED:
                    position_steps += 1
                else:
                    position_steps -= 1
                clamp_position()

    # ==================== DHT READING ====================
    if time.ticks_diff(now_ms, last_dht_time) > DHT_INTERVAL_MS:
        last_dht_time = now_ms
        read_dht_with_retry()

    # ==================== PMS READING ====================
    if time.ticks_diff(now_ms, last_pms_time) > PMS_INTERVAL_MS:
        last_pms_time = now_ms
        update_pms()

    # ==================== SMOKE ALERT BEEP ====================
    if pm25 is not None and pm25 > SMOKE_PM25_THRESHOLD:
        if time.ticks_diff(now_ms, last_smoke_beep) > SMOKE_BEEP_INTERVAL_MS:
            print("BASE: SMOKE ALERT - PM2.5 =", pm25, "ug/m3")
            beep(ms=150, freq=3500)
            last_smoke_beep = now_ms

    time.sleep_ms(5)

    # ==================== STATUS PRINT ====================
    if time.ticks_diff(now_ms, last_status_print) > 1000:
        last_status_print = now_ms
        tf_str = "{:.1f}F".format(indoor_temp_f) if indoor_temp_f is not None else "N/A"
        pm_str = "{} ug/m3".format(pm25) if pm25 is not None else "N/A"
        out_str = "{:.1f}F".format(outdoor_temp_f) if outdoor_temp_f is not None else "N/A"
        lock_status = "LOCKED" if servo_locked else "UNLOCKED"
        if manual_lock_override:
            lock_status += " (MANUAL)"
        
        # Determine current action and reason
        action_str = "IDLE"
        if close_and_lock_requested:
            if right_limit_pressed():
                action_str = "CLOSE-AND-LOCK sequence - Locking"
            else:
                action_str = "CLOSE-AND-LOCK sequence - Closing"
        elif manual_y_active:
            action_str = "MANUAL CONTROL"
        elif smoke_triggered:
            action_str = "SMOKE OVERRIDE - OPENING"
        elif smoke_manually_cancelled and smoke_high:
            action_str = "SMOKE DETECTED - Manual override active, waiting for clear"
        elif auto_mode:
            desired = compute_auto_desired()
            if desired == 'close':
                if outdoor_loud:
                    action_str = "AUTO CLOSING (outdoor noise)"
                elif indoor_temp_f is not None and outdoor_temp_f is not None:
                    temp_diff = abs(outdoor_temp_f - indoor_temp_f)
                    if temp_diff < TEMP_ACTION_MIN_DIFF:
                        action_str = "AUTO CLOSING (maintaining - temps too similar)"
                    elif indoor_temp_f >= TEMP_TARGET_HIGH and outdoor_temp_f > indoor_temp_f:
                        action_str = "AUTO CLOSING (indoor too hot, outdoor hotter)"
                    elif indoor_temp_f <= TEMP_TARGET_LOW and outdoor_temp_f < indoor_temp_f:
                        action_str = "AUTO CLOSING (indoor too cold, outdoor colder)"
                    elif outdoor_temp_f > TEMP_TARGET_HIGH + TEMP_PROACTIVE_DIFF:
                        action_str = "AUTO CLOSING (proactive: outdoor too hot)"
                    elif outdoor_temp_f < TEMP_TARGET_LOW - TEMP_PROACTIVE_DIFF:
                        action_str = "AUTO CLOSING (proactive: outdoor too cold)"
                    else:
                        action_str = "AUTO CLOSING (temp logic)"
                else:
                    action_str = "AUTO CLOSING (no temp data)"
            elif desired == 'open':
                if indoor_temp_f is not None and outdoor_temp_f is not None:
                    temp_diff = abs(outdoor_temp_f - indoor_temp_f)
                    if temp_diff < TEMP_ACTION_MIN_DIFF:
                        action_str = "AUTO OPENING (maintaining - temps too similar)"
                    elif indoor_temp_f < TEMP_TARGET_LOW and outdoor_temp_f > indoor_temp_f:
                        action_str = "AUTO OPENING (indoor cold, outdoor warmer)"
                    elif indoor_temp_f > TEMP_TARGET_HIGH and outdoor_temp_f < indoor_temp_f:
                        action_str = "AUTO OPENING (indoor hot, outdoor cooler)"
                    else:
                        action_str = "AUTO OPENING (temp logic)"
                else:
                    action_str = "AUTO OPENING (no temp data)"
            else:
                action_str = "AUTO IDLE (temps in range)"
        
        print("=== ACTION:", action_str, "===")
        print("BASE: Joy X:", remote_x, "Y:", remote_y,
              "| Indoor:", tf_str,
              "| Outdoor:", out_str,
              "| PM2.5:", pm_str,
              "| Pos:", position_steps,
              "| Closed_sw:", right_limit_pressed(),
              "| Open_sw:", left_limit_pressed(),
              "| Lock:", lock_status,
              "| PDLC:", "CLEAR" if relay_state else "PRIVACY",
              "| Loud_out:", outdoor_loud,
              "| AUTO:", auto_mode,
              "| Smoke:", smoke_triggered,
              "| Remote_OK:", remote_comm_ok,
              "| Outdoor_OK:", outdoor_comm_ok)

    # Only sleep when motor is not active to maintain precise timing
    if not motor_active:
        time.sleep_ms(5)
