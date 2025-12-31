# main.py - OUTDOOR ESP32 (Environmental Sensor)
# This is outdoor_esp32_env_DO.py v3.1
# Upload this file as "main.py" to your OUTDOOR ESP32

import network
import espnow
from machine import Pin, WDT
import time
import dht
import struct

# ==================== CONFIGURATION ====================
BASE_MAC = b'\x14+/\xaf=\xbc'
WIFI_CHANNEL = 1

# Noise detection settings
FILTER_COUNT = 3  # Consecutive readings to change state
NOISE_INTERVAL_MS = 100
LOUD_TO_QUIET_COUNT = 5  # More readings needed to go back to quiet (reduces false negatives)

# Temperature reading settings
DHT_INTERVAL_MS = 2000
DHT_MAX_RETRIES = 3

# Communication settings
SEND_INTERVAL_MS = 1000
MAX_SEND_RETRIES = 3

# ==================== WATCHDOG ====================
wdt = WDT(timeout=8000)  # 8 second watchdog

# ==================== NETWORK SETUP ====================
w0 = network.WLAN(network.STA_IF)
w0.active(True)
w0.config(channel=WIFI_CHANNEL)

e = espnow.ESPNow()
e.active(True)
e.add_peer(BASE_MAC)

print("OUTDOOR: DO-based noise detection starting...")

# ==================== DHT11 SETUP ====================
dht_sensor = dht.DHT11(Pin(14))
outdoor_temp_f = None
last_dht_ms = time.ticks_ms()

def read_dht_with_retry():
    """Read DHT with multiple retry attempts."""
    global outdoor_temp_f
    
    for attempt in range(DHT_MAX_RETRIES):
        try:
            dht_sensor.measure()
            t = dht_sensor.temperature()
            outdoor_temp_f = t * 9.0 / 5.0 + 32.0
            return True
        except Exception as ex:
            if attempt == DHT_MAX_RETRIES - 1:
                print("OUTDOOR: DHT read failed after", DHT_MAX_RETRIES, "attempts:", ex)
                outdoor_temp_f = None
            time.sleep_ms(100)
    return False

# ==================== NOISE SENSOR SETUP ====================
# Noise DO pin: quiet=1, loud=0
noise_pin = Pin(34, Pin.IN, Pin.PULL_UP)

def raw_loud():
    """Read raw noise sensor state."""
    return noise_pin.value() == 0  # LOW = loud

loud_state = False
loud_on_cnt = 0
loud_off_cnt = 0
last_noise_ms = time.ticks_ms()

# ==================== COMMUNICATION STATE ====================
last_send_ms = time.ticks_ms()
send_fail_count = 0

print("OUTDOOR: main loop...")

while True:
    wdt.feed()  # Feed watchdog
    now = time.ticks_ms()

    # ==================== NOISE FILTERING ====================
    # Asymmetric filtering: quick to detect loud, slower to return to quiet
    if time.ticks_diff(now, last_noise_ms) > NOISE_INTERVAL_MS:
        last_noise_ms = now
        
        if raw_loud():
            loud_on_cnt += 1
            loud_off_cnt = 0
            if loud_on_cnt >= FILTER_COUNT and not loud_state:
                loud_state = True
                print("OUTDOOR: LOUD -> True")
        else:
            loud_off_cnt += 1
            loud_on_cnt = 0
            # Require more consecutive quiet readings to switch back
            if loud_off_cnt >= LOUD_TO_QUIET_COUNT and loud_state:
                loud_state = False
                print("OUTDOOR: LOUD -> False")

    # ==================== TEMPERATURE READING ====================
    if time.ticks_diff(now, last_dht_ms) > DHT_INTERVAL_MS:
        last_dht_ms = now
        read_dht_with_retry()

    # ==================== SEND DATA TO BASE ====================
    if time.ticks_diff(now, last_send_ms) > SEND_INTERVAL_MS:
        last_send_ms = now
        
        # Prepare temperature data
        if outdoor_temp_f is None:
            temp_f10 = -32768  # Sentinel value for invalid temp
        else:
            temp_f10 = int(outdoor_temp_f * 10)
        
        # Pack and send
        pkt = struct.pack('>hB', temp_f10, 1 if loud_state else 0)
        
        # Retry logic for failed sends
        success = False
        for attempt in range(MAX_SEND_RETRIES):
            try:
                e.send(BASE_MAC, pkt)
                success = True
                send_fail_count = 0
                break
            except OSError as ex:
                time.sleep_ms(10)
        
        if not success:
            send_fail_count += 1
            print("OUTDOOR: Send failed after", MAX_SEND_RETRIES, "attempts (total fails:", send_fail_count, ")")

    time.sleep_ms(10)
