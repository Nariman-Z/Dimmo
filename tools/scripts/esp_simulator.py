import os
import time
import json
import random
import ssl
import paho.mqtt.client as mqtt
from datetime import datetime
from threading import Timer

# =============================================================================
# LOAD CONFIGURATION FROM JSON
# =============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "config.json")
EXAMPLE_CONFIG_FILE = os.path.join(SCRIPT_DIR, "config.example.json")

def load_config():
    target_file = CONFIG_FILE if os.path.exists(CONFIG_FILE) else EXAMPLE_CONFIG_FILE
    if not os.path.exists(target_file):
        raise FileNotFoundError(f"Configuration file not found at {CONFIG_FILE} or {EXAMPLE_CONFIG_FILE}")
    with open(target_file, "r", encoding="utf-8") as f:
        return json.load(f)

config = load_config()

BROKER = config.get("broker", "YOUR_BROKER_HOST")
PORT = int(config.get("port", 8883))
USER = config.get("username", "dev")
PASS = config.get("password", "dev")
DEVICE_ID = config.get("device_id", "dev")

# MQTT Topics
TOPIC_COMMAND = f"devices/{DEVICE_ID}/command"
TOPIC_STATUS = f"devices/{DEVICE_ID}/status"
TOPIC_TELEMETRY = f"devices/{DEVICE_ID}/telemetry"


# =============================================================================
# TERMINAL COLORS
# =============================================================================

class Colors:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"
    GREY = "\033[90m"

# =============================================================================
# SIMULATOR
# =============================================================================

class DimmoESPSimulator:
    def __init__(self):
        self.brightness = 0
        self.is_on = False
        self.status_timer = None
        self.last_published_brightness = -1

        sim_id = f"{DEVICE_ID}_hw_sim"
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=sim_id)
        except Exception:
            self.client = mqtt.Client(client_id=sim_id)

    def _log(self, action, topic, payload, decrypted_logic=None, color=Colors.WHITE):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # 1. Show RAW in White (Always)
        print(f"{Colors.GREY}[{timestamp}]{Colors.RESET} {Colors.BOLD}{action:<10}{Colors.RESET} | {topic}")
        print(f"    {Colors.WHITE}RAW MQTT: {payload}{Colors.RESET}")

        # 2. Show Processed Logic in Color
        if decrypted_logic:
            print(f"    {color}PROCESSED: {decrypted_logic}{Colors.RESET}")
        print(f"{Colors.GREY}{'─'*90}{Colors.RESET}")

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            print(f"\n{Colors.MAGENTA}{'═'*90}{Colors.RESET}")
            print(f"{Colors.BOLD}               🚀 Dimmo Secure ESP32 Simulator{Colors.RESET}")
            print(f"{Colors.GREY} Device : {DEVICE_ID} | Status : {Colors.GREEN}Connected{Colors.RESET}")
            print(f"{Colors.MAGENTA}{'═'*90}{Colors.RESET}\n")
            client.subscribe(TOPIC_COMMAND)
            self.publish_status("online")
        else:
            print(f"{Colors.RED}Connection failed (rc={rc}){Colors.RESET}")

    def publish_status(self, state="online"):
        """Authoritative state (Retained)"""
        if self.status_timer:
            self.status_timer.cancel()
            self.status_timer = None

        # Ensure consistency: if state is offline or power is false, brightness is 0
        effective_brightness = self.brightness if (state == "online" and self.is_on) else 0
        effective_power = self.is_on if state == "online" else False

        data = {
            "state": state,
            "power": effective_power,
            "brightness": effective_brightness
        }
        payload = json.dumps(data)

        # We always publish status to ensure new clients get the full picture immediately
        self.client.publish(TOPIC_STATUS, payload, qos=1, retain=True)

        self.last_published_brightness = effective_brightness
        self._log("PUB STATUS", TOPIC_STATUS, payload, f"Sync State -> {state.upper()} | Power: {effective_power} | Brightness: {effective_brightness}%", Colors.BLUE)

    def send_telemetry(self):
        current_ma = round((350.0 * (self.brightness / 100.0)) if self.is_on else 0.0, 1)
        wattage = round((80.5 * (self.brightness / 100.0)) if self.is_on else 0.0, 1)
        data = {
            "voltage": round(random.uniform(228.0, 232.0), 1),
            "current": current_ma,
            "power": wattage,
            "frequency": 50.0,
            "powerConsumption": 12.45,
            "powerFactor": round(random.uniform(0.95, 0.99), 2),
            "rssi": random.randint(-65, -55),
            "battery": random.randint(15, 100)  # Random battery for testing
        }
        payload = json.dumps(data)
        self.client.publish(TOPIC_TELEMETRY, payload)
        self._log("PUB TELE", TOPIC_TELEMETRY, payload, f"Sensors: {data['power']}W | {data['current']}mA", Colors.GREEN)

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        try:
            cmd = json.loads(payload)
            self._log("RECV CMD", msg.topic, payload, f"Apply: {cmd}", Colors.YELLOW)

            if "power" in cmd: self.is_on = bool(cmd["power"])
            if "brightness" in cmd: self.brightness = int(cmd["brightness"])

            # AGGRESSIVE DEBOUNCE: Only confirm status after the command stream stops for 300ms
            if self.status_timer:
                self.status_timer.cancel()

            self.status_timer = Timer(0.3, self.publish_status)
            self.status_timer.start()

        except Exception as e:
            print(f"{Colors.RED}JSON Error: {e}{Colors.RESET}")

    def run(self):
        self.client.username_pw_set(USER, PASS)
        self.client.tls_set(cert_reqs=ssl.CERT_NONE, tls_version=ssl.PROTOCOL_TLSv1_2)
        self.client.will_set(TOPIC_STATUS, json.dumps({"state": "offline", "power": False, "brightness": 0}), retain=True)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(BROKER, PORT)

        last_telemetry = time.time()
        try:
            while True:
                self.client.loop(0.05)
                if time.time() - last_telemetry >= 5:
                    self.send_telemetry()
                    last_telemetry = time.time()
        except KeyboardInterrupt:
            self.publish_status("offline")
            self.client.disconnect()
            print(f"\n{Colors.YELLOW}Simulator stopped gracefully.{Colors.RESET}")

if __name__ == "__main__":
    DimmoESPSimulator().run()
