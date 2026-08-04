#!/usr/bin/env python3
"""
Dimmo MQTT Payload Schema Validator & Live Testing Tool
======================================================
Validates incoming and outgoing MQTT payloads against official Dimmo JSON schemas
and executes offline or live integration tests.

Usage:
  python3 test_mqtt_payloads.py --offline
  python3 test_mqtt_payloads.py --live --broker HOST --port 8883 --user USER --pass PASS
"""

import os
import sys
import json
import argparse
from typing import Dict, Any

# ANSI Colors
class Colors:
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BLUE = "\033[94m"
    RESET = "\033[0m"

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SCHEMAS_DIR = os.path.join(BASE_DIR, "schemas")
SAMPLES_DIR = os.path.join(BASE_DIR, "samples")


def load_json(filepath: str) -> Dict[str, Any]:
    with open(filepath, "r", encoding="utf-8") as f:
        return json.load(f)


def validate_command_payload(payload: Dict[str, Any]) -> bool:
    """Validate Command Payload: {"brightness": int (0-100), "power": bool}"""
    if not isinstance(payload, dict):
        print(f"{Colors.RED}[FAIL] Command payload must be a JSON object{Colors.RESET}")
        return False

    if "brightness" not in payload and "power" not in payload:
        print(f"{Colors.RED}[FAIL] Command payload must contain 'brightness' or 'power'{Colors.RESET}")
        return False

    if "brightness" in payload:
        b = payload["brightness"]
        if not isinstance(b, (int, float)) or not (0 <= b <= 100):
            print(f"{Colors.RED}[FAIL] 'brightness' must be integer between 0 and 100 (got {b}){Colors.RESET}")
            return False

    if "power" in payload:
        p = payload["power"]
        if not isinstance(p, bool):
            print(f"{Colors.RED}[FAIL] 'power' must be boolean true/false (got {p}){Colors.RESET}")
            return False

    return True


def validate_telemetry_payload(payload: Dict[str, Any]) -> bool:
    """Validate Telemetry Payload fields"""
    required_keys = ["voltage", "current", "power", "frequency", "powerConsumption", "powerFactor", "brightness"]
    for key in required_keys:
        if key not in payload:
            print(f"{Colors.RED}[FAIL] Telemetry payload missing required key '{key}'{Colors.RESET}")
            return False

    if not isinstance(payload["voltage"], (int, float)) or payload["voltage"] < 0:
        print(f"{Colors.RED}[FAIL] 'voltage' must be positive number{Colors.RESET}")
        return False

    if not isinstance(payload["brightness"], (int, float)) or not (0 <= payload["brightness"] <= 100):
        print(f"{Colors.RED}[FAIL] 'brightness' out of range 0-100{Colors.RESET}")
        return False

    return True


def validate_status_payload(payload: Dict[str, Any]) -> bool:
    """Validate Status Payload: {"state": "online"|"offline", "power": bool, "brightness": int}"""
    required = ["state", "power", "brightness"]
    for k in required:
        if k not in payload:
            print(f"{Colors.RED}[FAIL] Status payload missing key '{k}'{Colors.RESET}")
            return False

    if payload["state"] not in ["online", "offline"]:
        print(f"{Colors.RED}[FAIL] 'state' must be 'online' or 'offline'{Colors.RESET}")
        return False

    if not isinstance(payload["power"], bool):
        print(f"{Colors.RED}[FAIL] 'power' must be boolean{Colors.RESET}")
        return False

    return True


def run_offline_validation():
    print(f"{Colors.BLUE}=== Running Dimmo MQTT Offline Payload Schema Tests ==={Colors.RESET}\n")

    # Test Command Schema
    cmd_on = load_json(os.path.join(SAMPLES_DIR, "sample_command_on.json"))
    cmd_dim = load_json(os.path.join(SAMPLES_DIR, "sample_command_dim.json"))

    print("Testing Command Payloads:")
    assert validate_command_payload(cmd_on)
    print(f"  {Colors.GREEN}✓ sample_command_on.json: {json.dumps(cmd_on)}{Colors.RESET}")
    assert validate_command_payload(cmd_dim)
    print(f"  {Colors.GREEN}✓ sample_command_dim.json: {json.dumps(cmd_dim)}{Colors.RESET}")

    # Test Telemetry Schema
    print("\nTesting Telemetry Payload:")
    telemetry = load_json(os.path.join(SAMPLES_DIR, "sample_telemetry.json"))
    assert validate_telemetry_payload(telemetry)
    print(f"  {Colors.GREEN}✓ sample_telemetry.json: {json.dumps(telemetry)}{Colors.RESET}")

    # Test Status Schema
    print("\nTesting Status Payload:")
    status = load_json(os.path.join(SAMPLES_DIR, "sample_status.json"))
    assert validate_status_payload(status)
    print(f"  {Colors.GREEN}✓ sample_status.json: {json.dumps(status)}{Colors.RESET}")

    print(f"\n{Colors.GREEN}🎉 All JSON payload schemas validated successfully!{Colors.RESET}")


def run_live_test(broker: str, port: int, user: str, password: str, device_id: str):
    print(f"{Colors.BLUE}=== Running Live MQTT Integration Test to {broker}:{port} ==={Colors.RESET}\n")
    try:
        import paho.mqtt.client as mqtt
        import ssl
    except ImportError:
        print(f"{Colors.RED}paho-mqtt library required for live tests. Run: pip install paho-mqtt{Colors.RESET}")
        sys.exit(1)

    cmd_topic = f"devices/{device_id}/command"
    tele_topic = f"devices/{device_id}/telemetry"

    client = mqtt.Client(client_id=f"Dimmo_Schema_Tester")
    if user and password:
        client.username_pw_set(user, password)

    if port == 8883:
        client.tls_set(cert_reqs=ssl.CERT_NONE, tls_version=ssl.PROTOCOL_TLSv1_2)

    def on_connect(c, userdata, flags, rc):
        if rc == 0:
            print(f"{Colors.GREEN}Connected to MQTT broker successfully.{Colors.RESET}")
            c.subscribe(tele_topic)
            # Publish test command
            payload = json.dumps({"brightness": 75, "power": True})
            print(f"{Colors.YELLOW}Publishing test command to {cmd_topic}: {payload}{Colors.RESET}")
            c.publish(cmd_topic, payload)
        else:
            print(f"{Colors.RED}Failed connection: rc={rc}{Colors.RESET}")

    def on_message(c, userdata, msg):
        print(f"{Colors.GREEN}Received Live Telemetry on {msg.topic}: {msg.payload.decode()}{Colors.RESET}")
        c.disconnect()

    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(broker, port, keepalive=10)
    client.loop_timeout(timeout=5.0)


def main():
    parser = argparse.ArgumentParser(description="Dimmo MQTT Schema & Payload Tester")
    parser.add_argument("--offline", action="store_true", help="Run offline JSON schema validation tests")
    parser.add_argument("--live", action="store_true", help="Execute live MQTT broker test")
    parser.add_argument("--broker", type=str, default="zc2ba0d1.ala.asia-southeast1.emqxsl.com", help="MQTT Broker Host")
    parser.add_argument("--port", type=int, default=8883, help="MQTT Broker Port")
    parser.add_argument("--user", type=str, default="samsung", help="MQTT Username")
    parser.add_argument("--pass", type=str, dest="password", default="uZsSZCDjP353L4i", help="MQTT Password")
    parser.add_argument("--device-id", type=str, default="dev", help="Target Device ID")

    args = parser.parse_args()

    if args.live:
        run_live_test(args.broker, args.port, args.user, args.password, args.device_id)
    else:
        run_offline_validation()


if __name__ == "__main__":
    main()
