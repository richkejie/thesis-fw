#!/usr/bin/env python3
"""
esp32_cmd.py — Send commands to the ESP32 WiFi command server.

Usage:
    python esp32_cmd.py <ESP32_IP> [port]

Then type one of:
    print    → ESP32 logs "print cmd received" on its monitor
    led_on   → turns LED on
    led_off  → turns LED off
    msg      → ESP32 replies "msg response sent"
    quit     → exit this script
"""

import socket
import sys

DEFAULT_PORT = 3333


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <ESP32_IP> [port]")
        sys.exit(1)

    host = sys.argv[1]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT

    print(f"Connecting to ESP32 at {host}:{port} …")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((host, port))
    except Exception as e:
        print(f"[ERROR] Could not connect: {e}")
        sys.exit(1)

    print("Connected! Commands: print | led_on | led_off | msg | quit\n")

    valid = {"print", "led_on", "led_off", "msg"}

    while True:
        try:
            cmd = input(">> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if cmd.lower() in ("quit", "exit", "q"):
            print("Disconnecting.")
            break

        if cmd not in valid:
            print(f"Unknown command '{cmd}'. Choose from: {', '.join(sorted(valid))}")
            continue

        # Send command with newline terminator
        try:
            sock.sendall((cmd + "\n").encode())
            response = sock.recv(256).decode().strip()
            print(f"ESP32: {response}")
        except socket.timeout:
            print("[WARN] No response within timeout.")
        except Exception as e:
            print(f"[ERROR] {e}")
            break

    sock.close()


if __name__ == "__main__":
    main()