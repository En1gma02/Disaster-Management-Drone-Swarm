import time
import socket
import subprocess
import shutil
import sys
import threading
from typing import List

def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")

def find_free_udp_port(preferred_start: int = 14560, max_tries: int = 50) -> int:
    """Finds a free UDP port for internal routing."""
    for port in range(preferred_start, preferred_start + max_tries):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("127.0.0.1", port))
            s.close()
            return port
        except Exception:
            s.close()
            continue
    raise RuntimeError("No free UDP port found.")

def start_mav_hub(input_ports: List[str], output_udp: str | None = None, baud: int = 57600):
    """
    Start a local MAVLink hub process using MAVProxy.
    This routes multiple incoming telemetry streams (e.g., from Scout and Mule)
    to a unified endpoint, satisfying the single-GCS requirement.
    """
    mavproxy = shutil.which("mavproxy.py")

    if not mavproxy:
        raise RuntimeError("MAVProxy not found in PATH. Install with: pip install MAVProxy")

    if output_udp is None:
        free_port = find_free_udp_port()
        output_udp = f"127.0.0.1:{free_port}"

    args = [sys.executable, mavproxy]
    for p in input_ports:
        if p.startswith("udp:"):
            args += ["--master", p]
        else:
            args += [f"--master={p},baud={baud}"]
            
    # Output to multiple local UDP endpoints for our different scripts
    args += [f"--out=udp:127.0.0.1:14560"] # Scout UDP
    args += [f"--out=udp:127.0.0.1:14561"] # Mule UDP
    args += ["--daemon"] # Run without interactive console

    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(2.0)
    return proc, [f"udp:127.0.0.1:14560", f"udp:127.0.0.1:14561"]

def main():
    print("=" * 60)
    print("UNIFIED GROUND CONTROL STATION (GCS) HUB LAUNCHER")
    print("Team DJS PHOENIX - Disaster Management Mission")
    print("=" * 60)

    # Example Input Ports: 
    # Can be SITL (udp:127.0.0.1:14550) or live telemetry radios (serial:///dev/ttyUSB0)
    # Using SITL defaults for demonstration
    input_ports = [
        'udp:127.0.0.1:14550', # SITL Instance 1 (Scout)
        'udp:127.0.0.1:14551', # SITL Instance 2 (Mule)
    ]

    print("[HUB] Starting MAVLink Router (MAVProxy)...")
    hub_proc = None
    try:
        hub_proc, out_urls = start_mav_hub(input_ports)
        log(f"Hub Active.")
        log(f"Routing Telemetry to:")
        for url in out_urls:
            log(f" -> {url}")
            
        print("\n[HUB] Unified GCS established. Ready for mission scripts.")
        print("[HUB] Press Ctrl+C to terminate the hub.")

        # Keep alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[HUB] Shutting down...")
    except Exception as e:
        print(f"[HUB] Error: {e}")
    finally:
        if hub_proc is not None:
            hub_proc.terminate()
        print("[HUB] Offline.")

if __name__ == "__main__":
    main()
