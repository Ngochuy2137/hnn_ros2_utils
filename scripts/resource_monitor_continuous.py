#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Continuous resource monitor: CPU, RAM, iftop (eth0 + wlan0), ros2 topic bw /scan only.

Heavy commands (iftop, ros2 topic bw) run once in background threads and update a
shared buffer. Main loop only reads the buffer and writes the latest values to the
log file at --interval, so no repeated warm-up each cycle.

- CPU/RAM: psutil, read in main loop (instant).
- iftop: one long-running loop per interface in a thread, updates buffer every ~8s.
- ros2 topic bw + delay: if rclpy available, subscribe to /scan and compute bandwidth + delay
  (from header.stamp); else subprocess ros2 topic bw and ros2 topic delay.

Install:
  pip install psutil
  sudo apt-get install -y iftop

Run (with ROS2 sourced for scan_mbps):
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 resource_monitor_continuous.py [--interval 2] [--no-iftop] [--no-ros2-bw]
"""

import argparse
import json
import os
import pty
import re
import select
import subprocess
import sys
import threading
import time
from datetime import datetime

try:
    import psutil
except ImportError:
    print("Install: pip install psutil", file=sys.stderr)
    sys.exit(1)

# Optional: use rclpy to subscribe to /scan and compute bandwidth (no ros2 topic bw subprocess)
_rclpy_available = False
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    _rclpy_available = True
except ImportError:
    pass


def sizeof_fmt(num, suffix="B"):
    for unit in ("", "Ki", "Mi", "Gi", "Ti"):
        if abs(num) < 1024.0:
            return f"{num:6.1f}{unit}{suffix}"
        num /= 1024.0
    return f"{num:.1f}Ti{suffix}"


def get_cpu_all_threads():
    per_cpu = psutil.cpu_percent(interval=0, percpu=True)
    return list(per_cpu)


def get_cpu_summary():
    total = psutil.cpu_percent(interval=0)
    freq = psutil.cpu_freq()
    freq_mhz = freq.current if freq else None
    temp_c = None
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp_c = int(f.read().strip()) / 1000.0
    except (FileNotFoundError, OSError):
        pass
    return {"total_percent": total, "freq_mhz": freq_mhz, "temp_c": temp_c}


def get_ram():
    v = psutil.virtual_memory()
    return {
        "total": v.total,
        "available": v.available,
        "used": v.used,
        "percent": v.percent,
        "free": v.free,
    }


def run_iftop_text(interface="eth0", timeout_sec=6):
    iftop_bin = "/usr/sbin/iftop" if os.path.isfile("/usr/sbin/iftop") else "iftop"
    use_sudo = os.geteuid() != 0
    cmd = ["sudo", iftop_bin, "-t", "-s", "1", "-i", interface, "-n", "-N"] if use_sudo else [iftop_bin, "-t", "-s", "1", "-i", interface, "-n", "-N"]
    try:
        out = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            env={**os.environ, "LANG": "C"},
        )
        return (out.stdout or "") + (out.stderr or "")
    except (FileNotFoundError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
        return None


def parse_iftop_summary(raw):
    if not raw:
        return {}
    m = re.search(r"Total send and receive rate\s*[:\s]+([\d.]+\s*[KMG]?b)", raw, re.I)
    if m:
        return {"total_rate": m.group(1).strip()}
    return {}


def _ros2_source_cmd(topic, verb="bw"):
    """verb: 'bw' or 'delay'."""
    ros_distro = os.environ.get("ROS_DISTRO")
    if not ros_distro and os.path.isdir("/opt/ros"):
        for name in sorted(os.listdir("/opt/ros")):
            if os.path.isfile(f"/opt/ros/{name}/setup.bash"):
                ros_distro = name
                break
    ros_distro = ros_distro or "jazzy"
    ros_setup = f"/opt/ros/{ros_distro}/setup.bash"
    ws_setups = [
        "/home/ubuntu/ros2_ws/install/setup.bash",
        os.path.expanduser("~/ros2_ws/install/setup.bash"),
    ]
    ws_source = " ".join(f"source {s} 2>/dev/null" for s in ws_setups if os.path.isfile(s))
    return [
        "bash", "-c",
        f"source {ros_setup} 2>/dev/null; {ws_source}; stdbuf -oL ros2 topic {verb} {topic}",
    ]


def _parse_bw_line(line):
    m = re.search(r"([\d.]+)\s*([KM])?B/s", line, re.I)
    if not m:
        return None, False
    val = float(m.group(1))
    unit = (m.group(2) or "M").upper()
    if unit == "K":
        val = val / 1024.0
    return round(val, 3), True


def _parse_delay_line(line):
    """Parse 'average delay: 0.012' or 'min: 0.01s max: 0.02s' -> (avg_ms, min_ms, max_ms) or None."""
    out = {}
    m = re.search(r"average delay:\s*([\d.]+)", line, re.I)
    if m:
        out["avg_ms"] = round(float(m.group(1)) * 1000, 2)
    m_min = re.search(r"min:\s*([\d.]+)s", line, re.I)
    m_max = re.search(r"max:\s*([\d.]+)s", line, re.I)
    if m_min:
        out["min_ms"] = round(float(m_min.group(1)) * 1000, 2)
    if m_max:
        out["max_ms"] = round(float(m_max.group(1)) * 1000, 2)
    return out if out else None


_buffer = {"scan_mbps": None, "scan_delay": None, "iftop": {"eth0": None, "wlan0": None}}
_buffer_lock = threading.Lock()

_scan_msg_log = []
_scan_msg_log_lock = threading.Lock()
_SCAN_MSG_SIZE_APPROX = 24 * 1024  # 24 KB (LaserScan typical)

_scan_delay_log = []  # (timestamp, delay_sec) for rolling window
_scan_delay_log_lock = threading.Lock()
_scan_delay_window_sec = 5.0


def _thread_ros2_bw_rclpy(topic="/scan"):
    """Subscribe to /scan: compute bandwidth and delay (from header.stamp) in one thread."""
    if not _rclpy_available:
        with _buffer_lock:
            _buffer["scan_mbps"] = "rclpy not available"
            _buffer["scan_delay"] = "rclpy not available"
        return
    try:
        rclpy.init()
        node = Node("resource_monitor_bw")
        def cb(msg):
            now = time.time()
            with _scan_msg_log_lock:
                _scan_msg_log.append((now, _SCAN_MSG_SIZE_APPROX))
                while _scan_msg_log and now - _scan_msg_log[0][0] > 5:
                    _scan_msg_log.pop(0)
            # delay = now - msg timestamp (header.stamp)
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            delay_sec = now - stamp_sec
            with _scan_delay_log_lock:
                _scan_delay_log.append((now, delay_sec))
                while _scan_delay_log and now - _scan_delay_log[0][0] > _scan_delay_window_sec:
                    _scan_delay_log.pop(0)
        node.create_subscription(LaserScan, topic, cb, 10)
        last_update = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if now - last_update < 0.5:
                continue
            last_update = now
            with _scan_msg_log_lock:
                if _scan_msg_log:
                    t0 = _scan_msg_log[0][0]
                    window = now - t0
                    if window >= 0.3:
                        total_bytes = sum(s for _, s in _scan_msg_log)
                        bw_mbps = (total_bytes / window) / (1024 * 1024)
                        with _buffer_lock:
                            _buffer["scan_mbps"] = round(bw_mbps, 3)
            with _scan_delay_log_lock:
                if _scan_delay_log:
                    delays = [d for _, d in _scan_delay_log]
                    avg_ms = sum(delays) / len(delays) * 1000
                    min_ms = min(delays) * 1000
                    max_ms = max(delays) * 1000
                    with _buffer_lock:
                        _buffer["scan_delay"] = {"avg_ms": round(avg_ms, 2), "min_ms": round(min_ms, 2), "max_ms": round(max_ms, 2)}
    except Exception as e:
        with _buffer_lock:
            _buffer["scan_mbps"] = str(e)
            _buffer["scan_delay"] = str(e)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


def _thread_ros2_bw(topic="/scan", restart_delay=3):
    env = {**os.environ, "PYTHONUNBUFFERED": "1"}
    cmd = ["ros2", "topic", "bw", topic]
    if os.path.isfile("/opt/ros/jazzy/setup.bash") or os.path.isfile("/opt/ros/humble/setup.bash"):
        cmd = _ros2_source_cmd(topic)
    while True:
        try:
            master, slave = pty.openpty()
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=slave,
                    stderr=slave,
                    env=env,
                )
            except FileNotFoundError:
                os.close(slave)
                os.close(master)
                raise
            os.close(slave)
            buf = ""
            try:
                while proc.poll() is None:
                    r, _, _ = select.select([master], [], [], 0.5)
                    if not r:
                        continue
                    try:
                        data = os.read(master, 4096).decode("utf-8", errors="replace")
                    except OSError:
                        break
                    if not data:
                        break
                    buf += data
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        val, ok = _parse_bw_line(line + "\n")
                        if ok:
                            with _buffer_lock:
                                _buffer["scan_mbps"] = val
            finally:
                try:
                    os.close(master)
                except OSError:
                    pass
                proc.wait()
        except FileNotFoundError:
            with _buffer_lock:
                _buffer["scan_mbps"] = "ros2 not found (source ROS2 workspace?)"
            time.sleep(restart_delay)
            continue
        except Exception as e:
            with _buffer_lock:
                _buffer["scan_mbps"] = str(e)
        time.sleep(restart_delay)


def _thread_ros2_delay(topic="/scan", restart_delay=3):
    """Long-running: ros2 topic delay TOPIC via subprocess (PTY). Updates _buffer['scan_delay']."""
    env = {**os.environ, "PYTHONUNBUFFERED": "1"}
    cmd = ["ros2", "topic", "delay", topic]
    if os.path.isfile("/opt/ros/jazzy/setup.bash") or os.path.isfile("/opt/ros/humble/setup.bash"):
        cmd = _ros2_source_cmd(topic, verb="delay")
    while True:
        try:
            master, slave = pty.openpty()
            try:
                proc = subprocess.Popen(cmd, stdout=slave, stderr=slave, env=env)
            except FileNotFoundError:
                os.close(slave)
                os.close(master)
                raise
            os.close(slave)
            buf = ""
            current = {}
            try:
                while proc.poll() is None:
                    r, _, _ = select.select([master], [], [], 0.5)
                    if not r:
                        continue
                    try:
                        data = os.read(master, 4096).decode("utf-8", errors="replace")
                    except OSError:
                        break
                    if not data:
                        break
                    buf += data
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        parsed = _parse_delay_line(line)
                        if parsed:
                            current.update(parsed)
                            with _buffer_lock:
                                _buffer["scan_delay"] = dict(current)
            finally:
                try:
                    os.close(master)
                except OSError:
                    pass
                proc.wait()
        except FileNotFoundError:
            with _buffer_lock:
                _buffer["scan_delay"] = "ros2 not found (source ROS2 workspace?)"
            time.sleep(restart_delay)
            continue
        except Exception as e:
            with _buffer_lock:
                _buffer["scan_delay"] = str(e)
        time.sleep(restart_delay)


def _thread_iftop(interface, key, interval_sec=8):
    iftop_bin = "/usr/sbin/iftop" if os.path.isfile("/usr/sbin/iftop") else "iftop"
    use_sudo = os.geteuid() != 0
    cmd = ["sudo", iftop_bin, "-t", "-s", "1", "-i", interface, "-n", "-N"] if use_sudo else [iftop_bin, "-t", "-s", "1", "-i", interface, "-n", "-N"]
    while True:
        try:
            out = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10,
                env={**os.environ, "LANG": "C"},
            )
            raw = (out.stdout or "") + (out.stderr or "")
            parsed = parse_iftop_summary(raw)
            val = parsed.get("total_rate") if parsed else None
            with _buffer_lock:
                _buffer["iftop"][key] = val if val else "iftop failed or not installed"
        except (FileNotFoundError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
            with _buffer_lock:
                _buffer["iftop"][key] = "iftop failed or not installed"
        time.sleep(interval_sec)


def _read_buffer(use_iftop, interfaces, use_ros2_bw):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cpu_per_core = get_cpu_all_threads()
    cpu_summary = get_cpu_summary()
    ram = get_ram()
    with _buffer_lock:
        iftop_buf = dict(_buffer["iftop"])
        scan_buf = _buffer["scan_mbps"]
        scan_delay_buf = _buffer["scan_delay"]

    out = {
        "timestamp": ts,
        "cpu": {
            "percent_all_threads": cpu_per_core,
            "percent_total": cpu_summary["total_percent"],
            "freq_mhz": cpu_summary["freq_mhz"],
            "temp_c": cpu_summary["temp_c"],
        },
        "memory": {
            "total_bytes": ram["total"],
            "available_bytes": ram["available"],
            "used_bytes": ram["used"],
            "percent": ram["percent"],
            "free_bytes": ram["free"],
        },
    }
    if use_iftop:
        out["iftop"] = {}
        for iface in interfaces:
            v = iftop_buf.get(iface)
            out["iftop"][iface] = {"total_rate": v} if isinstance(v, str) and v and "failed" not in v else ({"error": v or "waiting..."} if v else {"error": "waiting..."})
    if use_ros2_bw:
        out["ros2_topic_bw"] = {"bandwidth_mbps": scan_buf} if isinstance(scan_buf, (int, float)) else {"error": scan_buf or "waiting..."}
        if isinstance(scan_delay_buf, dict):
            out["ros2_topic_delay"] = scan_delay_buf
        else:
            out["ros2_topic_delay"] = {"error": scan_delay_buf or "waiting..."}
    return out


def to_log_line(data):
    cpu = data["cpu"]
    line = {
        "timestamp": data["timestamp"],
        "CPU%": round(cpu["percent_total"], 1),
        "CPU%_threads": [round(x, 1) for x in (cpu.get("percent_all_threads") or [])],
        "RAM%": round(data["memory"]["percent"], 1),
    }
    if "iftop" in data:
        line["iftop"] = {}
        for iface, val in data["iftop"].items():
            line["iftop"][iface] = val.get("total_rate") or val.get("error", "")
    if "ros2_topic_bw" in data:
        bw = data["ros2_topic_bw"]
        line["scan_mbps"] = bw.get("bandwidth_mbps") if "error" not in bw else bw.get("error", "")
    if "ros2_topic_delay" in data:
        d = data["ros2_topic_delay"]
        line["scan_delay_ms"] = d if isinstance(d, dict) else {"error": d or "waiting..."}
    return line


def print_human(data, verbose=True):
    ts = data["timestamp"]
    cpu = data["cpu"]
    mem = data["memory"]
    print(f"\n--- {ts} ---")
    print("CPU:")
    temp_str = f"{cpu['temp_c']} °C" if cpu.get("temp_c") is not None else "N/A"
    print(f"  Total: {cpu['percent_total']:.1f}%  Freq: {cpu['freq_mhz']} MHz  Temp: {temp_str}")
    if verbose and cpu.get("percent_all_threads"):
        cores = cpu["percent_all_threads"]
        print(f"  Per thread ({len(cores)}): {[round(x, 1) for x in cores]}")
    print("RAM:")
    print(f"  Used: {mem['percent']:.1f}%  {sizeof_fmt(mem['used_bytes'])} / {sizeof_fmt(mem['total_bytes'])}  (free {sizeof_fmt(mem['available_bytes'])})")
    if "iftop" in data and data["iftop"]:
        print("iftop:", data["iftop"])
    if "ros2_topic_bw" in data:
        bw = data["ros2_topic_bw"]
        if "error" in bw:
            print("ros2 topic bw /scan:", bw["error"])
        else:
            print("ros2 topic bw /scan:", f"{bw.get('bandwidth_mbps')} MB/s")
    if "ros2_topic_delay" in data:
        d = data["ros2_topic_delay"]
        if "error" in d:
            print("ros2 topic delay /scan:", d["error"])
        else:
            print("ros2 topic delay /scan:", f"avg={d.get('avg_ms')}ms min={d.get('min_ms')}ms max={d.get('max_ms')}ms")


def main():
    parser = argparse.ArgumentParser(description="Continuous monitor: CPU, RAM, iftop, ros2 topic bw")
    parser.add_argument("--interval", type=float, default=1.0, help="Sampling interval in seconds")
    parser.add_argument("--no-iftop", action="store_true", help="Do not run iftop")
    parser.add_argument("--no-ros2-bw", action="store_true", help="Do not run ros2 topic bw /scan")
    parser.add_argument("--interfaces", nargs="+", default=["eth0", "wlan0"], metavar="IFACE", help="Interfaces for iftop")
    parser.add_argument("--json", action="store_true", help="Print each sample as one JSON line")
    parser.add_argument("--log", type=str, default="resource_monitor.log", metavar="FILE", help="Write to file. Disable: --log ''")
    parser.add_argument("--print", action="store_true", dest="print_console", help="Print results to console")
    args = parser.parse_args()

    use_iftop = not args.no_iftop
    use_ros2_bw = not args.no_ros2_bw

    _ = psutil.cpu_percent(interval=0, percpu=True)
    time.sleep(0.2)

    hz = 1.0 / args.interval
    print("Monitoring started (Ctrl+C to stop). Interval =", args.interval, "s (~{:.2f} Hz)".format(hz))
    if use_iftop:
        print("iftop: on, interfaces =", args.interfaces)
    else:
        print("iftop: off")
    if use_ros2_bw:
        if _rclpy_available:
            print("ros2 topic bw + delay: on, topic = /scan (using rclpy subscription)")
        else:
            print("ros2 topic bw + delay: on, topic = /scan (using ros2 topic bw/delay subprocess)")
    else:
        print("ros2 topic bw: off")

    log_path = args.log.strip() if args.log else ""
    log_file = None
    if log_path:
        log_file = open(log_path, "a", encoding="utf-8")
        print("Writing to file:", os.path.abspath(log_path))

    if use_iftop or use_ros2_bw:
        print("Background threads started; iftop/scan may show 'waiting...' until first result.")
    if use_iftop:
        for i, iface in enumerate(args.interfaces):
            t = threading.Thread(target=_thread_iftop, args=(iface, iface), kwargs={"interval_sec": 8}, daemon=True)
            t.start()
            if i == 1:
                time.sleep(4)
    if use_ros2_bw:
        if _rclpy_available:
            threading.Thread(target=_thread_ros2_bw_rclpy, args=("/scan",), daemon=True).start()
        else:
            threading.Thread(target=_thread_ros2_bw, args=("/scan",), daemon=True).start()
            threading.Thread(target=_thread_ros2_delay, args=("/scan",), daemon=True).start()

    do_print = args.print_console or args.json
    try:
        while True:
            data = _read_buffer(use_iftop, args.interfaces, use_ros2_bw)
            if do_print:
                if args.json:
                    print(json.dumps(data, default=str))
                else:
                    print_human(data)
            if log_file:
                log_file.write(json.dumps(to_log_line(data), default=str) + "\n")
                log_file.flush()
            time.sleep(max(0.5, args.interval))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if log_file:
            log_file.close()


if __name__ == "__main__":
    main()
