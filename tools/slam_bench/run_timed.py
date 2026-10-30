#!/usr/bin/env python3
"""Run a command, measure wall/CPU/max-RSS, and optionally stop it when idle.

Used for GLIM's bag reader, which does not exit when the bag ends: after the
CPU has been idle for a while we send SIGINT so it saves /tmp/dump.

Usage:
  run_timed.py --out result.json [--idle-stop] -- cmd args...
"""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

PAGE = os.sysconf('SC_PAGE_SIZE')
CLK = os.sysconf('SC_CLK_TCK')


def descendants(pid):
    out = {pid}
    stack = [pid]
    while stack:
        p = stack.pop()
        try:
            kids = Path(f'/proc/{p}/task/{p}/children').read_text().split()
        except OSError:
            continue
        for k in kids:
            k = int(k)
            if k not in out:
                out.add(k)
                stack.append(k)
    return out


def proc_stats(pid):
    cpu = 0.0
    rss = 0
    for p in descendants(pid):
        try:
            stat = Path(f'/proc/{p}/stat').read_text()
            rest = stat[stat.rfind(')') + 2:].split()
            cpu += (int(rest[11]) + int(rest[12])) / CLK
            rss += int(Path(f'/proc/{p}/statm').read_text().split()[1]) * PAGE
        except (OSError, IndexError, ValueError):
            continue
    return cpu, rss


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--idle-stop', action='store_true')
    ap.add_argument('--min-runtime', type=float, default=30.0)
    ap.add_argument('--idle-window', type=float, default=10.0)
    ap.add_argument('--idle-cpu', type=float, default=0.5)
    ap.add_argument('--max-runtime', type=float, default=900.0)
    ap.add_argument('cmd', nargs=argparse.REMAINDER)
    args = ap.parse_args()
    cmd = args.cmd
    if cmd and cmd[0] == '--':
        cmd = cmd[1:]

    start = time.monotonic()
    proc = subprocess.Popen(cmd, start_new_session=True)
    samples = []          # (elapsed, cumulative cpu)
    max_rss = 0
    interrupted = False
    last_cpu = 0.0

    while proc.poll() is None:
        elapsed = time.monotonic() - start
        cpu, rss = proc_stats(proc.pid)
        max_rss = max(max_rss, rss)
        last_cpu = cpu
        samples.append((elapsed, cpu))
        samples = [s for s in samples if elapsed - s[0] <= args.idle_window + 1]

        if args.idle_stop and not interrupted and elapsed > args.min_runtime:
            old = [s for s in samples if elapsed - s[0] >= args.idle_window]
            if old and (cpu - old[0][1]) < args.idle_cpu:
                print('idle; sending SIGINT', flush=True)
                os.killpg(proc.pid, signal.SIGINT)
                interrupted = True

        if elapsed > args.max_runtime:
            print('max runtime; sending SIGINT', flush=True)
            os.killpg(proc.pid, signal.SIGINT)
            interrupted = True
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
            break

        time.sleep(0.5)

    try:
        rc = proc.wait(timeout=60)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        rc = proc.wait()

    wall = time.monotonic() - start
    cpu, rss = proc_stats(proc.pid)
    result = {
        'cmd': cmd,
        'wall_s': round(wall, 2),
        'cpu_s': round(max(last_cpu, cpu), 2),
        'max_rss_mb': round(max_rss / (1024 * 1024), 1),
        'exit_code': rc,
        'interrupted': interrupted,
    }
    print(json.dumps(result, indent=2))
    Path(args.out).write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
