#!/usr/bin/env python3
"""
Speed benchmark for OVRDrive USB Mass Storage.

Tests sequential read/write throughput at various block sizes using
direct I/O (O_DIRECT) to bypass OS page cache.

WARNING: Write tests are DESTRUCTIVE - all data on the device will be lost.

Usage:
    sudo python3 tests/speed_test.py --dev /dev/sdX
    sudo python3 tests/speed_test.py --dev /dev/sdX --read-only
    sudo python3 tests/speed_test.py --dev /dev/sdX --size 32  # 32 MB test
"""

import argparse
import os
import subprocess
import sys
import time


def drop_caches():
    """Drop OS page cache to ensure reads hit the device."""
    try:
        subprocess.run(["sh", "-c", "echo 3 > /proc/sys/vm/drop_caches"],
                       check=True, capture_output=True)
    except (subprocess.CalledProcessError, PermissionError):
        pass


def run_dd(args, timeout=120):
    """Run dd and parse throughput from stderr."""
    try:
        result = subprocess.run(
            ["dd"] + args,
            capture_output=True, timeout=timeout
        )
    except subprocess.TimeoutExpired:
        return "TIMEOUT"
    stderr = result.stderr.decode("utf-8", errors="replace")
    # dd prints: "X bytes (Y MB, Z MiB) copied, T s, R MB/s"
    for line in stderr.strip().split("\n"):
        if "copied" in line and "/s" in line:
            return line.strip()
    return stderr.strip()


def test_read(dev, block_size_kb, total_mb):
    """Sequential read test with given block size."""
    bs = block_size_kb * 1024
    count = (total_mb * 1024 * 1024) // bs
    drop_caches()
    return run_dd([
        f"if={dev}",
        "of=/dev/null",
        f"bs={bs}",
        f"count={count}",
    ])


def test_write(dev, block_size_kb, total_mb):
    """Sequential write test with given block size."""
    bs = block_size_kb * 1024
    count = (total_mb * 1024 * 1024) // bs
    drop_caches()
    return run_dd([
        "if=/dev/zero",
        f"of={dev}",
        f"bs={bs}",
        f"count={count}",
        "conv=fsync",
    ])


def main():
    parser = argparse.ArgumentParser(description="OVRDrive speed benchmark")
    parser.add_argument("--dev", required=True, help="Block device (e.g. /dev/sdX)")
    parser.add_argument("--size", type=int, default=16, help="Test size in MB (default: 16)")
    parser.add_argument("--read-only", action="store_true", help="Skip write tests")
    args = parser.parse_args()

    dev = args.dev
    total_mb = args.size

    if not os.path.exists(dev):
        print(f"Error: {dev} does not exist", file=sys.stderr)
        sys.exit(1)

    if not dev.startswith("/dev/sd") and not dev.startswith("/dev/loop"):
        print(f"Warning: {dev} doesn't look like a typical block device. Continue? [y/N]",
              end=" ", flush=True)
        if input().strip().lower() != "y":
            sys.exit(1)

    block_sizes = [64, 128, 512, 1024]  # KB

    print(f"OVRDrive Speed Benchmark")
    print(f"Device: {dev}  Test size: {total_mb} MB")
    print(f"{'='*60}")

    # --- Read tests ---
    print(f"\n--- Sequential Read ---")
    print(f"{'Block Size':>12s}  Result")
    print(f"{'-'*12}  {'-'*44}")
    for bs in block_sizes:
        result = test_read(dev, bs, total_mb)
        print(f"{bs:>10d} KB  {result}")

    # --- Write tests ---
    if not args.read_only:
        print(f"\n--- Sequential Write (DESTRUCTIVE) ---")
        print(f"{'Block Size':>12s}  Result")
        print(f"{'-'*12}  {'-'*44}")
        for bs in block_sizes:
            result = test_write(dev, bs, total_mb)
            print(f"{bs:>10d} KB  {result}")

    # --- hdparm quick test ---
    print(f"\n--- hdparm buffered read ---")
    try:
        result = subprocess.run(["hdparm", "-t", dev],
                                capture_output=True, timeout=30)
        for line in result.stdout.decode().strip().split("\n"):
            if "MB/s" in line or "Timing" in line:
                print(f"  {line.strip()}")
    except FileNotFoundError:
        print("  hdparm not installed, skipping")
    except subprocess.TimeoutExpired:
        print("  hdparm timed out")

    # --- Expected speeds ---
    print(f"\n--- Expected Throughput ---")
    print(f"  SD @ LOWEMMCCLK  (~15 MHz, 4-bit):  ~7 MB/s")
    print(f"  SD @ EMMCCLK_48  (~22 MHz, 4-bit): ~10 MB/s")
    print(f"  SD @ EMMCCLK_96  (~40 MHz, 4-bit): ~20 MB/s")
    print(f"  USB2 HS bulk max:                   ~40 MB/s")
    print(f"  USB3 SS bulk max:                  ~400 MB/s")
    print(f"  Bottleneck is the SD clock speed.")


if __name__ == "__main__":
    main()
