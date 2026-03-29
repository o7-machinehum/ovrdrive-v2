#!/usr/bin/env python3
"""
Write-verify diagnostic for OVRDrive USB Mass Storage.

Writes known patterns to a raw block device, reads them back, and analyzes
any corruption to classify the failure mode. Optionally captures firmware
serial debug output in a background thread.

Usage:
    sudo python3 tests/write_verify.py --dev /dev/sdX --serial /dev/ttyACM0 --sectors 2000
"""

import argparse
import json
import os
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime

try:
    import serial
except ImportError:
    serial = None


def make_sector(lba):
    """Build a 512-byte sector: [4-byte LE LBA] * 127 + [0x00000000].

    127 copies (odd) means XOR checksum = LBA, giving non-zero firmware
    checksums for LBA > 0 that can be cross-referenced with serial output.
    """
    word = struct.pack("<I", lba)
    return word * 127 + b"\x00\x00\x00\x00"


def xor_checksum(data):
    """XOR-fold 512 bytes into a uint32 (matches firmware checksum)."""
    ck = 0
    for i in range(128):
        ck ^= struct.unpack_from("<I", data, i * 4)[0]
    return ck


def classify_sector(lba, expected, got, num_sectors):
    """Classify a single corrupt sector.

    Returns a dict with classification info.
    """
    info = {
        "lba": lba,
        "expected_first16": expected[:16].hex(),
        "got_first16": got[:16].hex(),
        "hex_dump": got.hex(),
    }

    # All zeros?
    if got == b"\x00" * 512:
        info["type"] = "zeroed"
        return info

    # Is it a copy of a different LBA's pattern?
    if len(got) >= 4:
        found_lba = struct.unpack_from("<I", got, 0)[0]
        candidate = make_sector(found_lba)
        if got == candidate and found_lba != lba and found_lba < num_sectors:
            info["type"] = "shifted"
            info["found_lba"] = found_lba
            info["shift"] = found_lba - lba
            return info

    # Byte-level diff
    mismatched_bytes = sum(1 for a, b in zip(expected, got) if a != b)
    bit_flips = 0
    for a, b in zip(expected, got):
        bit_flips += bin(a ^ b).count("1")

    info["mismatched_bytes"] = mismatched_bytes
    info["bit_flips"] = bit_flips

    if mismatched_bytes == bit_flips and bit_flips <= 16:
        info["type"] = "bit_flips"
    elif mismatched_bytes < 256:
        info["type"] = "partial_corrupt"
    else:
        info["type"] = "full_corrupt"

    return info


def serial_logger(port, baudrate, log_path, stop_event):
    """Background thread: read serial and write to log file."""
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
    except Exception as e:
        print(f"[serial] Failed to open {port}: {e}", file=sys.stderr)
        return

    with open(log_path, "w") as f:
        f.write(f"# Serial log started {datetime.now().isoformat()}\n")
        f.write(f"# Port: {port} @ {baudrate}\n\n")
        while not stop_event.is_set():
            try:
                line = ser.readline()
                if line:
                    decoded = line.decode("utf-8", errors="replace")
                    f.write(decoded)
                    f.flush()
            except Exception:
                pass
    ser.close()


def main():
    parser = argparse.ArgumentParser(
        description="Write-verify diagnostic for OVRDrive"
    )
    parser.add_argument(
        "--dev", required=True, help="Block device (e.g. /dev/sdX)"
    )
    parser.add_argument(
        "--serial", default=None, help="Serial port for firmware debug (e.g. /dev/ttyACM0)"
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--sectors", type=int, default=2000, help="Number of sectors to test (default: 2000)"
    )
    parser.add_argument(
        "--offset", type=int, default=0, help="Starting LBA offset (default: 0)"
    )
    args = parser.parse_args()

    num_sectors = args.sectors
    start_lba = args.offset
    dev = args.dev

    # Safety check
    if not os.path.exists(dev):
        print(f"Error: {dev} does not exist", file=sys.stderr)
        sys.exit(1)

    if not dev.startswith("/dev/sd") and not dev.startswith("/dev/loop"):
        print(f"Warning: {dev} doesn't look like a typical block device. Continue? [y/N]",
              end=" ", flush=True)
        if input().strip().lower() != "y":
            sys.exit(1)

    # Create results directory
    os.makedirs("results", exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Start serial logger if requested
    stop_serial = threading.Event()
    serial_thread = None
    serial_log_path = None

    if args.serial:
        if serial is None:
            print("Error: pyserial not installed. Run: pip install pyserial",
                  file=sys.stderr)
            sys.exit(1)
        serial_log_path = f"results/serial_{timestamp}.log"
        serial_thread = threading.Thread(
            target=serial_logger,
            args=(args.serial, args.baud, serial_log_path, stop_serial),
            daemon=True,
        )
        serial_thread.start()
        print(f"[serial] Logging to {serial_log_path}")
        time.sleep(0.5)  # Let serial settle

    # === Write phase ===
    print(f"\n--- Write phase: {num_sectors} sectors to {dev} (LBA {start_lba}..{start_lba + num_sectors - 1}) ---")

    # Pre-compute write data and checksums
    write_checksums = {}
    write_data = bytearray()
    for i in range(num_sectors):
        lba = start_lba + i
        sector = make_sector(lba)
        write_data.extend(sector)
        write_checksums[lba] = xor_checksum(sector)

    with open(dev, "wb") as f:
        f.seek(start_lba * 512)
        f.write(write_data)
        f.flush()
        os.fsync(f.fileno())

    print(f"  Written {num_sectors * 512} bytes ({num_sectors} sectors)")

    # Brief pause to let any firmware logging finish
    time.sleep(0.5)

    # Drop OS page cache so reads go to the actual device, not cached write data
    print("  Dropping OS page cache...")
    try:
        subprocess.run(["sh", "-c", "echo 3 > /proc/sys/vm/drop_caches"],
                       check=True, capture_output=True)
    except (subprocess.CalledProcessError, PermissionError) as e:
        print(f"  Warning: could not drop caches ({e}). Reads may come from cache.")

    # === Read phase ===
    print(f"\n--- Read phase: reading back {num_sectors} sectors ---")

    with open(dev, "rb") as f:
        f.seek(start_lba * 512)
        read_data = f.read(num_sectors * 512)

    if len(read_data) != num_sectors * 512:
        print(f"Error: expected {num_sectors * 512} bytes, got {len(read_data)}",
              file=sys.stderr)
        sys.exit(1)

    print(f"  Read {len(read_data)} bytes")

    # === Analysis phase ===
    print(f"\n--- Analysis ---")

    corrupt_sectors = []
    type_counts = {}

    for i in range(num_sectors):
        lba = start_lba + i
        expected = write_data[i * 512 : (i + 1) * 512]
        got = read_data[i * 512 : (i + 1) * 512]

        if expected != got:
            info = classify_sector(lba, expected, got, num_sectors)
            corrupt_sectors.append(info)
            t = info["type"]
            type_counts[t] = type_counts.get(t, 0) + 1

    # Print firmware checksums for cross-reference
    print(f"\n  Host-side XOR checksums (first 5 LBAs):")
    for i in range(min(5, num_sectors)):
        lba = start_lba + i
        print(f"    LBA {lba}: {write_checksums[lba]:08x}")

    # Summary
    print(f"\n  Total sectors tested: {num_sectors}")
    print(f"  Corrupt sectors:     {len(corrupt_sectors)}")

    if corrupt_sectors:
        print(f"\n  Corruption types:")
        for t, count in sorted(type_counts.items(), key=lambda x: -x[1]):
            print(f"    {t:20s}: {count}")

        print(f"\n  First 10 corrupt sectors:")
        for info in corrupt_sectors[:10]:
            lba = info["lba"]
            print(f"    LBA {lba:6d}: type={info['type']:20s}  "
                  f"expected={info['expected_first16']}  "
                  f"got={info['got_first16']}")
            if info["type"] == "shifted":
                print(f"             contains LBA {info['found_lba']} data (shift={info['shift']})")

        # Check for patterns in corrupt LBAs
        corrupt_lbas = [s["lba"] for s in corrupt_sectors]
        if len(corrupt_lbas) >= 2:
            diffs = [corrupt_lbas[i + 1] - corrupt_lbas[i]
                     for i in range(len(corrupt_lbas) - 1)]
            if len(set(diffs)) == 1:
                print(f"\n  Pattern: corrupt sectors are evenly spaced (every {diffs[0]} sectors)")
            elif all(d > 0 for d in diffs):
                # Check if they cluster at chunk boundaries (80 sectors)
                at_boundary = [lba for lba in corrupt_lbas if lba % 80 < 3 or lba % 80 > 77]
                if len(at_boundary) > len(corrupt_lbas) * 0.5:
                    print(f"\n  Pattern: corruption clusters at 80-sector chunk boundaries")
    else:
        print("\n  No corruption detected!")

    # Save results
    results = {
        "timestamp": timestamp,
        "device": dev,
        "start_lba": start_lba,
        "num_sectors": num_sectors,
        "total_corrupt": len(corrupt_sectors),
        "type_counts": type_counts,
        "host_checksums_sample": {
            str(start_lba + i): f"{write_checksums[start_lba + i]:08x}"
            for i in range(min(10, num_sectors))
        },
        "corrupt_sectors": corrupt_sectors,
    }

    results_path = f"results/corruption_{timestamp}.json"
    with open(results_path, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\n  Results saved to {results_path}")
    if serial_log_path:
        print(f"  Serial log saved to {serial_log_path}")

    # Stop serial logger
    if serial_thread:
        stop_serial.set()
        serial_thread.join(timeout=2)

    # Exit code: 0 if no corruption, 1 if corrupt
    sys.exit(1 if corrupt_sectors else 0)


if __name__ == "__main__":
    main()
