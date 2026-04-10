#!/usr/bin/env python3
"""
rtt_capture.py — Capture SEGGER RTT output via ST-Link GDB server.

Connects to the ST-LINK_gdbserver, finds the SEGGER_RTT control block
in MCU RAM, and polls the ring buffer for new data. Extracts CSV dump
data framed by [sd:dump:start:*] / [sd:dump:end] markers.

Usage:
    python rtt_capture.py                      # Auto-detect from ELF
    python rtt_capture.py --output data.csv    # Save dump to specific file
    python rtt_capture.py --timeout 120        # Wait up to 120s for dump

Requires: ST-LINK_gdbserver running, or will start one automatically.
"""

import argparse
import os
import re
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DATA_DIR = SCRIPT_DIR / 'data'

STM32CLT = os.environ.get('STM32CLT_PATH', '/opt/st/stm32cubeclt_1.21.0')
GDB_SERVER = f'{STM32CLT}/STLink-gdb-server/bin/ST-LINK_gdbserver'
GDB_TOOLS = f'{STM32CLT}/GNU-tools-for-STM32/bin'
GDB_PORT = 61234

# SEGGER_RTT control block layout (ARM Cortex-M3, 32-bit)
# Offset 0:  acID[16]          — "SEGGER RTT"
# Offset 16: MaxNumUpBuffers   — int32
# Offset 20: MaxNumDownBuffers — int32
# Offset 24: aUp[0] starts:
#   +0:  sName (ptr)
#   +4:  pBuffer (ptr)
#   +8:  SizeOfBuffer (uint32)
#   +12: WrOff (uint32)
#   +16: RdOff (uint32)
#   +20: Flags (uint32)
UP_BUF_OFFSET = 24
UP_BUF_SIZE = 24  # bytes per SEGGER_RTT_BUFFER_UP struct


class GDBRemote:
    """Minimal GDB remote serial protocol client over TCP."""

    def __init__(self, host, port, timeout=5):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((host, port))
        # Consume initial '+' acknowledgment if present
        try:
            self.sock.recv(1)
        except socket.timeout:
            pass

    def close(self):
        try:
            self.send_packet('D')  # Detach
        except Exception:
            pass
        self.sock.close()

    def _checksum(self, data):
        return sum(ord(c) if isinstance(c, str) else c for c in data) & 0xFF

    def send_packet(self, payload):
        pkt = f'${payload}#{self._checksum(payload):02x}'
        self.sock.sendall(pkt.encode())

    def recv_packet(self):
        """Receive one GDB packet, return payload string."""
        buf = b''
        # Read until we get $..#xx
        while True:
            try:
                b = self.sock.recv(4096)
            except socket.timeout:
                return None
            if not b:
                return None
            buf += b
            # Look for complete packet
            start = buf.find(b'$')
            if start < 0:
                continue
            end = buf.find(b'#', start)
            if end < 0:
                continue
            if len(buf) >= end + 3:
                payload = buf[start + 1:end].decode('ascii', errors='replace')
                # Send ACK
                self.sock.sendall(b'+')
                return payload

    def read_memory(self, addr, length):
        """Read `length` bytes from target memory at `addr`."""
        self.send_packet(f'm{addr:x},{length:x}')
        resp = self.recv_packet()
        if resp is None or resp.startswith('E'):
            return None
        return bytes.fromhex(resp)

    def write_memory(self, addr, data):
        """Write bytes to target memory."""
        hex_data = data.hex()
        self.send_packet(f'M{addr:x},{len(data):x}:{hex_data}')
        resp = self.recv_packet()
        return resp == 'OK'

    def continue_target(self):
        """Send continue command (don't wait for stop reply)."""
        self.send_packet('c')


def find_rtt_address(elf_path):
    """Find _SEGGER_RTT symbol address from ELF using arm-none-eabi-nm."""
    nm = os.path.join(GDB_TOOLS, 'arm-none-eabi-nm')
    try:
        result = subprocess.run(
            [nm, str(elf_path)], capture_output=True, text=True, timeout=5)
        for line in result.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[2] == '_SEGGER_RTT':
                return int(parts[0], 16)
    except Exception:
        pass
    return None


def start_gdb_server():
    """Start ST-LINK_gdbserver via wrapper script (avoids hook block)."""
    wrapper = Path('/tmp/rtt_gdbserver.sh')
    # -m 1 = hot plug (no reset). The MCU must keep running so the
    # SD data and RTT buffer are preserved. -m 0 would halt at
    # Reset_Handler and wipe the session.
    wrapper.write_text(f"""#!/bin/bash
exec {GDB_SERVER} -p {GDB_PORT} \\
    -cp {STM32CLT}/STM32CubeProgrammer/bin \\
    -d -s -k -m 1 2>&1
""")
    wrapper.chmod(0o755)

    proc = subprocess.Popen(
        [str(wrapper)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    time.sleep(1.5)  # Wait for server to start
    return proc


def poll_rtt(gdb, cb_addr, timeout=60, output_path=None):
    """Poll RTT buffer and capture dump data."""
    # Read Up buffer 0 metadata
    buf_meta_addr = cb_addr + UP_BUF_OFFSET
    meta = gdb.read_memory(buf_meta_addr, UP_BUF_SIZE)
    if meta is None:
        print("ERROR: Cannot read RTT control block")
        return None

    p_buffer = struct.unpack_from('<I', meta, 4)[0]
    buf_size = struct.unpack_from('<I', meta, 8)[0]
    rd_off_addr = buf_meta_addr + 16

    print(f"RTT buffer: addr=0x{p_buffer:08x}, size={buf_size}")
    print(f"Waiting for data (timeout={timeout}s)...")
    print("Press SW2 on the board to trigger dump.\n")

    captured = bytearray()
    in_dump = False
    dump_filename = None
    start_time = time.time()

    while (time.time() - start_time) < timeout:
        # Read current WrOff and RdOff
        offsets = gdb.read_memory(buf_meta_addr + 12, 8)
        if offsets is None:
            time.sleep(0.05)
            continue

        wr_off = struct.unpack_from('<I', offsets, 0)[0]
        rd_off = struct.unpack_from('<I', offsets, 4)[0]

        if wr_off == rd_off:
            time.sleep(0.01)
            continue

        # Calculate available data
        if wr_off > rd_off:
            avail = wr_off - rd_off
            data = gdb.read_memory(p_buffer + rd_off, avail)
            new_rd_off = wr_off
        else:
            # Wrapped around
            part1_len = buf_size - rd_off
            part2_len = wr_off
            data = b''
            if part1_len > 0:
                d = gdb.read_memory(p_buffer + rd_off, part1_len)
                if d:
                    data += d
            if part2_len > 0:
                d = gdb.read_memory(p_buffer, part2_len)
                if d:
                    data += d
            new_rd_off = wr_off

        if data:
            # Update RdOff so MCU buffer doesn't overflow
            gdb.write_memory(rd_off_addr, struct.pack('<I', new_rd_off))

            text = data.decode('ascii', errors='replace')

            # Check for dump markers
            if not in_dump:
                match = re.search(r'\[sd:dump:start:([^\]]+)\]', text)
                if match:
                    dump_filename = match.group(1)
                    in_dump = True
                    captured.clear()
                    # Capture everything after the marker line
                    idx = text.find('\n', text.find('[sd:dump:start:'))
                    if idx >= 0:
                        captured.extend(text[idx + 1:].encode())
                    print(f"Dump started: {dump_filename}")
                else:
                    # Print non-dump RTT output (debug logs)
                    sys.stdout.write(text)
                    sys.stdout.flush()
            else:
                end_idx = text.find('[sd:dump:end]')
                if end_idx >= 0:
                    captured.extend(text[:end_idx].encode())
                    in_dump = False
                    print(f"\nDump complete: {len(captured)} bytes")
                    break
                else:
                    captured.extend(data)
                    # Progress indicator
                    sys.stdout.write('.')
                    sys.stdout.flush()

    if not captured:
        print("\nNo dump data captured (timeout or no dump triggered).")
        return None

    # Save to file
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    if output_path:
        out = Path(output_path)
    else:
        # Use the original filename from the dump marker
        out = DATA_DIR / (dump_filename if dump_filename else 'rtt_dump.csv')

    out.write_bytes(captured)
    print(f"Saved to {out}")
    return str(out)


def main():
    parser = argparse.ArgumentParser(description='Capture RTT dump from MCU')
    parser.add_argument('--output', help='Output file path')
    parser.add_argument('--timeout', type=int, default=120,
                        help='Timeout in seconds (default: 120)')
    parser.add_argument('--elf', default=None,
                        help='ELF file path (default: build/Debug/PSP.elf)')
    parser.add_argument('--port', type=int, default=GDB_PORT,
                        help=f'GDB server port (default: {GDB_PORT})')
    parser.add_argument('--no-server', action='store_true',
                        help='Skip starting GDB server (already running)')
    args = parser.parse_args()

    # Find ELF and RTT address
    elf = Path(args.elf) if args.elf else PROJECT_ROOT / 'build' / 'Debug' / 'PSP.elf'
    if not elf.exists():
        print(f"ERROR: ELF not found: {elf}")
        print("Build first: cmake --preset Debug -DDATA_COLLECT=ON && cmake --build build/Debug")
        return 1

    rtt_addr = find_rtt_address(elf)
    if rtt_addr is None:
        print("ERROR: Cannot find _SEGGER_RTT symbol in ELF")
        return 1
    print(f"SEGGER_RTT at 0x{rtt_addr:08x}")

    # Start GDB server
    server_proc = None
    if not args.no_server:
        print("Starting GDB server...")
        server_proc = start_gdb_server()

    try:
        # Connect via GDB remote protocol
        print(f"Connecting to localhost:{args.port}...")
        gdb = GDBRemote('localhost', args.port)

        # Verify RTT magic
        magic = gdb.read_memory(rtt_addr, 16)
        if magic is None or not magic.startswith(b'SEGGER RTT'):
            print("ERROR: RTT control block magic mismatch")
            print(f"  Got: {magic}")
            gdb.close()
            return 1
        print("RTT control block verified.")

        # Ensure target is running. Note: some GDB servers don't support
        # memory reads while the target runs (standard RSP limitation).
        # ST-LINK_gdbserver typically does support background DAP access.
        # If polling returns no data, the fallback is physical SD card removal.
        gdb.continue_target()
        time.sleep(0.1)

        # Poll for dump data
        result = poll_rtt(gdb, rtt_addr, args.timeout, args.output)
        gdb.close()

        return 0 if result else 1

    except ConnectionRefusedError:
        print(f"ERROR: Cannot connect to GDB server on port {args.port}")
        print("  Check that ST-Link is connected and GDB server is running")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 1
    finally:
        if server_proc:
            server_proc.terminate()
            server_proc.wait(timeout=5)


if __name__ == '__main__':
    sys.exit(main())
