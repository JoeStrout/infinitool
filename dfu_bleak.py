#!/usr/bin/env python3
"""
Nordic legacy DFU over BLE, using bleak instead of BlueZ's gatttool.

The dfu.py it grew out of (in ota-dfu-python, see NOTICE) drives `gatttool` via pexpect,
which limits it to Linux. This script speaks the same legacy DFU protocol through bleak,
which uses CoreBluetooth on macOS, BlueZ on Linux and WinRT on Windows.

Motivation: on macOS there is no companion app that can flash InfiniTime, and a browser
cannot do it either -- Chrome's Web Bluetooth GATT blocklist explicitly excludes Nordic's
legacy DFU service (00001530-1212-efde-1523-785feabcd123) so that web pages cannot reflash
hardware. Native code is the only option.

The protocol implemented here mirrors ble_legacy_dfu_controller.py, and was cross-checked
against the firmware's own implementation in src/components/ble/DfuService.cpp:

  - DfuService starts in States::Idle and accepts StartDFU directly, so there is no
    "switch to DFU mode" reset step -- the running application serves DFU itself.
  - The start packet is 12 bytes: softdevice size, bootloader size, application size,
    each uint32 little-endian (DfuService.cpp:132-134). Only the application size is
    non-zero, so the value is prepended with 8 zero bytes.
  - Packet receipt notifications carry the byte count as uint32 LE (DfuService.cpp:185-189).

Usage:
    pip install bleak
    ./dfu_bleak.py -z pinetime-mcuboot-app-dfu-1.16.0.zip

After a successful flash the watch reboots into the new firmware, but the image is NOT yet
validated: on the watch go to quick settings -> cog -> Firmware and validate it, otherwise
the next reset rolls back to the previous firmware.
"""

import argparse
import asyncio
import struct
import sys
import time

from unpacker import Unpacker

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("bleak is not installed. Run: pip install bleak")


UUID_CONTROL_POINT = "00001531-1212-efde-1523-785feabcd123"
UUID_PACKET = "00001532-1212-efde-1523-785feabcd123"

# Control point opcodes (Nordic legacy DFU)
START_DFU = 0x01
INITIALIZE_DFU = 0x02
RECEIVE_FIRMWARE_IMAGE = 0x03
VALIDATE_FIRMWARE = 0x04
ACTIVATE_IMAGE_AND_RESET = 0x05
PRN_REQUEST = 0x08
RESPONSE = 0x10
PACKET_RECEIPT_NOTIFICATION = 0x11

UPDATE_APPLICATION = 0x04

PROCEDURE_NAMES = {
    START_DFU: "START_DFU",
    INITIALIZE_DFU: "INITIALIZE_DFU",
    RECEIVE_FIRMWARE_IMAGE: "RECEIVE_FIRMWARE_IMAGE",
    VALIDATE_FIRMWARE: "VALIDATE_FIRMWARE",
    ACTIVATE_IMAGE_AND_RESET: "ACTIVATE_IMAGE_AND_RESET",
    PRN_REQUEST: "PRN_REQUEST",
}

RESPONSE_NAMES = {
    1: "SUCCESS",
    2: "INVALID_STATE",
    3: "NOT_SUPPORTED",
    4: "DATA_SIZE_EXCEEDS_LIMITS",
    5: "CRC_ERROR",
    6: "OPERATION_FAILED",
}


class DfuError(Exception):
    pass


class LegacyDfu:
    def __init__(self, client, chunk_size, prn_interval, timeout, verbose):
        self.client = client
        self.chunk_size = chunk_size
        self.prn_interval = prn_interval
        self.timeout = timeout
        self.verbose = verbose
        self.notifications = asyncio.Queue()

    def log(self, message):
        if self.verbose:
            print(f"  [dfu] {message}")

    def _on_notify(self, _characteristic, data):
        self.notifications.put_nowait(bytes(data))

    async def _next_notification(self):
        try:
            return await asyncio.wait_for(self.notifications.get(), self.timeout)
        except asyncio.TimeoutError:
            raise DfuError(f"no notification from the watch within {self.timeout}s")

    async def _expect_success(self, procedure):
        """Wait for a RESPONSE notification and raise unless it reports SUCCESS."""
        while True:
            data = await self._next_notification()
            if not data:
                continue
            if data[0] == PACKET_RECEIPT_NOTIFICATION:
                # A straggling receipt from the previous burst; not what we're waiting for.
                continue
            if data[0] != RESPONSE or len(data) < 3:
                raise DfuError(f"unexpected notification: {data.hex()}")
            proc, result = data[1], data[2]
            name = PROCEDURE_NAMES.get(proc, f"0x{proc:02x}")
            if result != 1:
                reason = RESPONSE_NAMES.get(result, f"0x{result:02x}")
                raise DfuError(f"{name} failed: {reason}")
            self.log(f"{name} -> SUCCESS")
            if proc != procedure:
                self.log(f"note: expected {PROCEDURE_NAMES.get(procedure)}, got {name}")
            return

    async def _write_control(self, *values):
        await self.client.write_gatt_char(UUID_CONTROL_POINT, bytes(values), response=True)

    async def _write_packet(self, payload):
        await self.client.write_gatt_char(UUID_PACKET, bytes(payload), response=False)

    async def run(self, firmware, init_packet):
        await self.client.start_notify(UUID_CONTROL_POINT, self._on_notify)

        print(f"Firmware image: {len(firmware)} bytes, init packet: {len(init_packet)} bytes")

        # 1. Start DFU, application only.
        self.log("START_DFU")
        await self._write_control(START_DFU, UPDATE_APPLICATION)

        # 2. Image sizes: softdevice and bootloader are zero, application last.
        #    See DfuService.cpp:132-134 for the layout the firmware expects.
        await self._write_packet(bytes(8) + struct.pack("<I", len(firmware)))
        print("Waiting for the watch to erase flash (this can take a few seconds)...")
        await self._expect_success(START_DFU)

        # 3. Init packet (the .dat from the DFU zip).
        self.log("INITIALIZE_DFU (receive init packet)")
        await self._write_control(INITIALIZE_DFU, 0x00)
        await self._write_packet(init_packet)
        self.log("INITIALIZE_DFU (init packet complete)")
        await self._write_control(INITIALIZE_DFU, 0x01)
        await self._expect_success(INITIALIZE_DFU)

        # 4. Ask for a receipt every prn_interval packets. This is the only backpressure
        #    in the protocol, so don't disable it.
        self.log(f"PRN_REQUEST interval={self.prn_interval}")
        await self._write_control(PRN_REQUEST, *struct.pack("<H", self.prn_interval))

        # 5. Stream the image.
        self.log("RECEIVE_FIRMWARE_IMAGE")
        await self._write_control(RECEIVE_FIRMWARE_IMAGE)

        total = len(firmware)
        sent = 0
        packets = 0
        started = time.monotonic()

        for offset in range(0, total, self.chunk_size):
            chunk = firmware[offset:offset + self.chunk_size]
            await self._write_packet(chunk)
            sent += len(chunk)
            packets += 1

            if sent == total:
                break

            if packets % self.prn_interval == 0:
                data = await self._next_notification()
                if data and data[0] == PACKET_RECEIPT_NOTIFICATION and len(data) >= 5:
                    acknowledged = struct.unpack_from("<I", data, 1)[0]
                    if acknowledged != sent:
                        raise DfuError(f"watch acknowledged {acknowledged} bytes, we sent {sent}")
                elif data and data[0] == RESPONSE:
                    # An error arrived mid-transfer; decode it for a useful message.
                    proc, result = data[1], data[2]
                    raise DfuError(
                        f"{PROCEDURE_NAMES.get(proc, proc)} failed mid-transfer: "
                        f"{RESPONSE_NAMES.get(result, result)}"
                    )
                else:
                    raise DfuError(f"unexpected notification during transfer: {data.hex()}")
                progress(sent, total, started)

        progress(total, total, started)
        print()

        print("Waiting for image-received confirmation...")
        await self._expect_success(RECEIVE_FIRMWARE_IMAGE)

        # 6. Validate, then activate and reset.
        self.log("VALIDATE_FIRMWARE")
        await self._write_control(VALIDATE_FIRMWARE)
        await self._expect_success(VALIDATE_FIRMWARE)

        await asyncio.sleep(1)

        print("Activating image and resetting the watch...")
        try:
            await self._write_control(ACTIVATE_IMAGE_AND_RESET)
        except Exception as exc:
            # The watch reboots immediately, so losing the link here is the expected outcome.
            self.log(f"disconnect during activate (expected): {exc}")


def progress(sent, total, started):
    fraction = sent / total
    filled = int(40 * fraction)
    elapsed = time.monotonic() - started
    rate = (sent / 1024 / elapsed) if elapsed > 0 else 0
    print(
        f"\r  [{'#' * filled}{'.' * (40 - filled)}] {fraction * 100:5.1f}%  "
        f"{sent}/{total} B  {rate:.1f} KiB/s",
        end="",
        flush=True,
    )


async def find_device(name, address, scan_time):
    if address:
        print(f"Looking for device {address}...")
        device = await BleakScanner.find_device_by_address(address, timeout=scan_time)
        if device is None:
            raise DfuError(f"no device with address/UUID {address}")
        return device

    print(f"Scanning for a device named {name!r} ({scan_time:.0f}s)...")
    # macOS does not expose BLE MAC addresses, so matching by advertised name is the
    # practical option; CoreBluetooth identifies peripherals by an opaque per-host UUID.
    device = await BleakScanner.find_device_by_filter(
        lambda d, _adv: bool(d.name) and name.lower() in d.name.lower(),
        timeout=scan_time,
    )
    if device is None:
        raise DfuError(
            f"no device named {name!r} found. Make sure the watch is awake, in range, "
            "and not already connected to another device."
        )
    return device


async def main_async(args):
    unpacker = Unpacker()
    try:
        binfile, datfile = unpacker.unpack_zipfile(args.zip)
        with open(binfile, "rb") as handle:
            firmware = handle.read()
        with open(datfile, "rb") as handle:
            init_packet = handle.read()
    except Exception as exc:
        raise DfuError(f"could not read DFU package {args.zip}: {exc}")

    device = await find_device(args.name, args.address, args.scan_time)
    print(f"Found {device.name} ({device.address})")

    async with BleakClient(device) as client:
        print("Connected.")
        services = client.services
        if services.get_characteristic(UUID_CONTROL_POINT) is None:
            raise DfuError(
                "the DFU control point characteristic is missing. Is this an InfiniTime "
                "watch, and is it running firmware (not the bootloader)?"
            )

        dfu = LegacyDfu(client, args.chunk_size, args.prn, args.timeout, args.verbose)
        await dfu.run(firmware, init_packet)

    unpacker.delete()
    print()
    print("Done. The watch is rebooting into the new firmware.")
    print("IMPORTANT: the image is not validated yet. On the watch, swipe right ->")
    print("cog -> Firmware -> validate, or the next reset will roll it back.")


def main():
    parser = argparse.ArgumentParser(
        description="Flash an InfiniTime DFU zip over BLE using bleak "
                    "(macOS, Linux, Windows).",
    )
    parser.add_argument("-z", "--zip", required=True, help="path to the *-dfu-*.zip package")
    parser.add_argument("-n", "--name", default="InfiniTime",
                        help="advertised name to look for (default: InfiniTime)")
    parser.add_argument("-a", "--address", default=None,
                        help="connect to a specific address/UUID instead of scanning by name")
    parser.add_argument("--scan-time", type=float, default=10.0,
                        help="seconds to scan for the watch (default: 10)")
    parser.add_argument("--chunk-size", type=int, default=20,
                        help="bytes per packet write (default: 20, the legacy DFU default)")
    parser.add_argument("--prn", type=int, default=10,
                        help="packets between receipt notifications (default: 10)")
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="seconds to wait for a notification (default: 60)")
    parser.add_argument("-v", "--verbose", action="store_true", help="log each protocol step")
    args = parser.parse_args()

    if args.prn < 1:
        sys.exit("--prn must be at least 1; receipt notifications provide the only backpressure")

    try:
        asyncio.run(main_async(args))
    except DfuError as exc:
        sys.exit(f"\nDFU failed: {exc}")
    except KeyboardInterrupt:
        sys.exit("\nInterrupted. The watch will roll back to its previous firmware on reset.")


if __name__ == "__main__":
    main()
