# infinitool

A one-stop management shell for a [PineTime](https://pine64.org/devices/pinetime/)
smartwatch running [InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime), over
Bluetooth LE. Files, clock and firmware, in a single connection.

It runs anywhere [bleak](https://github.com/hbldh/bleak) does — CoreBluetooth on macOS,
BlueZ on Linux, WinRT on Windows — which matters most on macOS, where there is no
companion app that can manage a PineTime and a browser cannot do it either (Chrome's Web
Bluetooth blocklist excludes Nordic's legacy DFU service).

Everything it uses is served by the normal running firmware; the watch never has to be
put into bootloader mode.

## Install

```sh
pip install bleak
```

## Use

```
$ ./infinitool.py
Found InfiniTime (C3764D0D-...)
InfiniTime 1.16.0  |  BLE FS v4  |  battery 87%
infinitool> ls /images
infinitool> cp !fuji.bin /images/fuji.bin
infinitool> time set
infinitool> flash !pinetime-mcuboot-app-dfu-1.16.0.zip
infinitool> exit
```

Plain paths are on the watch; a local path is prefixed with `!`, as in ftp/sftp. Any
command can also be run non-interactively, and `-c` is repeatable:

```sh
./infinitool.py -c "cp !fuji.bin /images/fuji.bin" -c "ls /images"
```

Run `help` at the prompt for the full command list. On the watch, all of this is gated
behind Settings → "Firmware & files", which must be enabled.

## Files

| | |
|---|---|
| `infinitool.py` | the tool: BLE filesystem, current time, device info, battery, `flash` |
| `dfu_bleak.py`  | Nordic legacy DFU over bleak; also usable standalone |
| `unpacker.py`   | unpacks a `*-dfu-*.zip` package (from ota-dfu-python) |

## Licence

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE) for the DFU code's provenance.
