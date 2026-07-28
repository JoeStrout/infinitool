# infinitool

A one-stop management shell for a [PineTime](https://pine64.org/devices/pinetime/)
smartwatch running [InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime), over
Bluetooth LE.  It provides the following functionality:

- flash new firmware
- view/update files ("external resources")
- get/set time and date

In short: it manages files, clock, and firmware, in a single connection.

It runs anywhere Python and the [bleak](https://github.com/hbldh/bleak) module are available — communicating with the device via CoreBluetooth on macOS, BlueZ on Linux, and WinRT on Windows.

Everything it uses is served by the normal running firmware; the watch never has to be
put into bootloader mode.

## Prerequisites

You will need a standard Python (3.10+) installation, and [bleak](https://github.com/hbldh/bleak):

```sh
pip install bleak
```

The 3.10 floor is bleak's, not this tool's. Developed and tested against bleak 3.0.2 on
Python 3.12 (macOS/CoreBluetooth); Linux and Windows should work, as bleak covers BlueZ
and WinRT, but neither has been tried. Older bleak releases that don't expose
mtu_size fall back to 23-byte transfers, which works but is slow.

Also, in order to connect successfully to your watch, you will need to go (on the watch) to `Settings` → `Over the Air` and enable "Firmware & files".

## Sample usage

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
command can also be run non-interactively using `-c`:

```sh
./infinitool.py -c "cp !fuji.bin /images/fuji.bin" -c "ls /images"
```

Run `help` at the prompt for the command list — one line each — or `help COMMAND`
(e.g. `help cp`) for the detail on any one of them.

## Files

| | |
|---|---|
| `infinitool.py` | the tool: BLE filesystem, current time, device info, battery, `flash` |
| `dfu_bleak.py`  | Nordic legacy DFU over bleak; also usable standalone |
| `unpacker.py`   | unpacks a `*-dfu-*.zip` package (from ota-dfu-python) |

## Licence

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE) for the DFU code's provenance.
