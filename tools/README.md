# Tools — Modbus RTU + UART2 config

Bench test scripts for the STM32 node (Modbus RTU slave + UART2 configuration commands). No HAL/STM32 dependency, only pyserial.

## Requirements

```bash
pip3 install pyserial
```

- USB-RS485: 9600 **8E1** (even parity)
- UART2 config: 115200 8N1, adapter TX → node RX (the node is RX-only; read results on the LCD or over Modbus)

## Scripts

| File | Purpose |
|------|---------|
| `modbus_full_test.py` | FC03/04/06 test matrix: CRC, broadcast, bounds, slow feed, soak |
| `uart2_config_test.py` | Sends `id=`/`addr=`/`thr=`/`wire=`/`qty=` in one burst, verifies over Modbus |

```bash
python3 tools/modbus_full_test.py --port /dev/cu.usbserial-XXXX --addr 1
python3 tools/uart2_config_test.py --uart2 /dev/cu.usbmodemXXXX --rs485 /dev/cu.usbserial-XXXX --addr 1
```

FrameGuard register map (1-based): see [docs/rs485-modbus-protocol.md](../docs/rs485-modbus-protocol.md).
