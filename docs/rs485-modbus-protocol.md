# RS-485 Modbus RTU Protocol (FrameGuard node)

Single source of truth for the node firmware, the test scripts and the Modbus master. The map is **1-based**: addr 1 = VAC P1.

## Serial

| | |
|---|---|
| Baud rate | 9600 bps |
| Framing | **8E1** (even parity) |
| Slave address | 1–247, default 1; UART2 command `addr=` |
| Broadcast 0 | Strictly silent |
| Functions | **FC03** read; FC04 alias with the same data (optional); FC06 write single register (addr 6, 17) |
| Request | Exactly **8 bytes**. Frame ≠ 8 bytes → silent |

## Register map

`reg_count = 18`. Standard read: **FC03 addr 1 qty 17** (registers 1–17, including wire threshold + channel count).

| Addr | Name | Unit | Notes |
|------|------|------|-------|
| 0 | reserved | — | reads 0 |
| 1–4 | VAC P1–P4 | 0.01 V | I2C error → `0xFFFF`; valid values clamp at `0xFFFE`. Disabled channel (`qty=`) → 0 |
| 5 | DEVICE_ID | — | 1–9999, device identifier |
| 6 | ALERT_THRESHOLD | mV | Active value in RAM. **Writable via FC06** (100–60000), not saved to flash |
| 7 | FW_HEARTBEAT | — | +1 per scan loop |
| 8–11 | CH_STATUS P1–P4 | enum | 0 INIT, 1 OK, 2 LEAK, 3 WIRE_NG, 4 ERR, **5 DISABLED** |
| 12–15 | WIRE_MVPP P1–P4 | — | wire-check reading (diagnostics). Disabled channel → 0 |
| 16 | WIRE_THRESHOLD | — | read-only; set with `wire=` (100–3000) |
| 17 | CH_COUNT | — | **Writable via FC06** (1–4), saved to flash — same as `qty=`. Channel i ≥ CH_COUNT: not measured, LED off, LCD blank, st=5 |

VAC < 0 ⇔ CH_STATUS = 4. VAC×1000 > ALERT_THRESHOLD ⇔ CH_STATUS = 2 (the same `is_leak()` used by LEDs/LCD).

FC06 to register 6 only changes RAM (a restart returns to the flash value). The persistent value is the UART2 `thr=` command (flash); a master that wants its own threshold should compare register 6 on every poll and re-write it with FC06 when it differs.

## Exceptions / silence

| Situation | Node |
|---|---|
| Unknown FC | exception 0x01 |
| addr/qty out of range, qty=0, FC06 addr ∉ {6, 17} | exception 0x02 |
| FC06 value out of range (reg 6: 100–60000; reg 17: 1–4) | exception 0x03 |
| Wrong address, bad CRC, frame ≠ 8 bytes, PE/FE/NE | silent |
| Request received > 150 ms ago and not yet answered | dropped silently |

## Timing

- One 8E1 character = 11 bits → ≈1.15 ms/byte. An FC03 17-register response = 39 bytes ≈ 45 ms.
- Frames complete **by length** (8 bytes). A gap ≥ 5 ms in the ISR marks a new frame boundary (USB-RS485 adapters often insert 1–16 ms gaps).
- Background TX DMA, hardware DE (DEM).
- Poll ≥ 1 s per node. USB-RS485 tool timeout: **500 ms** (measured Python+FTDI ~300 ms). Direct-UART master: 200 ms is enough unless the LCD stalls; 2 retries.
- The UART2 `config_save` flash erase takes ~22–40 ms: at most 1 request is lost, covered by master retries.

## UART2 — technician commands

USART2 **RX-only** on PA3, 115200 8N1. Feedback on the LCD for 3 s (inverted). Saved to flash immediately (magic in the last double-word). RX is interrupt-driven into a 32-byte ring, so a whole line can be sent in one burst (paste/script).

| Command | Range | Meaning |
|---------|-------|---------|
| `id=` | 1–9999 | Device identifier (DEVICE_ID) |
| `addr=` | 1–247 | Modbus address, effective immediately |
| `thr=` | 100–60000 mV | Leakage threshold (flash + RAM) |
| `wire=` | 100–3000 | Wire-break detection threshold, shared by 4 channels |
| `qty=` | 1–4 | Number of channels in the machine group |
| `v=` | 500–24000 mV | Calibrate K on all enabled channels: K = V_ref / VAC_untrimmed, K ∈ 0.5..2.0 |
| `k1=`…`k4=` | 500–24000 mV | Calibrate K on one channel, same guard |
| `config` | (no argument) | Settings page on the LCD for 5 s |

There is no `show` command (no TX). Measured VAC = ADS calibration table × K[i], K defaults to 1.0.

Flashing firmware with config layout v4 (`CFG_MAGIC 0xAC05C1D4`) **erases older configs** — the technician must set them again.

## Example frames (slave 1)

```
FC03 read 17 regs:   01 03 00 01 00 11 D4 06
FC06 threshold 3000: 01 06 00 06 0B B8 6E 89   → echoed on success
FC06 channels = 2:   01 06 00 11 00 02 58 0E   → echoed on success, saved to flash
qty=125:             01 04 00 01 00 7D …       → exception 0x02
```

## Test scripts

See [tools/README.md](../tools/README.md).

- `tools/modbus_full_test.py` — 27 PASS / 0 FAIL on a real bus (1000 consecutive polls, 0 WWDG resets).
- `tools/uart2_config_test.py` — id/addr/thr/wire/qty PASS. Calibration `v=`/`kN=` needs an AC source and is not covered by the script.

## Threat model

The RS-485 bus is unauthenticated. Defences: strict CRC/length/range checks, no broadcast, FC06 only changes RAM; the master should check DEVICE_ID against an expected addr→id table.
