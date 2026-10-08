#!/usr/bin/env python3
"""
Test the UART2 configuration commands (id= / addr= / thr= ...), verified over Modbus.
UART2: 115200 8N1, RX-only on the node → send a command, read the register back to check.
RS485: 9600 8E1.

Usage: python3 tools/uart2_config_test.py --uart2 /dev/cu.usbmodemXXXX --rs485 /dev/cu.usbserial-XXXX [--addr 1]
"""
import argparse
import struct
import sys
import time
import serial

REG_ID, REG_THR, REG_WIRE, REG_QTY = 5, 6, 16, 17
CH_DISABLED = 5


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


class Modbus:
    def __init__(self, port):
        self.ser = serial.Serial(port, 9600, parity=serial.PARITY_EVEN, timeout=0.5)
        time.sleep(0.05)

    def read(self, slave, addr, count=1):
        pdu = struct.pack('>BBHH', slave, 4, addr, count)
        self.ser.reset_input_buffer()
        self.ser.write(pdu + struct.pack('<H', crc16(pdu)))
        self.ser.flush()
        time.sleep(0.4)
        r = self.ser.read(256)
        if len(r) < 5 or r[0] != slave or r[1] & 0x80:
            return None
        if crc16(r[:-2]) != struct.unpack('<H', r[-2:])[0]:
            return None
        return [struct.unpack('>H', r[3 + 2 * i:5 + 2 * i])[0] for i in range(r[2] // 2)]

    def close(self):
        self.ser.close()


class Uart2:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.3)
        time.sleep(0.1)

    def cmd(self, text, settle=1.2):
        """Send the whole line in one burst (like paste/script) — no slow trickle."""
        self.ser.write((text + '\r\n').encode())
        self.ser.flush()
        time.sleep(settle)          # config_save (flash erase) + 3 s LCD message run in the background

    def close(self):
        self.ser.close()


def run(uart2_port, rs485_port, slave):
    print(f"UART2 Config Test — uart2={uart2_port} rs485={rs485_port} slave={slave}")
    print("=" * 60)
    mb, u2 = Modbus(rs485_port), Uart2(uart2_port)
    passed = failed = 0

    def check(name, fn):
        nonlocal passed, failed
        sys.stdout.write(f"  {name} ... ")
        sys.stdout.flush()
        try:
            fn()
            passed += 1
            print("PASS")
        except AssertionError as e:
            failed += 1
            print(f"FAIL: {e}")

    def reg(addr, s=None):
        v = mb.read(slave if s is None else s, addr, 1)
        assert v is not None, f"Modbus did not respond (slave {slave if s is None else s}, reg {addr})"
        return v[0]

    orig_id, orig_thr, orig_wire, orig_qty = reg(REG_ID), reg(REG_THR), reg(REG_WIRE), reg(REG_QTY)
    print(f"  Current: ID={orig_id} THR={orig_thr} WIRE={orig_wire} QTY={orig_qty} addr={slave}")

    # ---- id= ----
    def t_id_burst():
        u2.cmd("id=2222")
        assert reg(REG_ID) == 2222, f"ID={reg(REG_ID)}"
    check("id=2222 sent in one burst → reg5=2222", t_id_burst)

    def t_id_min_max():
        u2.cmd("id=1");    assert reg(REG_ID) == 1, f"ID={reg(REG_ID)}"
        u2.cmd("id=9999"); assert reg(REG_ID) == 9999, f"ID={reg(REG_ID)}"
    check("id=1 / id=9999 (limits)", t_id_min_max)

    def t_id_reject():
        for bad in ("id=0", "id=10000", "id=abc", "id=", "ID=12x"):
            u2.cmd(bad, settle=0.6)
            assert reg(REG_ID) == 9999, f"{bad!r} accepted: ID={reg(REG_ID)}"
    check("id=0 / 10000 / abc / empty / 12x → rejected", t_id_reject)

    def t_id_case():
        u2.cmd("ID=1234")
        assert reg(REG_ID) == 1234, f"ID={reg(REG_ID)}"
    check("ID=1234 (case-insensitive)", t_id_case)

    def t_id_lf_only():
        u2.ser.write(b"id=4321\n"); u2.ser.flush(); time.sleep(1.2)
        assert reg(REG_ID) == 4321, f"ID={reg(REG_ID)}"
    check("id=4321 terminated by \\n only", t_id_lf_only)

    def t_id_paste_two_lines():
        u2.ser.write(b"id=1111\r\nid=1212\r\n"); u2.ser.flush(); time.sleep(2.0)
        assert reg(REG_ID) == 1212, f"ID={reg(REG_ID)}"
    check("2 lines back-to-back in one burst → last command wins", t_id_paste_two_lines)

    # ---- thr= ----
    def t_thr():
        u2.cmd("thr=3000"); assert reg(REG_THR) == 3000, f"THR={reg(REG_THR)}"
        u2.cmd("thr=100");  assert reg(REG_THR) == 100,  f"THR={reg(REG_THR)}"
        u2.cmd("thr=60000"); assert reg(REG_THR) == 60000, f"THR={reg(REG_THR)}"
    check("thr=3000 / 100 / 60000 → reg6", t_thr)

    def t_thr_reject():
        for bad in ("thr=99", "thr=60001", "thr=x"):
            u2.cmd(bad, settle=0.6)
            assert reg(REG_THR) == 60000, f"{bad!r} accepted: THR={reg(REG_THR)}"
    check("thr=99 / 60001 / x → rejected", t_thr_reject)

    # ---- wire= ----
    def t_wire():
        u2.cmd("wire=600");  assert reg(REG_WIRE) == 600,  f"WIRE={reg(REG_WIRE)}"
        u2.cmd("wire=100");  assert reg(REG_WIRE) == 100,  f"WIRE={reg(REG_WIRE)}"
        u2.cmd("wire=3000"); assert reg(REG_WIRE) == 3000, f"WIRE={reg(REG_WIRE)}"
    check("wire=600 / 100 / 3000 → reg16", t_wire)

    def t_wire_reject():
        for bad in ("wire=99", "wire=3001", "wire=x"):
            u2.cmd(bad, settle=0.6)
            assert reg(REG_WIRE) == 3000, f"{bad!r} accepted: WIRE={reg(REG_WIRE)}"
    check("wire=99 / 3001 / x → rejected", t_wire_reject)

    # ---- qty= ----
    def all_regs():
        v = mb.read(slave, 1, 15)
        assert v is not None, "Modbus did not respond (addr 1 qty 15)"
        return v  # [vac1..4, id, thr, hb, st1..4, wire1..4]

    def t_qty():
        for q in (1, 2, 3, 4):
            u2.cmd(f"qty={q}", settle=2.5)     # wait ≥1 scan so disabled channels get their data cleared
            assert reg(REG_QTY) == q, f"QTY={reg(REG_QTY)}"
            r = all_regs()
            for i in range(4):
                st, vac, wire = r[7 + i], r[i], r[11 + i]
                if i < q:
                    assert st != CH_DISABLED, f"qty={q}: P{i+1} disabled (st={st})"
                else:
                    assert st == CH_DISABLED, f"qty={q}: P{i+1} st={st} ≠ 5"
                    assert vac == 0 and wire == 0, f"qty={q}: P{i+1} vac={vac} wire={wire} ≠ 0"
    check("qty=1..4 → reg17, channels > qty: st=5, vac=0, wire=0", t_qty)

    def t_qty_reject():
        for bad in ("qty=0", "qty=5", "qty=x"):
            u2.cmd(bad, settle=0.6)
            assert reg(REG_QTY) == 4, f"{bad!r} accepted: QTY={reg(REG_QTY)}"
    check("qty=0 / 5 / x → rejected", t_qty_reject)

    # ---- addr= ----
    other = 5 if slave != 5 else 6

    def t_addr():
        u2.cmd(f"addr={other}")
        assert mb.read(other, REG_ID, 1) is not None, f"slave {other} not responding after addr="
        assert mb.read(slave, REG_ID, 1) is None, f"old slave {slave} still responding"
        u2.cmd(f"addr={slave}")
        assert mb.read(slave, REG_ID, 1) is not None, f"could not return to addr {slave}"
    check(f"addr={other} → only slave {other} responds, addr={slave} → back again", t_addr)

    def t_addr_reject():
        for bad in ("addr=0", "addr=248", "addr=1a"):
            u2.cmd(bad, settle=0.6)
            assert mb.read(slave, REG_ID, 1) is not None, f"{bad!r} lost slave {slave}"
    check("addr=0 / 248 / 1a → rejected", t_addr_reject)

    def t_unknown():
        u2.cmd("show", settle=0.6)
        u2.cmd("xyz=1", settle=0.6)
        assert reg(REG_ID) == 1212 and reg(REG_THR) == 60000 and reg(REG_WIRE) == 3000, \
            "unknown command changed the config"
    check("show / xyz=1 → ignored, config unchanged", t_unknown)

    # ---- restore ----
    u2.cmd(f"id={orig_id}")
    u2.cmd(f"thr={orig_thr}")
    u2.cmd(f"wire={orig_wire}")
    u2.cmd(f"qty={orig_qty}")
    ok = (reg(REG_ID) == orig_id and reg(REG_THR) == orig_thr
          and reg(REG_WIRE) == orig_wire and reg(REG_QTY) == orig_qty)
    print(f"  Restore ID={orig_id} THR={orig_thr} WIRE={orig_wire} QTY={orig_qty}: {'OK' if ok else 'FAIL'}")
    if not ok:
        failed += 1

    print("=" * 60)
    print(f"  PASS: {passed}  FAIL: {failed}")
    print("=" * 60)
    mb.close(); u2.close()
    return failed == 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--uart2', required=True, help='USB-TTL port wired to PA3 (115200 8N1)')
    ap.add_argument('--rs485', required=True, help='USB-RS485 port (9600 8E1)')
    ap.add_argument('--addr', type=int, default=1)
    a = ap.parse_args()
    sys.exit(0 if run(a.uart2, a.rs485, a.addr) else 1)
