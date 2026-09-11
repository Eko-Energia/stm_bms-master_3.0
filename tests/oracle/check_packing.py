#!/usr/bin/env python3
"""Decode vectors.txt with cantools and assert against independently
computed expected values.

Python holds its own expectations here, derived directly from the known test
fixture inputs (ADC counts, JK response fields, thermistor raw counts) in
test_pack_vectors.c - never from anything the C side computed or printed. If
test_pack_vectors.c emitted the expected values itself, a bug in our C
packing would produce a matching wrong expectation and this script would
agree with itself. cantools decodes the DBC independently of our packing
code, so a scaling, offset, or byte-order bug here produces a genuine
mismatch even though the frame still decodes without error.
"""
import math
import pathlib
import sys

import cantools

HERE = pathlib.Path(__file__).resolve().parent
DBC = HERE.parents[1] / "docs" / "CAN-DATABASE" / "CAN_DB.dbc"

THERM_SCALE = 0.39216  # BMSMaster_PCB<m>Therm<t>Temp: raw count -> degC


def _module_of(frame_id: int) -> int:
    return (frame_id // 10) - 20


def _offset_of(frame_id: int) -> int:
    return frame_id % 10


def _therm_of(frame_id: int) -> int:
    m = _module_of(frame_id)
    o = _offset_of(frame_id)
    return o if (m & 1) else (10 - o)


def _therm_expected(therm_index: int) -> dict:
    """Expected decoded values for BMSMaster_PCBsTherm<therm_index>Temp,
    from the fixture's THERM_OnFrame(id, 60 + id % 7) for id in 211..279
    (module/thermistor id mapping per docs/pcb.md; raw count 60..66)."""
    raw = {}
    for frame_id in range(211, 280):
        if frame_id % 10 == 0:
            continue
        raw[(_module_of(frame_id), _therm_of(frame_id))] = 60 + (frame_id % 7)
    result = {
        f"BMSMaster_PCB{m}Therm{therm_index}Temp": raw[(m, therm_index)] * THERM_SCALE
        for m in range(1, 8)
    }
    if therm_index == 9:
        # docs/CAN-DATABASE/CAN_DB.dbc has a typo for this one signal:
        # "BMSMaster_PCB13herm9Temp" instead of "BMSMaster_PCB3Therm9Temp".
        # Not our bug to fix here - match the DBC as it stands.
        result["BMSMaster_PCB13herm9Temp"] = result.pop("BMSMaster_PCB3Therm9Temp")
    return result


def _cell_mv(first: int, last: int) -> dict:
    """Expected mV for cells [first, last], from the fixture's
    3300 + (cell - 1) * 7 mV per tap."""
    return {
        f"BMSMaster_JK_Cell{c}_mV": 3300 + ((c - 1) * 7)
        for c in range(first, last + 1)
    }


EXPECTED = {
    0x080: {                                    # BMSMaster_NODE: active fault reported
        "Error_Code": 10,                       # BMS_ERR_FATAL_INIT
        "Error_Specific_Data": 0x1234,
    },
    0x082: {                                    # BMSMaster_MasterVoltCurrTemp
        "BMSMaster_MasterBatteryVoltage": 73.1,
        "BMSMaster_MasterBatteryCurrent": 25.0, # positive: ADC's own signed spot-check
        "BMSMaste_MasterBatteryTemperatur": 37.0,
    },
    0x08C: {                                    # BMSMaster_JK_Pack
        "BMSMaster_JK_PackVoltage": 72.56,
        "BMSMaster_JK_PackCurrent": -20.0,      # negative: JK's own signed spot-check
        "BMSMaster_JK_SOC": 77,
        "BMSMaster_JK_SOH": 88,
        "BMSMaster_JK_StatusFlags": 1,
        "BMSMaster_JK_ModeFlags": 0x0B,  # 0x8c raw 0xab, reserved bits 4..15 dropped
    },
    0x08D: _cell_mv(1, 4),
    0x08E: _cell_mv(5, 8),
    0x08F: _cell_mv(9, 12),
    0x090: _cell_mv(13, 16),
    0x091: _cell_mv(17, 20),
    0x092: _cell_mv(21, 21),
    0x093: {                                    # BMSMaster_JK_Temp
        "BMSMaster_JK_MosTemp": 45,
        "BMSMaster_JK_BalTemp": -5,
    },
    0x094: {                                    # BMSMaster_JK_CycleStats
        "BMSMaster_JK_Cycles": 1234,
        "BMSMaster_JK_CellCount": 21,
    },
    0x09F: {},                                  # BMSMaster_END: no signals defined
}
for _t in range(1, 10):
    EXPECTED[0x82 + _t] = _therm_expected(_t)


def _mismatch(frame_id: int, name: str, expected, actual) -> bool:
    if isinstance(expected, float):
        if not math.isclose(actual, expected, rel_tol=1e-6, abs_tol=1e-6):
            print(f"FAIL 0x{frame_id:03X} {name}: expected {expected}, got {actual}")
            return True
        return False
    if actual != expected:
        print(f"FAIL 0x{frame_id:03X} {name}: expected {expected}, got {actual}")
        return True
    return False


def main() -> int:
    db = cantools.database.load_file(DBC)
    vectors = (HERE / "vectors.txt").read_text().split("\n")
    failures = 0
    frames = 0
    checked_ids = set()

    for line in vectors:
        line = line.strip()
        if not line:
            continue
        frame_id_hex, payload_hex = line.split()
        frame_id = int(frame_id_hex, 16)
        payload = bytes.fromhex(payload_hex)
        frames += 1
        try:
            message = db.get_message_by_frame_id(frame_id)
            decoded = message.decode(payload[: message.length])
        except Exception as exc:                      # noqa: BLE001
            print(f"FAIL 0x{frame_id:03X}: {exc}")
            failures += 1
            continue

        if frame_id not in EXPECTED:
            print(f"FAIL 0x{frame_id:03X} {message.name}: no expected values on file")
            failures += 1
            continue
        checked_ids.add(frame_id)

        expected = EXPECTED[frame_id]
        mismatched = False
        for name, exp_value in expected.items():
            if name not in decoded:
                print(f"FAIL 0x{frame_id:03X} {name}: signal missing from decode")
                failures += 1
                mismatched = True
                continue
            if _mismatch(frame_id, name, exp_value, decoded[name]):
                failures += 1
                mismatched = True
        if not mismatched:
            print(f"ok   0x{frame_id:03X} {message.name}")

    missing = set(EXPECTED) - checked_ids
    for frame_id in sorted(missing):
        print(f"FAIL 0x{frame_id:03X}: expected in EXPECTED but not present in vectors.txt")
        failures += 1

    print(f"\n{frames} frames, {failures} failed")
    return 1 if failures else 0

if __name__ == "__main__":
    sys.exit(main())
