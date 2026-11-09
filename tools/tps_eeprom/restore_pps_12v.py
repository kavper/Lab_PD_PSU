"""Restore archived PPS and add 12 V to the specific balancing image.

Offline only. Fixed image length, register length and all charger writes stay
unchanged. Both configuration CRCs are updated using the validated delta.
"""
import argparse
import hashlib
import struct
from pathlib import Path

from patch_18v import config_crc_delta

SOURCE_SHA256 = 'b2f0acbe27c6a6883bec0fd6b1498627077b19c3cc4a71636c84baf9c49af5d3'
BASES = (0x800, 0x4400)
CAPS_OFFSET = 0xF9
PPS_RAW = 0xC1A43264  # archived TI-generated image: 5..21 V, 5 A
FIXED_12V_RAW = (12000 // 50 << 10) | (3000 // 10)
OLD_PDOS = (0x2401912C, 0x0002D12C, 0x0004B12C,
            0x000641F4, 0xE004B1F4)
NEW_PDOS = (*OLD_PDOS[:2], FIXED_12V_RAW, *OLD_PDOS[2:], PPS_RAW)


def patch(source):
    if hashlib.sha256(source).hexdigest() != SOURCE_SHA256:
        raise ValueError('Unsupported input image; original must remain unchanged')
    result = bytearray(source)
    for base in BASES:
        header = base + 0x80
        assert struct.unpack_from('<H', source, header + 2)[0] == 0x32E
        begin, end = header + 16, header + 0x32E
        caps = base + CAPS_OFFSET
        # Command record prefix: register 32h, fixed payload length 62 bytes.
        assert source[caps-3:caps] == bytes.fromhex('32 00 3e')
        assert source[caps] == 5
        assert struct.unpack_from('<7I', source, caps + 3) == (*OLD_PDOS, 0, 0)
        assert all(((int.from_bytes(source[caps+1:caps+3], 'little') >> (2*i)) & 3) == 2
                   for i in range(7))  # all seven slots already use same path
        # Existing BQ current/voltage operands must remain the balancing profile.
        for off in (0x315, 0x356):
            assert struct.unpack_from('<H', source, base + off)[0] == 17496
        for off in (0x31B, 0x35C):
            assert struct.unpack_from('<H', source, base + off)[0] == 0x100
        result[caps] = 7
        struct.pack_into('<7I', result, caps + 3, *NEW_PDOS)
        old_crc = struct.unpack_from('<I', source, header+4)[0]
        struct.pack_into('<I', result, header+4, old_crc ^
                         config_crc_delta(source[begin:end], result[begin:end]))
    allowed = {base + offset for base in BASES
               for offset in (*range(0x84, 0x88), CAPS_OFFSET,
                              *range(CAPS_OFFSET+3, CAPS_OFFSET+31))}
    assert len(result) == len(source)
    assert all(i in allowed for i, (a,b) in enumerate(zip(source,result)) if a != b)
    return bytes(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.write_bytes(patch(args.source.read_bytes()))
    print(hashlib.sha256(args.output.read_bytes()).hexdigest())


if __name__ == '__main__':
    main()
