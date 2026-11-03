"""Restore the earlier TPS charging ceiling: 17.496 V / 8.064 A at 5 mOhm.

Does not change SOURCE/SINK capabilities or any BMS protection. Offline only.
"""
import argparse
import hashlib
import struct
from pathlib import Path
from patch_18v import config_crc_delta

SUPPORTED = {
    '086dc309ff9c60b24fcaf7e357cfc9e5ab2b925a8f1f2c3a34de7ad8ab968ff2',
    '634d60e364cbe877e33af0defe91f17b2c541d18d15f831c9a47cb90ff11d56e',
}


def patch(source):
    if hashlib.sha256(source).hexdigest() not in SUPPORTED:
        raise ValueError('Unsupported source image')
    result = bytearray(source)
    allowed = set()
    for base in (0x800, 0x4400):
        h = base+0x80
        start, end = h+16, h+0x32E
        for off in (0x315, 0x356):
            assert source[base+off-1:base+off+2] == bytes.fromhex('04 58 44')
            struct.pack_into('<H', result, base+off, 17496)
            allowed.update((base+off, base+off+1))
        for off in (0x31B, 0x35C):
            assert source[base+off-1:base+off+2] == bytes.fromhex('02 00 01')
            struct.pack_into('<H', result, base+off, 0x0FC0)
            allowed.update((base+off, base+off+1))
        original = struct.unpack_from('<I', source, h+4)[0]
        struct.pack_into('<I', result, h+4, original ^
                         config_crc_delta(source[start:end], result[start:end]))
        allowed.update(range(h+4, h+8))
    assert len(result) == len(source)
    assert all(i in allowed for i,(a,b) in enumerate(zip(source,result)) if a != b)
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
