"""Offline sink selection update; optional 5 V / 5 A SOURCE bench image."""
import argparse
import hashlib
import struct
from pathlib import Path
from patch_18v import config_crc_delta

SOURCE_SHA256 = '14ebbd5ccf056ab13711c4c89723530279deab493365a9b88177bbfb908717b6'
OLD_SINK = (0x1001912C, 0x0002D12C, 0x0004B12C, 0x000641F4)
NEW_SINK = (*OLD_SINK[:2], 0x0003C12C, *OLD_SINK[2:])


def patch(source, source_5a=False):
    if hashlib.sha256(source).hexdigest() != SOURCE_SHA256:
        raise ValueError('Unsupported source image')
    image = bytearray(source)
    allowed = set()
    for base in (0x800, 0x4400):
        header = base+0x80
        start, end = header+16, header+0x32E
        sink = base+0x13C
        assert source[sink-3:sink] == bytes.fromhex('33 00 34')
        assert source[sink] == 4
        assert struct.unpack_from('<7I', source, sink+1) == (*OLD_SINK, 0, 0, 0)
        image[sink] = 5
        struct.pack_into('<5I', image, sink+1, *NEW_SINK)
        auto = base+0x175
        assert source[auto-3:auto] == bytes.fromhex('37 00 17')
        assert source[auto] == 0x3B
        # Only tie-breaker changes: maximum usable power remains primary.
        image[auto] &= ~1
        allowed.update(range(header+4, header+8))
        allowed.update(range(sink, sink+1+5*4))
        allowed.add(auto)
        if source_5a:
            pdo1 = base+0xFC
            original = struct.unpack_from('<I', source, pdo1)[0]
            assert original == 0x2401912C
            # Existing PDO1 path selection = 2 (high-voltage path), not PP5V.
            assert (source[base+0xFA]&3) == 2
            struct.pack_into('<I', image, pdo1, (original & ~0x3FF) | 500)
            allowed.update(range(pdo1, pdo1+4))
        old_crc = struct.unpack_from('<I', source, header+4)[0]
        struct.pack_into('<I', image, header+4, old_crc ^
                         config_crc_delta(source[start:end], image[start:end]))
    assert len(source) == len(image)
    assert all(i in allowed for i, (a,b) in enumerate(zip(source,image)) if a != b)
    return bytes(image)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--source-5a', action='store_true', help='bench variant, not load validated')
    args = parser.parse_args()
    image = patch(args.source.read_bytes(), args.source_5a)
    args.output.write_bytes(image)
    print(hashlib.sha256(image).hexdigest())


if __name__ == '__main__':
    main()
