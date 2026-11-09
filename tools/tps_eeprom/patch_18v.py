"""Fixed-length patch for the archived F8091164 100 W image; no device writes.
CRC delta uses the reflected IEEE polynomial and reversed 32-bit output.
This is NOT a general image encoder. Hardware FLvy/boot/readback is required.
"""
from pathlib import Path
import hashlib, json, re, struct, zipfile, zlib


def reverse32(n):
    return int(f'{n:032b}'[::-1], 2)


def config_crc_delta(old, new):
    assert len(old) == len(new)
    return reverse32(zlib.crc32(old) ^ zlib.crc32(new))


def patch(source):
    expected = "329b451b1c8ee54cbb40729576b12165f960795902989c2d2a7090cff9e9dfc0"
    if hashlib.sha256(source).hexdigest() != expected:
        raise ValueError("Unsupported source image: SHA256 mismatch")
    b = bytearray(source)
    assert len(b) == 31680
    for base in (0x800, 0x4400):
        assert struct.unpack_from('<I', b, base)[0] == 0xACE00001
        h = base + 0x80
        assert struct.unpack_from('<H', b, h + 2)[0] == 0x32e
        start, end = h + 16, h + 0x32e
        before = bytes(b[start:end])
        # Two existing BQ25731 ChargeVoltage write payloads: reg 04, 4078h.
        # 4078h = 16504 mV; 4650h = 18000 mV (8 mV resolution).
        for offset in (0x315, 0x356):
            p = base + offset
            assert b[p-1:p+2] == bytes.fromhex('04 78 40')
            b[p:p+2] = struct.pack('<H', 18000)
        crc = struct.unpack_from('<I', b, h+4)[0]
        struct.pack_into('<I', b, h+4,
                         crc ^ config_crc_delta(before, bytes(b[start:end])))
    return bytes(b)


def main():
    root = Path(__file__).resolve().parents[2]
    archive = root.parent/'esp32_tps25751_memorybin_writer/full_working_max_100W/config_TPS25751_F8091164_fullFlash.zip'
    with zipfile.ZipFile(archive) as z:
        source = z.read('config_TPS25751_F8091164_fullFlash.bin')
    # Independently reproduce CRC of the second TI-generated local image.
    other = (root.parent/'esp32_tps25751_memorybin_writer/full.h').read_text()
    other = bytes(int(x,16) for x in re.findall(r'0x([0-9a-fA-F]{2})\b', other.split('{',1)[1].rsplit('}',1)[0]))
    for base in (0x800,0x4400):
        h=base+0x80; start=h+16; end=h+0x32e
        predicted=struct.unpack_from('<I',source,h+4)[0]^config_crc_delta(source[start:end],other[start:end])
        assert predicted == struct.unpack_from('<I',other,h+4)[0], 'Reference CRC cross-check failed'
    result=patch(source)
    allowed={base+o for base in (0x800,0x4400) for o in (0x84,0x85,0x86,0x87,0x315,0x316,0x356,0x357)}
    changed=[i for i,(a,b) in enumerate(zip(source,result)) if a!=b]
    assert set(changed)<=allowed
    out=root/'output/tps_18v_candidate';out.mkdir(parents=True,exist_ok=True)
    (out/'tps25751_18v_UNVERIFIED.bin').write_bytes(result)
    lines=[', '.join(f'0x{x:02x}' for x in result[i:i+16])+',' for i in range(0,len(result),16)]
    (out/'full_18v_UNVERIFIED.h').write_text('#pragma once\n#include <Arduino.h>\nconst unsigned char tps25750x_fullFlash_i2c_array[] = {\n'+'\n'.join(lines)+'\n};\n')
    report={'status':'bench validation required; precharge unchanged','source_sha256':hashlib.sha256(source).hexdigest(),'output_sha256':hashlib.sha256(result).hexdigest(),'changed_offsets':[hex(i) for i in changed],'reference_crc_crosscheck':'PASS (both regions of second TI-generated image)','charge_voltage_mv':18000}
    (out/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
