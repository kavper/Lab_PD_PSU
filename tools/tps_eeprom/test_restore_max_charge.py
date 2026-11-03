from pathlib import Path
import struct,sys,unittest
sys.path.insert(0,str(Path(__file__).parent))
import restore_max_charge as m
root=Path(__file__).resolve().parents[2]/'config/tps25751/MAX_CHARGE'

def delta(a,b):
    crc=0
    for x,y in zip(a,b):
        crc^=x^y
        for _ in range(8):crc=(crc>>1)^(0xEDB88320 if crc&1 else 0)
    return int(f'{crc:032b}'[::-1],2)

class MaxChargeTests(unittest.TestCase):
    def test_both_artifacts_and_crc(self):
        for name in ('SOURCE_5V_3A','SOURCE_5V_5A_BENCH_ONLY'):
            d=root/name;before=(d/'PREVIOUS_512mA.bin').read_bytes();after=m.patch(before)
            self.assertEqual(after,(d/'candidate.bin').read_bytes())
            for base in (0x800,0x4400):
                h=base+0x80;start=h+16;end=h+0x32E
                expected=struct.unpack_from('<I',before,h+4)[0]^delta(before[start:end],after[start:end])
                self.assertEqual(struct.unpack_from('<I',after,h+4)[0],expected)
                for off in (0x315,0x356):self.assertEqual(struct.unpack_from('<H',after,base+off)[0],18000)
                for off in (0x31B,0x35C):self.assertEqual(struct.unpack_from('<H',after,base+off)[0],0xFC0)
                self.assertEqual(before[base+0xF9:base+0x1A0],after[base+0xF9:base+0x1A0])
    def test_unknown_image_rejected(self):
        b=bytearray((root/'SOURCE_5V_3A/PREVIOUS_512mA.bin').read_bytes());b[0x1000]^=1
        with self.assertRaises(ValueError):m.patch(b)

if __name__=='__main__':unittest.main()
