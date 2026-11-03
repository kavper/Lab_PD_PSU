import importlib.util
from pathlib import Path
import struct, sys, unittest
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(Path(__file__).parent))
import restore_pps_12v as m
folder=root/'config/tps25751/pps_12v_17v5_512ma'

def delta(a,b):
    crc=0
    for x,y in zip(a,b):
        crc^=x^y
        for _ in range(8):crc=(crc>>1)^(0xEDB88320 if crc&1 else 0)
    return int(f'{crc:032b}'[::-1],2)

class RestoreTests(unittest.TestCase):
    def setUp(self):
        self.old=(folder/'PREVIOUS_without_PPS_12V.bin').read_bytes()
        self.new=m.patch(self.old)
    def test_artifact_and_profile_order(self):
        self.assertEqual(self.new,(folder/'tps_PPS_12V_17496mV_512mA_CANDIDATE.bin').read_bytes())
        expected=(0x2401912C,0x2D12C,0x3C12C,0x4B12C,0x641F4,0xE004B1F4,0xC1A43264)
        for base in (0x800,0x4400):
            self.assertEqual(self.new[base+0xF9],7)
            self.assertEqual(struct.unpack_from('<7I',self.new,base+0xFC),expected)
    def test_crc_and_unrelated_configuration_preserved(self):
        self.assertEqual(len(self.new),len(self.old))
        allowed=set()
        for base in (0x800,0x4400):
            h=base+0x80;start=h+16;end=h+0x32E
            expected=struct.unpack_from('<I',self.old,h+4)[0]^delta(self.old[start:end],self.new[start:end])
            self.assertEqual(struct.unpack_from('<I',self.new,h+4)[0],expected)
            allowed.update(range(h+4,h+8));allowed.add(base+0xF9);allowed.update(range(base+0xFC,base+0x118))
            for off in (0x315,0x356):self.assertEqual(struct.unpack_from('<H',self.new,base+off)[0],17496)
            for off in (0x31B,0x35C):self.assertEqual(struct.unpack_from('<H',self.new,base+off)[0],0x100)
        self.assertTrue(all(i in allowed for i,(a,b) in enumerate(zip(self.old,self.new)) if a!=b))
    def test_corrupt_input_refused(self):
        corrupted=bytearray(self.old);corrupted[0x1000]^=1
        with self.assertRaises(ValueError):m.patch(corrupted)

if __name__=='__main__':unittest.main()
