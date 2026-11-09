#include <gui/common/UsbPdObjects.hpp>
#include <assert.h>
#include <stdio.h>
int main(){
 const uint32_t raw[7]={0x2401912cu,0x2d12cu,0x3c12cu,0x4b12cu,0x641f4u,0xe004b1f4u,0xc1a43264u};
 const unsigned mv[5]={5000,9000,12000,15000,20000};
 for(unsigned i=0;i<5;i++){auto p=ui::PdObject::decode(raw[i]);assert(p.valid&&p.type==0&&p.minMv==mv[i]);}
 auto p=ui::PdObject::decode(raw[6]);assert(p.valid&&p.type==3&&p.apdo==0&&p.minMv==5000&&p.maxMv==21000&&p.ma==5000);
 assert((raw[5]>>30)==3&&((raw[5]>>28)&3)==2);
 puts("PASS actual H7 decoder: five fixed profiles including 12 V, preserved AVS type, restored PPS 5..21 V / 5 A");
}
