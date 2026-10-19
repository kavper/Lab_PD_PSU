"""Exercise the production read-only PD export and G4 -> H7 wire contract.

Run: python tests/test_pd_capabilities.py path/to/H7
CC may name a native C compiler. No hardware or policy writes are simulated.
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
h7 = Path(sys.argv[1]).resolve()
source = (root / 'Core/Src/power_manager.c').read_text()
getter = source[source.index('void PowerManager_GetPdCapabilities('):]
header = (root / 'Core/Inc/power_manager.h').read_text()
end = header.index('} PowerManager_PdCapabilities_t;') + len('} PowerManager_PdCapabilities_t;')
start = header.rfind('typedef struct', 0, end)
caps_type = header[start:end]
host = (root / 'Core/Src/host_link.c').read_text()
start = host.index('    caps_payload[0] = 1U;')
end = host.index('    {\n        uint8_t aux_payload', start)
packing = host[start:end]
code = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include "h7_link_proto.h"
#include "g4_ascii.h"
#define TPS25751_ROLE_SINK 1
#define TPS25751_ROLE_SOURCE 2
typedef struct { unsigned count; struct {uint32_t raw;} pdo[7]; } TPS25751_Capabilities_t;
typedef struct {
 uint32_t attach_generation;
 struct {struct {bool attached;int role;} tps;bool pd_reset_busy;struct {uint32_t active_rdo_raw,active_pdo_raw;} pd_snapshot;} status;
 bool partner_source_caps_current,partner_source_caps_fresh,local_source_caps_valid;
 TPS25751_Capabilities_t partner_source_caps,local_source_caps;
 bool coherent_snapshot_valid,snap_rdo_valid,snap_pdo_valid;int snap_role;
} MockManager;
static MockManager g_pm;
''' + caps_type + '\n' + getter + r'''
static G4Port h7port;
static uint8_t s_host_tx_seq;
#define LINK_UART_PRI_SLOW 2
static bool HostLink_QueueFrame(uint8_t type,uint8_t seq,const uint8_t *payload,size_t size,int pri,int slot){
 uint8_t bytes[120];assert(type==0x14 && size==44 && pri==2 && slot==3);
 size_t n=g4_frame(bytes,type,seq,payload,size);assert(n>0);g4_rx_bytes(&h7port,bytes,n,1234);return true;
}
static void pack(PowerManager_PdCapabilities_t caps){uint8_t caps_payload[H7_LINK_PD_CAPS_BYTES]={0};
''' + packing + r'''
}
int main(void){
 PowerManager_PdCapabilities_t c;G4PdCapabilities got;g4_init(&h7port);
 g_pm.attach_generation=42;g_pm.status.tps.role=1;
 PowerManager_GetPdCapabilities(&c);assert(c.flags==0 && c.count==0);
 g_pm.status.tps.attached=true;g_pm.partner_source_caps_current=true;
 g_pm.partner_source_caps.count=7;g_pm.partner_source_caps.pdo[4].raw=(400U<<10)|300;
 PowerManager_GetPdCapabilities(&c);assert(c.flags==1 && !c.count); // not fresh
 g_pm.partner_source_caps_fresh=true;g_pm.coherent_snapshot_valid=true;
 g_pm.snap_rdo_valid=g_pm.snap_pdo_valid=true;g_pm.snap_role=1;
 g_pm.status.pd_snapshot.active_rdo_raw=(5U<<28)|(300U<<10)|300;
 g_pm.status.pd_snapshot.active_pdo_raw=g_pm.partner_source_caps.pdo[4].raw;
 PowerManager_GetPdCapabilities(&c);assert(c.flags==7&&c.count==7&&c.attach_generation==42);
 pack(c);g4_pd_capabilities_snapshot(&h7port,&got);
 assert(got.valid&&got.role==1&&got.flags==7&&got.count==7&&got.ms==1234&&got.attach_generation==42);
 assert(got.rdo==c.rdo&&got.pdo[4]==c.pdo[4]&&got.active_pdo==c.active_pdo);
 g_pm.attach_generation++;g_pm.partner_source_caps_current=false;
 g_pm.coherent_snapshot_valid=false;PowerManager_GetPdCapabilities(&c);
 assert(c.count==0&&c.flags==1&&!c.rdo);pack(c);g4_pd_capabilities_snapshot(&h7port,&got);
 assert(got.attach_generation==43&&got.count==0&&!got.rdo);
 g_pm.status.tps.role=2;g_pm.local_source_caps_valid=true;g_pm.local_source_caps.count=1;
 g_pm.local_source_caps.pdo[0].raw=(100U<<10)|300;
 PowerManager_GetPdCapabilities(&c);assert(c.flags==11&&c.count==1&&c.pdo[0]==((100U<<10)|300));
 pack(c);g4_pd_capabilities_snapshot(&h7port,&got);assert(got.flags==11&&got.role==2&&got.count==1);
 g_pm.status.pd_reset_busy=true;PowerManager_GetPdCapabilities(&c);assert(!c.flags&&!c.count&&!c.rdo);
 PowerManager_GetPdCapabilities(NULL);
 puts("PASS actual G4 export/packing -> H7 parser: all seven PDOs, RDO, role, attach epoch, freshness, local source and reset");
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp)
    (p / 'test.c').write_text(code)
    exe = p / ('test.exe' if os.name == 'nt' else 'test')
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(root/'Core/Inc'), '-I'+str(h7/'Appli/Core/Inc'),
                    str(p/'test.c'), str(h7/'Appli/Core/Src/g4_ascii.c'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
