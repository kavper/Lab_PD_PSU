"""Compile the actual G4 CLEAR handler with a deterministic actuator boundary."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = (root / 'Core/Src/ldo_link.c').read_text()
start = source.index('void LdoLink_ClearFaults(void)')
brace = source.index('{', start)
depth = 1
end = brace + 1
while depth:
    depth += (source[end] == '{') - (source[end] == '}')
    end += 1
handler = source[start:end]
harness = r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#define H7_LINK_NACK_UNSAFE 4
#define LDO_G0_CTRL_SEND_OUT_OFF 10
static bool wanted=true, permit=true, force_off=false;
#define LDO_STOP_NONE 0
static unsigned s_stop_reason=3;
static struct { bool active; } s_kill_confirm={true};
static bool s_host_inflight=true, s_link_drop_latched=true;
static uint8_t s_host_inflight_seq=42, s_retry_count=3;
static struct { bool valid; } s_host_pending={true};
static unsigned cancelled, cleared, ctrl=12;
static void LdoLink_RequestOutput(bool on) {
    wanted=on;if(!on){permit=false;force_off=true;}
}
static void LdoLink_ClearPendingAcks(void) {cleared++;}
static void LdoLink_PostHostResult(uint8_t seq,bool ok,uint8_t reason) {
    assert(seq==42 && !ok && reason==4);s_host_inflight=false;cancelled++;
}
static uint32_t HAL_GetTick(void) {return 123;}
static void LdoLink_EnterState(unsigned state,uint32_t now) {assert(now==123);ctrl=state;}
'''
harness += handler + r'''
int main(void) {
    LdoLink_ClearFaults();
    assert(!wanted && !permit && force_off && s_stop_reason==0 && !s_kill_confirm.active);
    assert(ctrl==10 && cleared==1 && cancelled==1);
    assert(!s_host_pending.valid && !s_link_drop_latched && s_retry_count==0);
    LdoLink_ClearFaults();
    assert(!wanted && !permit && ctrl==10 && cancelled==1);
    puts("PASS: actual G4 CLEAR stops output, cancels SET and exits FAULT into OFF recovery");
}
'''
with tempfile.TemporaryDirectory() as output:
    path = pathlib.Path(output)
    (path / 'test.c').write_text(harness)
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',str(path/'test.c'),'-o',str(path/'test')],check=True)
    subprocess.run([str(path/'test')],check=True)
