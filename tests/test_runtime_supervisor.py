"""Run actual G4 state-machine bodies against deterministic UART/GPIO boundaries.
No analogue regulation or physical DMA is simulated by this test.
"""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
source = (root/'Core/Src/ldo_link.c').read_text()
header = (root/'Core/Inc/ldo_link.h').read_text()
def body(name):
    start=source.index(name+'(')
    # Skip the forward declaration of HardKill.
    while source.find(';',start)<source.find('{',start):
        start=source.index(name+'(',start+len(name))
    start=source.rfind('\n',0,start)+1
    brace=source.index('{',start); end=brace+1; depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]
enum=header[header.index('typedef enum'):header.index('} LdoLink_CtrlState_t;')+len('} LdoLink_CtrlState_t;')]
status=header[header.index('typedef struct'):header.index('} LdoLink_Status_t;')+len('} LdoLink_Status_t;')]
preamble=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include "ldo_ctrl_policy.h"
#define Debug_Printf(...) ((void)0)
#define LDO_TLM_STALE_MS 500U
#define LDO_CMD_TIMEOUT_MS 150U
#define LDO_CMD_RETRY_MAX 4U
#define LDO_VIN_MIN_MV 4500U
#define LDO_VOUT_ZERO_MV 250U
#define LDO_FAULT_VIN_LOW 8U
#define H7_LINK_NACK_LINK 7U
#define H7_LINK_NACK_UNSAFE 4U
#define H7_LINK_NACK_TIMEOUT 6U
#define H7_LINK_SET_PHASE_INFLIGHT 2U
#define H7_LINK_SET_PHASE_PENDING 1U
#define H7_LINK_SET_PHASE_IDLE 0U
'''+enum+status+r'''
enum {LDO_STOP_NONE,LDO_STOP_HOST_LINK,LDO_STOP_G0_LINK,LDO_STOP_G0_KILL,LDO_STOP_G0_FAULT,LDO_STOP_START_FAILURE};
enum {LDO_PENDING_NONE,LDO_PENDING_SET,LDO_PENDING_OUT_ON,LDO_PENDING_OUT_OFF};
static LdoLink_Status_t s_status;
static LdoLink_CtrlState_t s_ctrl;
static bool s_output_wanted,s_link_drop_latched,s_dcdc_permit_request;
static bool s_host_inflight,s_setpoint_dirty,s_ack_set_ok,s_ack_out_on_ok,s_ack_out_off_ok,s_nack_seen;
static uint8_t s_stop_reason,s_host_inflight_seq,s_nack_reason,s_retry_count;
static unsigned s_pending,s_pending_since_ms,s_state_since_ms;
static LdoKillConfirm s_kill_confirm;
static LdoPendingSet s_host_pending;
static float s_g0_volts,s_g0_amps;
static unsigned now,hard_stops;
static bool permit,force_disable,override_off;
static bool send_ok=true;
static uint32_t HAL_GetTick(void){return now;}
static void LdoLink_FlushHeldHostResult(void){}
static void LdoLink_SetPermitPin(bool on){permit=on;}
static void LdoPrereg_SetForceDisable(bool on){force_disable=on;}
static void LdoPrereg_SetPermitOverrideOff(bool on){override_off=on;}
static bool LdoPrereg_IsPermitGranted(void){return permit;}
static bool LdoLink_IsPowerPermitted(void){return permit;}
static void PSU_Stop(void){hard_stops++;}
static void LdoLink_ClearPendingAcks(void){s_pending=0;s_ack_set_ok=s_ack_out_on_ok=s_ack_out_off_ok=s_nack_seen=false;}
static void LdoLink_PostHostResult(uint8_t seq,bool ack,uint8_t reason){(void)seq;(void)ack;(void)reason;s_host_inflight=false;}
static bool LdoLink_HostResultWaiting(void){return false;}
static bool LdoLink_SendSet(uint32_t t,bool replay){(void)replay;LdoLink_ClearPendingAcks();s_pending=LDO_PENDING_SET;s_pending_since_ms=t;return send_ok;}
static bool LdoLink_SendOutOn(uint32_t t){LdoLink_ClearPendingAcks();s_pending=LDO_PENDING_OUT_ON;s_pending_since_ms=t;return send_ok;}
static bool LdoLink_SendOutOff(uint32_t t){LdoLink_ClearPendingAcks();s_pending=LDO_PENDING_OUT_OFF;s_pending_since_ms=t;return send_ok;}
static void LdoLink_HardKillFromFault(const char *why,uint8_t reason);
'''
# why is consumed only by a production debug macro.
preamble=preamble.replace('#define Debug_Printf(...) ((void)0)', 'static void Debug_Printf(const char *fmt,...){(void)fmt;}')
code=preamble+'\n'+ '\n'.join(body(n) for n in ['LdoLink_EnterState','LdoLink_HardKillFromFault','LdoLink_TlmFresh','LdoLink_PendingTimedOut','LdoLink_CtrlTask','LdoLink_RequestOutput','LdoLink_ClearFaults'])
code+=r'''
static void reset(void){
 s_status=(LdoLink_Status_t){0};s_host_pending=(LdoPendingSet){0};
 s_ctrl=LDO_G0_CTRL_IDLE;s_output_wanted=false;s_stop_reason=0;s_kill_confirm.active=false;
 s_host_inflight=s_setpoint_dirty=s_link_drop_latched=false;s_retry_count=0;s_pending=0;
 s_ack_set_ok=s_ack_out_on_ok=s_ack_out_off_ok=s_nack_seen=false;
 permit=true;force_disable=override_off=false;now=10;hard_stops=0;send_ok=true;
 s_status.telemetry_valid=true;s_status.vin_mv=6000;s_status.pgood=1;
}
static void tick(uint32_t t,bool fresh){now=t;if(fresh)s_status.last_tlm_ms=t;LdoLink_CtrlTask(t);}
static void running(void){
 reset();LdoLink_RequestOutput(true);
 tick(11,true);assert(s_ctrl==LDO_G0_CTRL_WAIT_LINK);
 tick(12,true);assert(s_ctrl==LDO_G0_CTRL_WAIT_PERMIT);
 tick(13,true);assert(s_ctrl==LDO_G0_CTRL_WAIT_VIN);
 tick(14,true);assert(s_ctrl==LDO_G0_CTRL_SEND_SET);
 tick(15,true);assert(s_ctrl==LDO_G0_CTRL_WAIT_SET_ACK);
 s_ack_set_ok=true;tick(16,true);assert(s_ctrl==LDO_G0_CTRL_SEND_OUT_ON);
 tick(17,true);assert(s_ctrl==LDO_G0_CTRL_WAIT_OUT_ON_ACK);
 s_ack_out_on_ok=true;tick(18,true);assert(s_ctrl==LDO_G0_CTRL_RUNNING);
 s_pending=0;s_status.output_on=true;
}
int main(void){
 running();for(unsigned t=19;t<10000;t++)tick(t,true);
 assert(s_ctrl==LDO_G0_CTRL_RUNNING&&s_output_wanted&&hard_stops==0);
 s_status.kill_reported=1;tick(10000,true);tick(10004,true);
 s_status.kill_reported=0;tick(10005,true);assert(s_output_wanted&&s_stop_reason==0);
 s_status.kill_reported=1;tick(10010,true);tick(10059,true);assert(s_output_wanted);
 tick(10060,true);assert(!s_output_wanted&&s_ctrl==LDO_G0_CTRL_FAULT&&s_stop_reason==3&&!permit);
 tick(10061,true);assert(!s_output_wanted&&s_ctrl==LDO_G0_CTRL_FAULT&&hard_stops==1);
 LdoLink_ClearFaults();assert(s_stop_reason==0&&s_ctrl==LDO_G0_CTRL_SEND_OUT_OFF&&!s_output_wanted);
 running();s_status.fault_flags=8;tick(20,true);assert(s_output_wanted&&s_ctrl==LDO_G0_CTRL_WAIT_VIN);
 running();s_status.fault_flags=256;tick(20,true);assert(!s_output_wanted&&s_stop_reason==4&&!permit);
 running();tick(519,false);assert(!s_output_wanted&&s_stop_reason==2&&s_ctrl==LDO_G0_CTRL_FAULT);
 reset();LdoLink_RequestOutput(true);s_ctrl=LDO_G0_CTRL_SEND_SET;send_ok=false;
 for(unsigned t=11;t<20;t++)tick(t,true);
 assert(!s_output_wanted&&s_ctrl==LDO_G0_CTRL_FAULT&&s_stop_reason==5&&!permit&&hard_stops==1);
 tick(30,true);assert(s_ctrl==LDO_G0_CTRL_FAULT&&hard_stops==1);
 puts("PASS: actual G4 start/running/SET/OUT state machine, 10 s stable run, KILL debounce, VIN recovery, measurement/link faults, failed-start stop, CLEAR and no auto restart");
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'test.c').write_text(code)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(root/'Core/Inc'),str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
