"""Compile real G0 telemetry packing -> G4 decoding/forwarding -> H7 parser.
Usage: python3 tests/test_peer_protocol.py /path/to/G0 /path/to/H7
Hardware measurements/GPIO and transport submission are deterministic fixtures.
"""
from pathlib import Path
import subprocess,sys,tempfile
root=Path(__file__).resolve().parents[1];g0=Path(sys.argv[1]);h7=Path(sys.argv[2])
a=(g0/'Core/Src/uart_protocol.c').read_text();b=(root/'Core/Src/ldo_link.c').read_text();c=(root/'Core/Src/host_link.c').read_text()
def extract(s,n):
 start=s.index(n+'('); start=s.rfind('\n',0,start)+1
 brace=s.index('{',start);end=brace+1;depth=1
 while depth:
  depth+=(s[end]=='{')-(s[end]=='}');end+=1
 return s[start:end]
def decl(path,start,end):
 s=path.read_text();return s[s.index(start):s.index(end)+len(end)]
code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "g4_ascii.h"
#include "h7_link_proto.h"
#include "host_heartbeat.h"
#include "ldo_tlm_layout.h"
#include "measurements.h"
#include "control.h"
'''+decl(root/'Core/Inc/ldo_link.h','typedef enum','} LdoLink_CtrlState_t;')+decl(root/'Core/Inc/ldo_link.h','typedef struct','} LdoLink_Status_t;')+decl(root/'Core/Inc/ldo_prereg.h','typedef struct','} LdoPrereg_Status_t;')+r'''
#define PGOOD_5V_IN_GPIO_Port 0
#define PGOOD_5V_IN_Pin 0
#define POWER_KILL_GPIO_Port 0
#define POWER_KILL_Pin 1
#define CC_CV_STATE_GPIO_Port 0
#define CC_CV_STATE_Pin 2
#define OUT_OFF_GPIO_Port 0
#define OUT_OFF_Pin 3
#define PGOOD_ASSERTED_LEVEL 1
#define POWER_KILL_ASSERTED_LEVEL 1
#define CC_CV_STATE_CC_LEVEL 1
#define OUT_OFF_ASSERTED_LEVEL 1
#define LDO_TLM_PAYLOAD_LENGTH 68
#define LINK_UART_PRI_FAST 0
static unsigned now=100;
static bool fresh=true;
static uint32_t faults;
static Measurements_Data_t m;
static Control_Status_t ctl;
static LdoLink_Status_t s_status;
static uint8_t s_host_tx_seq;
static int pin[4]={1,0,0,0};
static G4Port host;
static const Measurements_Data_t *Measurements_GetData(void){return &m;}
static const Control_Status_t *Control_GetStatus(void){return &ctl;}
static bool Measurements_CriticalFresh(void){return fresh;}
static bool Bleeder_IsEnabled(void){return false;}
static uint8_t FanRequest_Percent(void){return 23;}
static uint32_t uart_fault_flags(void){return faults|(fresh?0:256);}
static int HAL_GPIO_ReadPin(int port,int p){(void)port;return pin[p];}
static uint32_t HAL_GetTick(void){return now;}
static uint32_t LdoLink_GetU32Le(const uint8_t *p){return H7Link_GetU32(p);}
static void LdoPrereg_GetStatus(LdoPrereg_Status_t *p){*p=(LdoPrereg_Status_t){.vpre_request_v=6,.vpre_command_v=6,.permit_granted=true};}
static void LdoLink_GetStatus(LdoLink_Status_t *p){*p=s_status;}
static float App_GetInputVoltage(void){return 15;}
static float App_GetOutputVoltage(void){return 6;}
static float App_GetHsBuckCurrent(void){return 0.1f;}
static float App_GetHsBoostCurrent(void){return 0;}
static float LdoLink_GetG0Voltage(void){return 3;}
static float LdoLink_GetG0Current(void){return 1;}
static uint32_t PowerStage_GetDutyA10k(void){return 4000;}
static uint32_t PowerStage_GetDutyCPhys10k(void){return 0;}
static uint32_t App_GetFaultFlags(void){return 0;}
static bool LdoLink_IsOutputWanted(void){return true;}
static bool LdoPrereg_IsPermitGranted(void){return true;}
static bool PSU_IsRunning(void){return true;}
static bool App_IsStageEnabled(void){return true;}
static bool PowerStage_IsEnabled(void){return true;}
static bool PowerStage_IsFaultActive(void){return false;}
static bool LdoLink_IsRemoteSenseEnabled(void){return false;}
static bool PowerStage_IsBuckTrEnActive(void){return true;}
static bool PowerStage_IsBoostTrEnActive(void){return false;}
static LdoLink_CtrlState_t LdoLink_GetCtrlState(void){return LDO_G0_CTRL_RUNNING;}
static uint8_t LdoLink_HostSetPhase(void){return 0;}
static bool LdoLink_TakeHostSetResult(uint8_t *seq,uint8_t *ack,uint8_t *reason){(void)seq;(void)ack;(void)reason;return false;}
static void HostLink_QueueAck(uint8_t seq,uint8_t type){(void)seq;(void)type;}
static void HostLink_QueueNack(uint8_t seq,uint8_t type,uint8_t reason){(void)seq;(void)type;(void)reason;}
static bool HostLink_QueueFrame(uint8_t type,uint8_t seq,const uint8_t *p,uint8_t len,int pri,uint8_t slot){
 uint8_t frame[120];(void)pri;(void)slot;
 size_t n=H7Link_Build(frame,sizeof(frame),type,seq,p,len);
 assert(n);g4_rx_bytes(&host,frame,n,now);return true;
}
'''
# Public measurement/control declarations are included above.
code=code.replace('static const Measurements_Data_t *Measurements_GetData', 'const Measurements_Data_t *Measurements_GetData').replace('static const Control_Status_t *Control_GetStatus','const Control_Status_t *Control_GetStatus').replace('static bool Measurements_CriticalFresh','bool Measurements_CriticalFresh')
code+='\n'+'\n'.join(extract(a,n) for n in ['uart_put_u16_le','uart_put_i16_le','uart_put_u32_le','uart_temperature_deci_c','uart_fill_telemetry'])
code+='\n'+extract(b,'LdoLink_HandleTelemetry')
code+='\n'+'\n'.join(extract(c,n) for n in ['HostLink_Mv','HostLink_Ma','HostLink_DutyX10','HostLink_QueueMeter'])
code+=r'''
int main(void){
 uint8_t p[68],frame[120];unsigned n;
 g4_init(&host);m.vout_mV=3000;m.vin_mV=6000;m.iout_mA=123;
 m.dac_cv_readback_mV=250;m.dac_cc_readback_mV=500;
 for(unsigned i=0;i<4;i++){m.temperature_centi_C[i]=2530+(int)i*10;m.temperature_raw[i]=1000+i;m.temperature_filtered[i]=1100+i;}
 ctl.voltage_target_mV=3000;ctl.current_target_mA=1000;ctl.vpre_request_mV=6000;ctl.output_enabled=true;ctl.mode=CONTROL_MODE_CV;
 assert(uart_fill_telemetry(p)==68);LdoLink_HandleTelemetry(p,68);HostLink_QueueMeter();
 assert(s_status.vout_mv==3000&&s_status.vin_mv==6000&&s_status.dac_cv_mv==250);
 assert(host.telemetry.vin_mv==15000&&host.telemetry.rail_mv==6000);
 assert(host.telemetry.g0_vin_mv==6000&&host.telemetry.vout_mv==3000&&host.telemetry.iout_ma==123);
 assert(host.telemetry.g0_vset_mv==3000&&host.telemetry.g0_iset_ma==1000&&host.telemetry.ctrl==9&&host.telemetry.out);
 assert(s_status.temp_centi_c[0]==253&&s_status.fan_percent==23&&s_status.kill_reported==0);
 /* H7 heartbeat must keep the G4 watchdog alive across loss of METER. */
 uint32_t last=100;
 for(now=200;now<10000;now+=100){
  g4_process(&host,now);n=g4_pop_frame(&host,frame,sizeof(frame),now);assert(n&&frame[3]==H7_LINK_PING);
  H7LinkParser parser;H7Link_ParserInit(&parser);int valid=0;
  for(unsigned i=0;i<n;i++)valid|=H7Link_ParserByte(&parser,frame[i]);
  assert(valid&&parser.type==H7_LINK_PING);last=now;
  assert(!HostHeartbeat_Expired(true,now,last));
 }
 assert(HostHeartbeat_Expired(true,last+1001,last));
 now=10001;fresh=false;faults=256;assert(uart_fill_telemetry(p)==68);
 LdoLink_HandleTelemetry(p,68);HostLink_QueueMeter();
 assert(host.telemetry.vout_mv==0&&host.telemetry.g0_vin_mv==0&&host.telemetry.g0_fault==256);
 assert(s_status.temp_centi_c[0]==INT16_MIN);
 puts("PASS: actual G0 packing -> G4 decoding/METER forwarding -> H7 parser; voltage/current/temperature/fault units, 10 s no-METER heartbeat and G4 disconnect deadline");
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'test.c').write_text(code)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-DPSU_SIMULATOR','-I'+str(g0/'Core/Inc'),'-I'+str(root/'Core/Inc'),'-I'+str(h7/'Appli/Core/Inc'),str(p/'test.c'),str(h7/'Appli/Core/Src/g4_ascii.c'),'-lm','-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
