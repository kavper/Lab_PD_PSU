"""Execute production BQ stop/readback cases and fail-closed handler."""
from pathlib import Path
import tempfile, subprocess
root=Path(__file__).resolve().parents[1]
s=(root/'Core/Src/power_manager.c').read_text()
a=s.index('static void PowerManager_FailBqStop(void)');b=s.index('\n}\n',a)+3
fail=s[a:b]
a=s.index('static void PowerManager_FinishSessionResetIfIdle(uint32_t now_ms)\n{');b=s.index('\nstatic void PowerManager_QueuePolicyPortConfig',a)
finish=s[a:b]
a=s.index('        case PM_JOB_BQ_READ_OPTION3:\n            raw16 =');b=s.index('\n        case PM_JOB_BQ_WRITE_ADC:',a)
cases=s[a:b]
code=r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define PM_JOB_BQ_READ_OPTION3 1
#define PM_JOB_BQ_WRITE_OPTION3 2
#define PM_JOB_BQ_VERIFY_STOP_OPTION3 3
#define BQ25731_CHARGE_OPTION3_EN_OTG 0x1000U
#define BQ25731_CHARGE_OPTION3_EN_HIZ 0x8000U
#define BQ25731_CHARGE_OPTION3_EN_OTG_BIGCAP 1U
#define TPS25751_PORT_DISABLED 0
#define POWER_MANAGER_FAULT 99
#define PM_BQ_OWNER_STM_EMERGENCY 2
#define PM_SESSION_RESET_COOLDOWN_MS 1500
#define Debug_Printf(...) ((void)0)
static struct {
 bool bq_option3_read_pending,bq_option3_write_pending,bq_option3_verify_pending;
 bool session_bq_stop_verified,session_reset_busy,session_reset_wait_vsafe0;
 bool session_reset_restore_config,session_reset_faulted,mode_update_pending;
 bool port_config_valid,port_write_pending,port_control_read_pending,port_control_write_pending;
 bool dbfg_pending,bq_option3_valid;
 int bq_owner;uint16_t bq_option3_target;uint8_t port_config[17];
 uint32_t session_reset_cooldown_until_ms;
 struct {bool session_reset_faulted,pd_reset_busy,source_fault_latched,applied_mode_valid;
 int state;struct {uint16_t charge_option3;} bq;} status;
} g_pm;
static uint16_t input;static bool input_valid=true,pin_high;
static void PowerManager_ForceOtgOff(void){pin_high=false;}
static void TPS25751_SetPortStateMachine(uint8_t *p,int mode){p[0]=(uint8_t)mode;}
static uint16_t PowerManager_ResultLe16(bool *valid){*valid=input_valid;return input;}
'''+fail+finish+r'''
static void complete(int job){bool valid;uint16_t raw16;uint32_t now_ms=100;switch(job){
'''+cases+r'''}}
static void begin(void){
 memset(&g_pm,0,sizeof(g_pm));input_valid=true;pin_high=true;
 g_pm.session_reset_busy=true;g_pm.port_config_valid=true;
 g_pm.status.applied_mode_valid=true;g_pm.port_config[0]=2;
 /* Other session steps pending: readback alone must not finish the reset. */
 g_pm.session_reset_wait_vsafe0=true;
}
static void assert_failed(void){
 assert(g_pm.session_reset_faulted&&g_pm.status.session_reset_faulted);
 assert(!g_pm.session_reset_busy&&!g_pm.status.applied_mode_valid);
 assert(g_pm.status.source_fault_latched&&g_pm.status.state==POWER_MANAGER_FAULT);
 assert(!pin_high&&g_pm.port_write_pending&&g_pm.port_config[0]==0);
 assert(!g_pm.bq_option3_verify_pending&&!g_pm.bq_option3_write_pending);
}
int main(void){
 begin();input=0x1001;complete(1);
 assert(g_pm.bq_option3_target==0x8001&&g_pm.bq_option3_write_pending);
 complete(2);
 assert(!g_pm.bq_option3_write_pending&&g_pm.bq_option3_verify_pending);
 assert(g_pm.status.bq.charge_option3==0x1001&&!g_pm.session_bq_stop_verified);
 PowerManager_FinishSessionResetIfIdle(100);assert(g_pm.session_reset_busy);
 input=0x8001;complete(3);
 assert(g_pm.session_bq_stop_verified&&g_pm.status.bq.charge_option3==0x8001);
 assert(g_pm.session_reset_busy); /* vSafe0 still pending */
 g_pm.session_reset_wait_vsafe0=false;
 g_pm.bq_option3_target=1;g_pm.bq_option3_write_pending=true;
 complete(2);PowerManager_FinishSessionResetIfIdle(100);assert(g_pm.session_reset_busy);
 input=1;complete(3);assert(!g_pm.session_reset_busy);
 const uint16_t bad[]={0x9001,1,0x8000};
 for(unsigned i=0;i<3;i++){
  begin();g_pm.bq_option3_target=0x8001;input=bad[i];complete(3);assert_failed();
 }
 begin();g_pm.bq_option3_target=0x8001;input_valid=false;complete(3);assert_failed();
 begin();input_valid=false;complete(1);assert_failed();
 puts("PASS: production OTG stop/readback/release; mismatch and invalid reads keep port Disabled/FAULT; pending verification blocks reset completion");
}
'''
with tempfile.TemporaryDirectory() as t:
 c=Path(t)/'test.c';c.write_text(code);exe=Path(t)/'test'
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
