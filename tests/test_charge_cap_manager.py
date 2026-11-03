"""Execute the production charge-cap result handler with mocked transport."""
from pathlib import Path
import tempfile, subprocess
r=Path(__file__).resolve().parents[1];s=(r/'Core/Src/power_manager.c').read_text()
a=s.index('static bool PowerManager_ChargeCapEligible(void)');b=s.index('\nstatic void PowerManager_ClearContractTelemetry',a);helpers=s[a:b]
a=s.index('    if (completed_job>=PM_JOB_CHARGE_CAP_READ');b=s.index('    if (operation_status != TPS25751_OK)',a);handler=s[a:b]
code=r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define CHARGE_BALANCE_LIMITS_ENABLED 1U /* exercise optional bench build only */
#include "charge_balance_limits.h"
#define PM_BQ_INIT_DONE 1
#define TPS25751_ROLE_SINK 1
#define POWER_MANAGER_USER_SOURCE_ONLY 2
#define POWER_MANAGER_USER_OFF 3
#define POWER_MANAGER_ERROR_BQ 1
#define TPS25751_COMMAND_ERROR 1
#define TPS25751_OK 0
#define PM_JOB_CHARGE_CAP_READ 1
#define PM_JOB_CHARGE_CAP_WRITE_I 2
#define PM_JOB_CHARGE_CAP_WRITE_V 3
#define PM_JOB_CHARGE_CAP_VERIFY 4
#define Debug_Printf(...) ((void)0)
static struct {int bq_init;struct {
 struct {bool id_valid;uint16_t charge_current,charge_voltage;} bq;
 struct {bool attached;int role;} tps;int requested_mode;
 } status;bool session_reset_busy,session_reset_faulted;
 uint8_t charge_cap_phase;bool charge_cap_voltage_pending,charge_cap_verified,charge_cap_fault;
 uint32_t charge_cap_next_ms,charge_cap_applied_count;
} g_pm;
static unsigned errors;
static void PowerManager_RecordError(int a,int b,int c,uint32_t d){(void)a;(void)b;(void)c;(void)d;errors++;}
static bool PowerManager_SetUserMode(int mode){g_pm.status.requested_mode=mode;return true;}
static bool decode(const uint8_t *d,uint8_t len){
 if(len<8)return false;
 g_pm.status.bq.charge_current=d[2]|((uint16_t)d[3]<<8);
 g_pm.status.bq.charge_voltage=d[4]|((uint16_t)d[5]<<8);return true;
}
#define BQ25731_DecodeConfigBlock(out,data,length) decode(data,length)
'''+helpers+r'''
static void result(int completed_job,int operation_status,uint16_t i,uint16_t v,uint8_t length){
 uint32_t now_ms=100;uint8_t buf[8]={0,0,i&255,i>>8,v&255,v>>8,0,0};const uint8_t *data=buf;
'''+handler+r'''}
static void begin(void){memset(&g_pm,0,sizeof(g_pm));errors=0;
 g_pm.bq_init=1;g_pm.status.bq.id_valid=true;g_pm.status.tps.attached=true;
 g_pm.status.tps.role=1;g_pm.status.requested_mode=1;}
int main(void){
 begin();result(1,0,0xfc0,18000,8);assert(g_pm.charge_cap_phase==1);
 result(2,0,0,0,0);assert(g_pm.charge_cap_phase==2&&!g_pm.charge_cap_verified);
 result(3,0,0,0,0);assert(g_pm.charge_cap_phase==3&&!g_pm.charge_cap_verified);
 result(4,0,0x100,17496,8);assert(g_pm.charge_cap_verified&&g_pm.charge_cap_phase==0);
 assert(g_pm.charge_cap_applied_count==1&&g_pm.charge_cap_next_ms==1100);
 /* A TPS overwrite is caught on the next check and invokes reapplication. */
 result(1,0,0xfc0,18000,8);assert(g_pm.charge_cap_phase==1);
 begin();result(1,0,0x80,18000,8);assert(g_pm.charge_cap_phase==2); /* don't raise lower I */
 begin();result(1,0,0xfc0,16504,8);result(2,0,0,0,0);assert(g_pm.charge_cap_phase==3); /* don't raise lower V */
 begin();result(4,0,0xfc0,17496,8);assert(g_pm.charge_cap_fault&&errors&&g_pm.status.requested_mode==3);
 begin();result(4,0,0x100,18000,8);assert(g_pm.charge_cap_fault&&g_pm.status.requested_mode==3);
 begin();result(1,0,0,0,2);assert(g_pm.charge_cap_fault&&g_pm.status.requested_mode==3);
 begin();result(2,1,0,0,0);assert(g_pm.charge_cap_fault&&g_pm.status.requested_mode==3);
 begin();g_pm.status.tps.role=2;result(2,1,0,0,0);assert(!g_pm.charge_cap_fault&&!errors);
 begin();g_pm.session_reset_busy=true;assert(!PowerManager_ChargeCapEligible());
 begin();g_pm.status.bq.id_valid=false;assert(!PowerManager_ChargeCapEligible());
 begin();g_pm.status.requested_mode=2;assert(!PowerManager_ChargeCapEligible());
 puts("PASS: production charge-cap state machine, readback, TPS overwrite, lower settings, failure OFF, role/init/reset isolation");
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'test.c';p.write_text(code);exe=Path(t)/'test'
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(r/'Core/Inc'),str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
