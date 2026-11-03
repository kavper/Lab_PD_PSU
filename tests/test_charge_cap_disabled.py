"""Production charging must never schedule the temporary 512 mA clamp."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'Core/Src/power_manager.c').read_text()
a=s.index('static bool PowerManager_ChargeCapEligible(void)')
b=s.index('\nstatic void PowerManager_ChargeCapFailed',a)
gate=s[a:b]
code=r"""
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include "charge_balance_limits.h"
#define PM_BQ_INIT_DONE 1
#define TPS25751_ROLE_SINK 1
#define POWER_MANAGER_USER_SOURCE_ONLY 2
#define POWER_MANAGER_USER_OFF 3
static struct {
 int bq_init;bool session_reset_busy,session_reset_faulted;
 struct {struct {bool id_valid;} bq;struct {bool attached;int role;} tps;int requested_mode;} status;
} g_pm;
"""+gate+r"""
int main(void){
 assert(CHARGE_BALANCE_LIMITS_ENABLED==0);
 g_pm.bq_init=PM_BQ_INIT_DONE;g_pm.status.bq.id_valid=true;
 g_pm.status.tps.attached=true;g_pm.status.tps.role=TPS25751_ROLE_SINK;
 g_pm.status.requested_mode=1;
 assert(!PowerManager_ChargeCapEligible());
 puts("PASS production SINK: temporary 512mA clamp is disabled even with a fully eligible charger");
}
"""
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'test.c';p.write_text(code);out=Path(t)/'test'
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(root/'Core/Inc'),str(p),'-o',str(out)],check=True)
 subprocess.run([str(out)],check=True)
