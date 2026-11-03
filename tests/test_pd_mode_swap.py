"""Execute production user mode changes: PR_Swap on a live PD session,
port mode deferred to detach, port reset without one, and reset fallback when the swap fails."""
from pathlib import Path
import tempfile, subprocess
root=Path(__file__).resolve().parents[1]
s=(root/'Core/Src/power_manager.c').read_text()
def fn(sig):
    a=s.index(sig);b=s.index('\n}\n',a)+3;return s[a:b]
conn=fn('static bool PowerManager_HasTypecPowerConnection(void)')
fail=fn('static void PowerManager_FailModeSwap(uint32_t now_ms, const char *why)')
setmode=fn('bool PowerManager_SetUserMode(PowerManager_UserMode_t mode)')
code=r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#define Debug_Printf(...) ((void)0)
#define PM_MODE_SWAP_TIMEOUT_MS 1500U
typedef enum {TPS25751_ROLE_UNKNOWN=0,TPS25751_ROLE_SINK,TPS25751_ROLE_SOURCE} TPS25751_PowerRole_t;
typedef enum {POWER_MANAGER_USER_AUTO=0,POWER_MANAGER_USER_SINK_ONLY,POWER_MANAGER_USER_SOURCE_ONLY,POWER_MANAGER_USER_OFF} PowerManager_UserMode_t;
enum {PM_POLICY_IDLE=0,PM_POLICY_WAIT_SETTLE=1};
static struct {
 struct {struct {bool attached;TPS25751_PowerRole_t role;uint8_t connection_state;
  struct {bool valid;} active_rdo;} tps;
  PowerManager_UserMode_t requested_mode,applied_mode;bool applied_mode_valid,source_fault_latched,pd_reset_busy;} status;
 TPS25751_PowerRole_t mode_swap_target,policy_desired_role,swap_policy;
 bool mode_swap_command_pending,mode_update_pending,port_write_pending,session_reset_busy;
 bool session_reset_faulted,port_config_valid,port_control_valid,policy_decided,port_mode_deferred;
 uint32_t mode_swap_deadline_ms;int policy_phase;uint8_t mode_change_path;
} g_pm;
static int resets,policy_resets;
static uint32_t HAL_GetTick(void){return 1000;}
static const char *PowerManager_RoleToString(TPS25751_PowerRole_t r){(void)r;return "";}
static const char *PowerManager_UserModeToString(PowerManager_UserMode_t m){(void)m;return "";}
static void PowerManager_QueueAutoSwapControl(TPS25751_PowerRole_t r){g_pm.swap_policy=r;}
static void PowerManager_ResetPolicy(uint32_t now_ms,bool attached){(void)now_ms;(void)attached;policy_resets++;}
static void PowerManager_BeginSessionReset(uint32_t now_ms,const char *why,bool force){(void)now_ms;(void)why;(void)force;
 resets++;g_pm.session_reset_busy=true;}
'''+conn+fail+setmode+r'''
static void live(TPS25751_PowerRole_t role){
 g_pm=(typeof(g_pm)){0};resets=policy_resets=0;
 g_pm.status.tps.attached=true;g_pm.status.tps.role=role;g_pm.status.tps.connection_state=7;
 g_pm.status.tps.active_rdo.valid=true;g_pm.port_config_valid=g_pm.port_control_valid=true;
 g_pm.status.requested_mode=POWER_MANAGER_USER_AUTO;
}
int main(void){
 /* SOURCE -> SINK on a live contract: PR_Swap, no port reset. */
 live(TPS25751_ROLE_SOURCE);
 assert(PowerManager_SetUserMode(POWER_MANAGER_USER_SINK_ONLY));
 assert(!resets&&g_pm.mode_swap_target==TPS25751_ROLE_SINK&&g_pm.mode_swap_command_pending);
 assert(g_pm.swap_policy==TPS25751_ROLE_SINK&&!g_pm.mode_update_pending&&g_pm.port_mode_deferred&&g_pm.mode_swap_deadline_ms==2500);
 assert(g_pm.status.applied_mode_valid&&g_pm.status.applied_mode==POWER_MANAGER_USER_SINK_ONLY);
 /* SINK -> SOURCE: the BQ needs its VBUS node <3.2 V first, so reset. */
 live(TPS25751_ROLE_SINK);
 assert(PowerManager_SetUserMode(POWER_MANAGER_USER_SOURCE_ONLY));
 assert(resets==1&&g_pm.mode_swap_target==TPS25751_ROLE_UNKNOWN&&!g_pm.mode_swap_command_pending);
 /* Already in the selected role: only the port policy is written. */
 live(TPS25751_ROLE_SINK);
 assert(PowerManager_SetUserMode(POWER_MANAGER_USER_SINK_ONLY));
 /* PORT_CONFIG would restart Type-C: written after detach instead. */
 assert(!resets&&g_pm.mode_swap_target==TPS25751_ROLE_UNKNOWN&&!g_pm.mode_swap_command_pending&&!g_pm.mode_update_pending&&g_pm.port_mode_deferred);
 /* AUTO on a live session re-runs the PDO based decision, no reset. */
 live(TPS25751_ROLE_SOURCE);g_pm.status.requested_mode=POWER_MANAGER_USER_SOURCE_ONLY;
 assert(PowerManager_SetUserMode(POWER_MANAGER_USER_AUTO));
 assert(!resets&&policy_resets==1&&g_pm.mode_swap_target==TPS25751_ROLE_UNKNOWN);
 /* OFF, no PD contract, cable only: full port reset as before. */
 live(TPS25751_ROLE_SOURCE);assert(PowerManager_SetUserMode(POWER_MANAGER_USER_OFF));assert(resets==1);
 live(TPS25751_ROLE_SOURCE);g_pm.status.tps.active_rdo.valid=false;
 assert(PowerManager_SetUserMode(POWER_MANAGER_USER_SINK_ONLY));assert(resets==1&&!g_pm.mode_swap_target);
 live(TPS25751_ROLE_SOURCE);g_pm.status.tps.connection_state=4;
 assert(PowerManager_SetUserMode(POWER_MANAGER_USER_SINK_ONLY));assert(resets==1);
 /* A rejected or timed-out swap falls back to the reset. */
 live(TPS25751_ROLE_SOURCE);
 assert(PowerManager_SetUserMode(POWER_MANAGER_USER_SINK_ONLY));
 PowerManager_FailModeSwap(1200,"rejected");
 assert(resets==1&&g_pm.mode_swap_target==TPS25751_ROLE_UNKNOWN&&!g_pm.mode_swap_command_pending&&!g_pm.port_mode_deferred);
 /* Same mode twice does nothing. */
 live(TPS25751_ROLE_SOURCE);
 assert(PowerManager_SetUserMode(POWER_MANAGER_USER_AUTO)&&!resets&&!policy_resets);
 puts("PASS: user mode by PR_Swap to SINK on live PD, SOURCE by reset, policy-only when role matches, AUTO re-decide, reset for OFF/no PD/cable/failed swap");
}
'''
with tempfile.TemporaryDirectory() as d:
    c=Path(d)/'t.c';c.write_text(code);exe=Path(d)/'t'
    subprocess.run(['cc','-std=gnu11','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-parameter',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
