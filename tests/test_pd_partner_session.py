"""Execute production STATUS/INT_EVENT session handling for USB partners.

Covers a partner joining an e-marked cable that is already inserted (no
PlugPresent edge), Source_Capabilities read before STATUS shows the
connection, and DRP toggling that must not count as a role change.
"""
from pathlib import Path
import tempfile, subprocess
root=Path(__file__).resolve().parents[1]
s=(root/'Core/Src/power_manager.c').read_text()
def cut(start,end):
    a=s.index(start);b=s.index(end,a);return s[a:b]
conn=cut('static bool PowerManager_HasTypecPowerConnection(void)','\nstatic void PowerManager_LogCapabilities')
a=s.index('static void PowerManager_StartSession(const char *why)')
b=s.index('\n}\n',s.index('static void PowerManager_UpdateAttachPolicy(uint32_t now_ms)'))+3
session=s[a:b]
status_case=cut('        case PM_JOB_READ_STATUS:\n        {','        case PM_JOB_READ_TYPEC_STATE:')
caps_event=cut('    if (event.source_caps_received) {','    if (event.plug_changed || event.source_caps_received ||')
code=r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define Debug_Printf(...) ((void)0)
typedef enum {TPS25751_ROLE_UNKNOWN=0,TPS25751_ROLE_SINK,TPS25751_ROLE_SOURCE} TPS25751_PowerRole_t;
enum {PM_SRC_PATH_IDLE=0,PM_SRC_PATH_WAIT_CONTRACT=1};
enum {POWER_MANAGER_USER_AUTO=0,POWER_MANAGER_USER_SINK_ONLY=1};
#define PM_AUTO_SRC_FAIL_MS 1500U
#define PM_AUTO_SRC_FAIL_WINDOW_MS 6000U
#define PM_AUTO_TRY_SNK_MS 15000U
typedef struct {unsigned count;} Caps;
static struct {
 struct {struct {bool attached;TPS25751_PowerRole_t role;uint8_t connection_state;uint32_t updated_ms;
  uint8_t typec_port_state;uint64_t status_raw;} tps;uint32_t attach_generation;int requested_mode;} status;
 Caps partner_source_caps,partner_sink_caps;
 bool partner_source_caps_current,partner_source_caps_fresh,session_caps_event_this_gen;
 bool source_caps_event_pending,source_caps_trace_pending,local_source_caps_valid;
 bool local_source_caps_pending,coherent_snapshot_valid,pdo_report_pending;
 bool previous_attached,typec_initial_role_latched,session_gsrc_ok_this_gen;
 bool session_gsrc_failed_this_gen,auto_swap_command_sent;
 TPS25751_PowerRole_t typec_initial_role,mode_swap_target;
 bool mode_swap_command_pending,mode_update_pending,port_mode_deferred;uint8_t mode_change_path;
 uint32_t auto_src_attach_ms,auto_src_fail_ms,auto_try_snk_until_ms;uint8_t auto_src_fail_count;bool auto_try_snk;
 int source_path_phase;uint8_t telemetry_phase,stuck_confirm_count;
 uint32_t attach_generation,attach_count,detach_count,stuck_debounce_until_ms;
} g_pm;
static struct {bool attached;TPS25751_PowerRole_t role;uint8_t conn;} next;
static int resets,reset_attached,invalidates,contract_clears;
static void PowerManager_Log(uint8_t k,uint8_t a,uint16_t b,uint32_t c){(void)k;(void)a;(void)b;(void)c;}
static void PowerManager_WatchRoleChange(uint32_t now_ms){(void)now_ms;}
static void TPS25751_DecodeStatus(void *t,const uint8_t *d){(void)t;(void)d;
 g_pm.status.tps.attached=next.attached;g_pm.status.tps.role=next.role;g_pm.status.tps.connection_state=next.conn;}
static const char *PowerManager_RoleToString(TPS25751_PowerRole_t r){(void)r;return "";}
static void PowerManager_ClearContractTelemetry(void){contract_clears++;}
static void PowerManager_ResetPolicy(uint32_t now_ms,bool attached){(void)now_ms;resets++;reset_attached=attached;
 g_pm.partner_source_caps_current=g_pm.partner_source_caps_fresh=false;}
static void PowerManager_InvalidateSessionOnDetach(uint32_t now_ms){(void)now_ms;invalidates++;
 g_pm.session_caps_event_this_gen=false;g_pm.source_caps_event_pending=false;g_pm.source_caps_trace_pending=false;}
'''+conn+'\n'+session+r'''
static uint32_t clock_ms=10;
static void status(bool attached,TPS25751_PowerRole_t role,uint8_t c){
 const uint8_t *data=0;uint32_t now_ms=clock_ms;next.attached=attached;next.role=role;next.conn=c;
 switch(1){case 1:
'''+status_case.replace('        case PM_JOB_READ_STATUS:\n','',1)+r'''}}
static void caps_event(void){struct {bool source_caps_received;} event={true};
'''+caps_event+r'''}
int main(void){
 /* Cable alone: PlugPresent edge, Ra only. */
 status(true,TPS25751_ROLE_SINK,4);
 assert(g_pm.attach_count==1&&resets==1);
 uint32_t gen=g_pm.attach_generation;
 /* DRP toggling while cable-only is not a role change. */
 int clears=contract_clears;
 status(true,TPS25751_ROLE_SOURCE,4);status(true,TPS25751_ROLE_SINK,4);
 assert(contract_clears==clears&&!g_pm.pdo_report_pending&&g_pm.attach_generation==gen);
 /* Charger joins the cable: caps event is read before STATUS shows conn=6. */
 caps_event();
 assert(g_pm.source_caps_event_pending&&!g_pm.session_caps_event_this_gen);
 status(true,TPS25751_ROLE_SINK,6);
 assert(g_pm.attach_count==2&&resets==2&&reset_attached);
 assert(g_pm.attach_generation==gen+1&&g_pm.source_path_phase==PM_SRC_PATH_WAIT_CONTRACT);
 assert(g_pm.session_caps_event_this_gen&&g_pm.source_caps_trace_pending&&!g_pm.source_caps_event_pending);
 /* Caps during an established connection apply immediately. */
 g_pm.session_caps_event_this_gen=g_pm.source_caps_trace_pending=false;
 caps_event();
 assert(g_pm.session_caps_event_this_gen&&g_pm.source_caps_trace_pending);
 /* A real swap with the partner present is still a role change. */
 clears=contract_clears;
 status(true,TPS25751_ROLE_SOURCE,6);
 assert(contract_clears==clears+1&&g_pm.pdo_report_pending&&resets==2);
 /* Partner leaves, cable stays: generation changes, pending event discarded. */
 gen=g_pm.attach_generation;
 caps_event();caps_event();
 g_pm.source_caps_event_pending=true;
 status(true,TPS25751_ROLE_SINK,4);
 assert(g_pm.attach_generation==gen+1&&!g_pm.session_caps_event_this_gen);
 assert(!g_pm.source_caps_event_pending&&!g_pm.source_caps_trace_pending);
 /* A stale pending event without a partner never marks caps fresh. */
 caps_event();
 status(true,TPS25751_ROLE_SOURCE,4);
 assert(!g_pm.source_caps_event_pending&&!g_pm.session_caps_event_this_gen);
 /* Fresh plug with partner: normal attach path, single session start. */
 g_pm.port_mode_deferred=true;g_pm.mode_update_pending=false;
 status(false,TPS25751_ROLE_UNKNOWN,0);
 assert(invalidates==1&&g_pm.mode_update_pending&&!g_pm.port_mode_deferred);
 uint32_t attaches=g_pm.attach_count;
 caps_event();
 status(true,TPS25751_ROLE_SINK,6);
 assert(g_pm.attach_count==attaches+1&&g_pm.session_caps_event_this_gen);
 /* AUTO: two source sessions that collapse within 1.5 s -> Try.SNK. */
 g_pm.status.requested_mode=POWER_MANAGER_USER_AUTO;g_pm.mode_update_pending=false;
 status(true,TPS25751_ROLE_SINK,4);
 clock_ms=1000;status(true,TPS25751_ROLE_SOURCE,7);clock_ms=1200;status(true,TPS25751_ROLE_SINK,0);
 assert(!g_pm.auto_try_snk&&g_pm.auto_src_fail_count==1);
 clock_ms=1800;status(true,TPS25751_ROLE_SOURCE,7);clock_ms=2000;status(true,TPS25751_ROLE_SINK,0);
 assert(g_pm.auto_try_snk&&g_pm.mode_update_pending&&g_pm.auto_try_snk_until_ms==17000);
 /* A source session that holds is not a failure; nor is SINK ONLY mode. */
 g_pm.auto_try_snk=false;g_pm.auto_src_fail_count=0;
 clock_ms=30000;status(true,TPS25751_ROLE_SOURCE,7);clock_ms=33000;status(true,TPS25751_ROLE_SINK,0);
 assert(!g_pm.auto_src_fail_count);
 g_pm.status.requested_mode=POWER_MANAGER_USER_SINK_ONLY;
 clock_ms=40000;status(true,TPS25751_ROLE_SOURCE,7);clock_ms=40100;status(true,TPS25751_ROLE_SINK,0);
 clock_ms=40500;status(true,TPS25751_ROLE_SOURCE,7);clock_ms=40600;status(true,TPS25751_ROLE_SINK,0);
 assert(!g_pm.auto_try_snk);
 puts("PASS: partner on inserted cable starts a session, early Source_Capabilities kept, DRP toggling ignored, swap/leave/replug, deferred port mode on detach, AUTO Try.SNK after collapsed source attaches");
}
'''
with tempfile.TemporaryDirectory() as d:
    c=Path(d)/'t.c';c.write_text(code);exe=Path(d)/'t'
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function',
                    '-Wno-unused-parameter','-Wno-unused-but-set-variable',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
