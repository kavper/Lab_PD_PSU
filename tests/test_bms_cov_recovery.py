"""Run the production fault decoder against COV recovery and CUV scenarios."""
from pathlib import Path
import subprocess, tempfile
r=Path(__file__).resolve().parents[1]
s=(r/'Core/Src/bq76922.c').read_text()
h=(r/'Core/Inc/bq76922.h').read_text()
a=s.index('static void BQ76922_DecodeSafetyFaults(')
b=s.index('\nstatic void BQ76922_UpdateWarnState',a)
flags=h[h.index('typedef enum {\n    BQ76922_FAULT_NONE'):h.index('} BQ76922_FaultFlags_t;')+len('} BQ76922_FaultFlags_t;')]
macros=s[s.index('#define BQ76922_SAFETY_A_SCD'):s.index('/* Battery Status 0x12 */')]
code='''#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#define BMS_CUV_EFFECTIVE_MV 2479U
#define BQ76922_STATE_FAULT 6
 typedef struct { struct {
 uint8_t safety_status_a,safety_status_b,safety_status_c;
 bool sample_valid,alert_latched,configured,shutdown_request;
 int16_t min_cell_mv,max_cell_mv;
 uint32_t fault_flags; int state;
 } snapshot; } BQ76922_Device_t;
'''+flags+'\n'+macros+s[a:b]+'''
int main(void) {
 BQ76922_Device_t d={0};
 d.snapshot.configured=true; d.snapshot.sample_valid=true;
 d.snapshot.min_cell_mv=3420; d.snapshot.max_cell_mv=3620;
 d.snapshot.safety_status_a=8;
 BQ76922_DecodeSafetyFaults(&d);
 assert(d.snapshot.fault_flags==BQ76922_FAULT_CELL_OV);
 assert(!d.snapshot.shutdown_request);
 /* COV remains reported below trip, until the AFE clears its latch. */
 d.snapshot.max_cell_mv=3580;
 BQ76922_DecodeSafetyFaults(&d);
 assert(d.snapshot.fault_flags==BQ76922_FAULT_CELL_OV);
 d.snapshot.safety_status_a=0;
 BQ76922_DecodeSafetyFaults(&d);
 assert(d.snapshot.fault_flags==0 && !d.snapshot.shutdown_request);
 /* Genuine low-cell protection must still request host shutdown. */
 d.snapshot.safety_status_a=4; d.snapshot.min_cell_mv=2400;
 BQ76922_DecodeSafetyFaults(&d);
 assert(d.snapshot.fault_flags&BQ76922_FAULT_CELL_UV);
 assert(d.snapshot.shutdown_request && d.snapshot.state==BQ76922_STATE_FAULT);
 /* Other safety conditions remain visible. */
 d.snapshot.safety_status_a=0x18;
 BQ76922_DecodeSafetyFaults(&d);
 assert(d.snapshot.fault_flags&BQ76922_FAULT_SAFETY);
 assert(d.snapshot.fault_flags&BQ76922_FAULT_CELL_OV);
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'test.c';p.write_text(code);exe=Path(t)/'test'
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('PASS: actual fault decoder, COV hysteresis/recovery, CUV host shutdown, combined protections')
