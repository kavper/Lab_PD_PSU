"""Execute the production shutdown function with a pack-powered host boundary."""
from pathlib import Path
import tempfile, subprocess
root=Path(__file__).resolve().parents[1]
s=(root/'Core/Src/bq76922.c').read_text()
a=s.index('BQ76922_Status_t BQ76922_EnterShutdown('); b=s.index('\nstatic BQ76922_Status_t',a)
h=r'''#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <assert.h>
#define BMS_ENABLE 1U
#define BQ76922_SUBCMD_SHUTDOWN 0x10U
#define BQ76922_SUBCMD_ALL_FETS_OFF 0x95U
typedef enum {BQ76922_OK,BQ76922_NOT_READY,BQ76922_INVALID_ARG,BQ76922_IO_ERROR} BQ76922_Status_t;
typedef struct {void *hi2c;} BQ76922_Device_t;
static bool s_passq_reset_pending, absent;
static unsigned writes; static int fail_at;
static void HAL_Delay(unsigned ms){assert(ms<=4000U);}
static BQ76922_Status_t BQ76922_SendSubcommand(BQ76922_Device_t *d,unsigned cmd){
 (void)d; assert(cmd==BQ76922_SUBCMD_SHUTDOWN); writes++;
 return (int)writes==fail_at?BQ76922_IO_ERROR:BQ76922_OK;
}
static void BQ76922_ForceAbsent(BQ76922_Device_t *d){(void)d;absent=true;}
'''+s[a:b]+r'''
int main(void){
 BQ76922_Device_t d={(void*)1};
 assert(BQ76922_EnterShutdown(NULL)==BQ76922_INVALID_ARG);
 assert(BQ76922_EnterShutdown(&d)==BQ76922_OK);
 assert(writes==2&&absent&&s_passq_reset_pending);
 writes=0;absent=false;fail_at=1;
 assert(BQ76922_EnterShutdown(&d)==BQ76922_IO_ERROR);assert(writes==1&&!absent);
 writes=0;absent=false;fail_at=2;
 assert(BQ76922_EnterShutdown(&d)==BQ76922_IO_ERROR);assert(writes==2&&absent);
}
'''
with tempfile.TemporaryDirectory() as t:
 c=Path(t)/'test.c';c.write_text(h);exe=Path(t)/'test'
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('BMS shutdown sequence: PASS')
