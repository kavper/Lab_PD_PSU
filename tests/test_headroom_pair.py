"""Exercise actual G0/G4 request policies with the production configuration."""
from pathlib import Path
import re, subprocess, sys, tempfile
root = Path(__file__).resolve().parents[1]
g0 = Path(sys.argv[1])
def value(path, name):
    text = path.read_text()
    return float(re.search(r'^#define\s+' + name + r'\s+([0-9.]+)', text, re.M)[1])
a = g0/'Core/Inc/app_config.h'
b = root/'Core/Inc/board_rev.h'
margin = value(a, 'VPRE_MARGIN_MV')
minimum = value(a, 'VPRE_MIN_MV')
floor = value(a, 'VPRE_VIN_FLOOR_MV')
maximum = value(a, 'VPRE_MAX_MV')
for name, v in [('BOARD_VPRE_MARGIN_V', margin), ('BOARD_VPRE_MIN_V', minimum),
                ('BOARD_VPRE_VIN_FLOOR_V', floor), ('BOARD_VPRE_MAX_V', maximum)]:
    assert value(b, name)*1000 == v, name
assert margin == minimum == floor == 1500
assert value(a, 'CONSOLE_MINIMUM_VIN_MV') == value(root/'Core/Src/ldo_link.c', 'LDO_VIN_MIN_MV') < minimum
code = f'''#include "vpre_request.h"
#include "prereg_request.h"
#include <assert.h>
#include <math.h>
int main(void) {{
 for (unsigned set=0; set<=30000; set+=1000) {{
  for (unsigned out=0; out<=set; out+=100) {{
   for (unsigned cc=0; cc<2; ++cc) {{
    unsigned want=(cc ? out : set)+1500;
    unsigned request=Vpre_RequestMv(true,cc,set,out,{margin:.0f}U,{floor:.0f}U,{minimum:.0f}U,{maximum:.0f}U);
    assert(request==want);
    float selected=Prereg_SelectRequestV(true,true,cc,true,request/1000.f,out/1000.f,
                                          (set+1500)/1000.f,1.5f,1.5f,1.5f,36.f);
    assert(fabsf(selected-want/1000.f)<0.001f);
   }}
  }}
 }}
 assert(Vpre_RequestMv(true,true,12000,0,1500,1500,1500,36000)==1500);
 return 0;
}}'''
with tempfile.TemporaryDirectory() as d:
    src=Path(d)/'test.c'; exe=Path(d)/'test'; src.write_text(code)
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(g0/'Core/Inc'),
                    '-I'+str(root/'Core/Inc'),str(src),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS: production G0/G4 limits match; CV set+1.5 V and CC output+1.5 V across 0-30 V')
