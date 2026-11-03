#include <assert.h>
#include <stdio.h>
#include "bms_fet_guard.h"
int main(void) {
 assert(BmsFetGuard_StartComplete(5,0,0,0));
 assert(BmsFetGuard_StartComplete(4,8,0,0));
 assert(!BmsFetGuard_StartComplete(4,0,0,0));
 assert(!BmsFetGuard_StartComplete(4,12,0,0));
 assert(!BmsFetGuard_StartComplete(4,8,16,0));
 assert(!BmsFetGuard_StartComplete(4,8,0,16));
 assert(!BmsFetGuard_StartComplete(0,8,0,0));
 assert(!BmsFetGuard_StartComplete(12,8,0,0));
 BmsFetGuard g={0};
 assert(BmsFetGuard_AllowStart(&g));g.start_attempted=true;
 BmsFetGuard_Observe(&g,8U,0U);
 assert(!BmsFetGuard_Expired(&g,499U));
 BmsFetGuard_Observe(&g,5U,500U);
 assert(!BmsFetGuard_Expired(&g,2000U));
 assert(!BmsFetGuard_AllowStart(&g));assert(g.latched);
 g=(BmsFetGuard){0};
 BmsFetGuard_Observe(&g,8U,UINT32_MAX-500U);
 assert(!BmsFetGuard_Expired(&g,498U));
 assert(BmsFetGuard_Expired(&g,499U));
 BmsFetGuard_Observe(&g,5U,500U);
 assert(BmsFetGuard_Expired(&g,501U));
 assert(!BmsFetGuard_AllowStart(&g));
 g=(BmsFetGuard){0};
 BmsFetGuard_Observe(&g,13U,100U);
 assert(BmsFetGuard_Expired(&g,1100U));
 puts("PASS: PDSG completion, timeout, tick wrap, no repeated host starts or auto recovery");
}
