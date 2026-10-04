#include "sense_check.h"

#include <stdio.h>

static int g_failures;

static void ExpectEq(unsigned got, unsigned want, const char *msg)
{
    if (got != want) {
        g_failures++;
        printf("FAIL: %s (got %u want %u)\n", msg, got, want);
    }
}

int main(void)
{
    /* 1.000 V at the pin, 3.000 V reference → 12.000 V at the divider input. */
    ExpectEq(Sense_CountsToMv(1365U, 3000U), 12000U,
             "1365 counts is 12.000 V through the 12:1 divider");
    ExpectEq(Sense_CountsToMv(0U, 3000U), 0U, "zero counts is 0 V");
    ExpectEq(Sense_CountsToMv(4095U, 3000U), 36000U,
             "full scale is 36.000 V");

    ExpectEq(Sense_Classify(12000U, 11500U, 200U), SENSE_OK,
             "positive near Vout and negative near ground is OK");
    ExpectEq(Sense_Classify(12000U, 100U, 100U), SENSE_OPEN,
             "both leads near ground are an open positive");
    ExpectEq(Sense_Classify(12000U, 200U, 11800U), SENSE_REVERSED,
             "negative on Vout and positive near ground is swapped");
    ExpectEq(Sense_Classify(12000U, 11800U, 11500U), SENSE_N_ON_POUT,
             "both leads on the positive output");
    ExpectEq(Sense_Classify(12000U, 11500U, 5000U), SENSE_N_HIGH,
             "positive matches, negative sits well above ground");
    ExpectEq(Sense_Classify(12000U, 6000U, 200U), SENSE_P_MISMATCH,
             "positive is neither at Vout nor open");
    ExpectEq(Sense_Classify(12000U, 200U, 5000U), SENSE_OPEN_P,
             "positive open, negative on neither rail");
    ExpectEq(Sense_Classify(1000U, 1000U, 0U), SENSE_NOT_READY,
             "below 2 V the test does not judge the leads");
    ExpectEq(Sense_Classify(2000U, 1000U, 0U), SENSE_NOT_READY,
             "2 V local, remote +1 V and 0 V overlap and must not pass");
    ExpectEq(Sense_Classify(3000U, 1500U, 0U), SENSE_NOT_READY,
             "at 3 V the 1.5 V window still touches both bands");
    ExpectEq(Sense_Classify(3001U, 3001U, 0U), SENSE_OK,
             "just above 3 V the bands separate and a matched pair passes");
    ExpectEq(Sense_Classify(24000U, 20000U, 1500U), SENSE_OK,
             "a few volts of cable drop still passes");

    if (g_failures != 0) {
        printf("%d failure(s)\n", g_failures);
        return 1;
    }
    printf("test_sense_check: PASS\n");
    return 0;
}
