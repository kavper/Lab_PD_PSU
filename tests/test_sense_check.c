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

static void ExpectTrue(int cond, const char *msg)
{
    if (!cond) {
        g_failures++;
        printf("FAIL: %s\n", msg);
    }
}

static SenseInput Sample(uint32_t local_mv, uint32_t remote_p_mv,
                         uint32_t remote_n_mv, uint32_t setpoint_mv,
                         bool compare, bool wanted)
{
    SenseInput in;

    in.local_mv = local_mv;
    in.remote_p_mv = remote_p_mv;
    in.remote_n_mv = remote_n_mv;
    in.setpoint_mv = setpoint_mv;
    in.have_sample = true;
    in.compare_setpoint = compare;
    in.wanted = wanted;
    return in;
}

static SenseStep Feed(SenseGate *gate, SenseInput in)
{
    return Sense_GateStep(gate, &in);
}

static void CloseGood(SenseGate *gate)
{
    SenseStep step;
    unsigned i;

    Sense_GateRequest(gate, true);
    for (i = 0U; i < 3U; i++) {
        step = Feed(gate, Sample(12000U, 11600U, 300U, 12000U, false, true));
    }
    ExpectEq(step.code, SENSE_OK, "three stable drops close as OK");
    ExpectTrue(step.closed, "relay closes after three agreeing samples");
    ExpectTrue(!step.drop_permit, "a good close leaves PERMIT alone");
    ExpectTrue(!step.latched, "a good close does not latch");
}

int main(void)
{
    SenseLimits tight;
    SenseGate gate;
    SenseInput in;
    SenseStep step;
    uint32_t open_p;
    uint32_t min_local;
    unsigned i;

    ExpectEq(Sense_CountsToMv(1365U, 3000U), 12000U,
             "1365 counts is 12.000 V through the 12:1 divider");
    ExpectEq(Sense_CountsToMv(0U, 3000U), 0U, "zero counts is 0 V");
    ExpectEq(Sense_CountsToMv(4095U, 3000U), 36000U,
             "full scale is 36.000 V");

    min_local = Sense_MinLocalMv(SENSE_DROP_WIRE_MV);
    ExpectEq(min_local, 592U,
             "low-voltage threshold is (500 + 2*40) / (1 - 0.019)");
    ExpectTrue(Sense_PeriodSettled(),
               "100 ms period is many filter time constants");
    open_p = Sense_OpenPlusMv(12000U);
    ExpectEq(open_p, 11769U, "open plus at 12 V rests at 98.1% through R112");

    /* Correct connection: 400 mV on plus, 300 mV on minus, sum 700 mV. */
    ExpectEq(Sense_Classify(12000U, 11600U, 300U), SENSE_OK,
             "drops inside the bench limits pass");
    ExpectTrue(Sense_DpErrorMv(12000U, 11600U) != SENSE_DROP_WIRE_MV,
               "the ADC budget is a different number from the wire limit");

    /* 800 mV of positive drop is past 500 mV and still inside the error. */
    ExpectEq(Sense_Classify(12000U, 11200U, 200U), SENSE_OK,
             "800 mV at 12 V stays inside limit plus the channel error");

    /* 4 V of drop is outside the error. The old ±25% window used to pass this. */
    ExpectEq(Sense_Classify(24000U, 20000U, 200U), SENSE_DROP_P,
             "4 V of positive drop fails DP");

    ExpectEq(Sense_Classify(12000U, 200U, 11800U), SENSE_REVERSED,
             "swapped leads");
    ExpectEq(Sense_Classify(12000U, 11800U, 11700U), SENSE_N_ON_POUT,
             "both sense leads on the positive output");
    ExpectEq(Sense_Classify(12000U, 6000U, 6000U), SENSE_SHORT,
             "sense leads shorted together at mid-rail");

    /* Plus held at ground is a real DP fault. R112 would not read 0 V. */
    ExpectEq(Sense_Classify(12000U, 0U, 0U), SENSE_DROP_P,
             "plus at ground is an excessive drop");

    /* R112/R115 signature, before K1. This is not a detected break. */
    ExpectEq(Sense_Classify(12000U, open_p, 0U), SENSE_OK,
             "open plus via R112 and open minus via R115 read as a small drop");
    ExpectEq(Sense_Classify(24000U, Sense_OpenPlusMv(24000U), 0U), SENSE_OK,
             "the same open pair at 24 V is still inside the budget");

    ExpectEq(Sense_Classify(min_local - 1U, min_local - 1U, 0U), SENSE_NOT_READY,
             "below the budget threshold the leads are not judged");
    ExpectEq(Sense_Classify(min_local, min_local, 0U), SENSE_OK,
             "at the threshold a matched pair is OK");
    ExpectEq(Sense_Classify(min_local, 0U, 0U), SENSE_DROP_P,
             "at the threshold plus-at-ground is separable");
    ExpectEq(Sense_Classify(SENSE_FULL_SCALE_MV, 35000U, 0U), SENSE_LOCAL_HIGH,
             "a saturated local reading is not a voltage");

    tight = Sense_DefaultLimits();
    tight.sum_mv = 700U;
    ExpectEq(Sense_ClassifyLimits(2000U, 1400U, 500U, tight), SENSE_DROP_SUM,
             "600 mV and 500 mV pass the wire limit; 1100 mV fails a 700 mV sum");
    ExpectEq(Sense_Classify(2000U, 1400U, 500U), SENSE_OK,
             "the same pair passes the 1000 mV bench sum");

    /* Before close: R112 signature is allowed to close. It is not an open code. */
    Sense_GateInit(&gate);
    Sense_GateRequest(&gate, true);
    for (i = 0U; i < 3U; i++) {
        step = Feed(&gate, Sample(12000U, open_p, 0U, 12000U, false, true));
    }
    ExpectEq(step.code, SENSE_OK, "R112 signature before close stays OK");
    ExpectTrue(step.closed, "the undetectable open is not blocked before K1");
    ExpectTrue(!step.drop_permit, "that close does not drop PERMIT");

    Sense_GateInit(&gate);
    Sense_GateRequest(&gate, true);
    step = Feed(&gate, Sample(12000U, 200U, 11800U, 12000U, false, true));
    ExpectEq(step.code, SENSE_REVERSED, "swapped leads before close");
    ExpectTrue(!step.closed && !step.drop_permit, "swapped leads stay local");

    Sense_GateInit(&gate);
    Sense_GateRequest(&gate, true);
    step = Feed(&gate, Sample(12000U, 11800U, 11700U, 12000U, false, true));
    ExpectEq(step.code, SENSE_N_ON_POUT, "both leads on plus before close");
    ExpectTrue(!step.closed && !step.drop_permit, "both-on-plus stays local");

    Sense_GateInit(&gate);
    Sense_GateRequest(&gate, true);
    step = Feed(&gate, Sample(12000U, 6000U, 6000U, 12000U, false, true));
    ExpectEq(step.code, SENSE_SHORT, "shorted sense leads before close");
    ExpectTrue(!step.closed && !step.drop_permit, "a sense short stays local");

    /* Before close: plus at ground refuses the relay and leaves PERMIT up. */
    Sense_GateInit(&gate);
    Sense_GateRequest(&gate, true);
    step = Feed(&gate, Sample(12000U, 0U, 0U, 12000U, false, true));
    ExpectEq(step.code, SENSE_DROP_P, "plus at ground before close");
    ExpectTrue(!step.closed, "wiring fault keeps K1 in local");
    ExpectTrue(!step.drop_permit, "a fault before close does not drop PERMIT");
    ExpectTrue(!step.latched, "a fault before close does not latch");

    /* After close: R112 signature is still not a break and not critical.
     * In CV the 1.9% deficit also sits inside the VD error, so the setpoint
     * check does not turn it into a detected break either. */
    Sense_GateInit(&gate);
    CloseGood(&gate);
    for (i = 0U; i < 3U; i++) {
        step = Feed(&gate, Sample(12000U, open_p, 0U, 12000U, true, true));
    }
    ExpectEq(step.code, SENSE_OK, "R112 signature after close stays OK in CV");
    ExpectTrue(step.closed, "K1 stays closed on the R112/R115 reading");
    ExpectTrue(!step.drop_permit, "that reading does not drop PERMIT");

    /* After close: plus falls to ground. Critical. */
    step = Feed(&gate, Sample(12000U, 0U, 0U, 12000U, false, true));
    ExpectEq(step.code, SENSE_DROP_P, "plus at ground after close");
    ExpectTrue(!step.closed, "critical fault opens K1");
    ExpectTrue(step.drop_permit, "critical fault drops PERMIT");
    ExpectTrue(step.latched, "critical fault latches remote off");

    step = Feed(&gate, Sample(12000U, 11600U, 300U, 12000U, false, true));
    ExpectTrue(!step.closed, "a later OK sample does not reclose");
    ExpectTrue(step.latched, "the latch holds while remote stays requested");
    ExpectTrue(!step.drop_permit, "permit is dropped once");

    in = Sample(12000U, 11600U, 300U, 12000U, false, false);
    step = Feed(&gate, in);
    ExpectTrue(!step.latched, "REMOTE off clears the latch");
    ExpectTrue(!step.closed, "the relay stays local");
    Sense_GateRequest(&gate, true);
    for (i = 0U; i < 2U; i++) {
        step = Feed(&gate, Sample(12000U, 11600U, 300U, 12000U, false, true));
        ExpectTrue(!step.closed, "two new samples are not enough to reclose");
    }
    step = Feed(&gate, Sample(12000U, 11600U, 300U, 12000U, false, true));
    ExpectTrue(step.closed, "the third new sample may close again");

    /* Moving output: samples do not agree, so K1 stays open. */
    Sense_GateInit(&gate);
    Sense_GateRequest(&gate, true);
    step = Feed(&gate, Sample(12000U, 11600U, 300U, 12000U, false, true));
    step = Feed(&gate, Sample(14000U, 13600U, 300U, 12000U, false, true));
    step = Feed(&gate, Sample(16000U, 15600U, 300U, 12000U, false, true));
    ExpectTrue(!step.closed, "a 2 V step between samples does not close K1");

    /* Low voltage never closes. */
    Sense_GateInit(&gate);
    Sense_GateRequest(&gate, true);
    for (i = 0U; i < 3U; i++) {
        step = Feed(&gate, Sample(400U, 400U, 0U, 400U, false, true));
    }
    ExpectEq(step.code, SENSE_NOT_READY, "400 mV is below the budget threshold");
    ExpectTrue(!step.closed, "low voltage keeps the relay local");
    ExpectTrue(!step.drop_permit, "low voltage before close leaves PERMIT up");

    /* CC: VD far below the voltage setpoint is not a fault. */
    Sense_GateInit(&gate);
    CloseGood(&gate);
    for (i = 0U; i < 3U; i++) {
        step = Feed(&gate, Sample(8000U, 7600U, 200U, 12000U, false, true));
    }
    ExpectEq(step.code, SENSE_OK, "CC does not compare VD with the setpoint");
    ExpectTrue(step.closed, "CC with a legal drop stays remote");
    ExpectTrue(!step.drop_permit, "CC leaves PERMIT up");

    /* CV: the same VD miss trips on the third sample. */
    Sense_GateInit(&gate);
    CloseGood(&gate);
    step = Feed(&gate, Sample(8000U, 7600U, 200U, 12000U, true, true));
    ExpectEq(step.code, SENSE_CV_VD, "CV shows the setpoint miss immediately");
    ExpectTrue(step.closed, "one CV miss does not open yet");
    ExpectTrue(!step.drop_permit, "one CV miss does not drop PERMIT");
    step = Feed(&gate, Sample(8000U, 7600U, 200U, 12000U, true, true));
    ExpectTrue(step.closed, "two CV misses still wait");
    step = Feed(&gate, Sample(8000U, 7600U, 200U, 12000U, true, true));
    ExpectEq(step.code, SENSE_CV_VD, "third CV miss is the fault");
    ExpectTrue(!step.closed, "CV fault opens K1");
    ExpectTrue(step.drop_permit, "CV fault drops PERMIT");
    ExpectTrue(step.latched, "CV fault latches");

    /* ADC loss while still local. */
    Sense_GateInit(&gate);
    Sense_GateRequest(&gate, true);
    in = Sample(12000U, 11600U, 300U, 12000U, false, true);
    in.have_sample = false;
    step = Feed(&gate, in);
    ExpectEq(step.code, SENSE_NO_SAMPLE, "a missed conversion before close");
    ExpectTrue(!step.closed, "ADC loss before close keeps K1 local");
    ExpectTrue(!step.drop_permit, "ADC loss before close leaves PERMIT up");
    ExpectTrue(!step.latched, "ADC loss before close does not latch");

    /* ADC loss after close. */
    Sense_GateInit(&gate);
    CloseGood(&gate);
    in = Sample(0U, 0U, 0U, 0U, false, true);
    in.have_sample = false;
    step = Feed(&gate, in);
    ExpectEq(step.code, SENSE_NO_SAMPLE, "a missed conversion after close");
    ExpectTrue(!step.closed, "ADC loss after close opens K1");
    ExpectTrue(step.drop_permit, "ADC loss after close drops PERMIT");
    ExpectTrue(step.latched, "ADC loss after close latches");

    if (g_failures != 0) {
        printf("%d failure(s)\n", g_failures);
        return 1;
    }
    printf("test_sense_check: PASS\n");
    return 0;
}
