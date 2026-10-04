#include "ldo_ctrl_policy.h"

#include <stdio.h>

static int g_failures;

static void Expect(int cond, const char *msg)
{
    if (!cond) {
        g_failures++;
        printf("FAIL: %s\n", msg);
    }
}

int main(void)
{
    bool ready = false;
    uint8_t seq = 0U;
    uint8_t ack = 0U;
    uint8_t reason = 0U;
    LdoPendingSet pending;

    Expect(Ldo_TryPostHostResult(&ready, &seq, &ack, &reason, 7U, true, 0U),
           "the in-flight SET stores its ACK");
    Expect((seq == 7U) && (ack == 1U), "the stored result is SEQ 7 ACK");
    Expect(!Ldo_TryPostHostResult(&ready, &seq, &ack, &reason, 8U, true, 0U),
           "a second completion does not overwrite the undelivered ACK");
    Expect(seq == 7U, "SEQ 7 is still the result waiting for H7");
    Expect(!Ldo_MayDispatchHostSet(ready),
           "the pending SET stays queued until H7 takes the ACK");

    ready = false;
    Expect(Ldo_MayDispatchHostSet(ready),
           "after the take, the pending SET may start");
    Expect(Ldo_SameSeqReplay(true, 7U, 7U),
           "a retransmitted SEQ is the stored result, not a new SET");
    pending.valid = false;
    pending.seq = 0U;
    if (!Ldo_SameSeqReplay(true, 7U, 7U)) {
        Ldo_PendingSetStore(&pending, 5000U, 100U, 7U);
    }
    Expect(!pending.valid, "the replay does not apply the setpoint again");

    Expect(Ldo_RejectOutOn(LDO_G0_NACK_UNSAFE, true, LDO_G0_FAULT_POWER_KILL,
                           1U, 12000U, 0U, 4500U, 250U, 0U, 4U) ==
               LDO_ON_WAIT_PERMIT,
           "UNSAFE with kill waits for permit");
    Expect(Ldo_RejectOutOn(LDO_G0_NACK_UNSAFE, true, 0U, 0U, 12000U, 0U,
                           4500U, 250U, 0U, 4U) == LDO_ON_RETRY,
           "the same ON proceeds once kill has cleared");
    Expect(Ldo_RejectOutOn(LDO_G0_NACK_UNSAFE, true, LDO_G0_FAULT_VIN_LOW,
                           0U, 4000U, 0U, 4500U, 250U, 0U, 4U) ==
               LDO_ON_WAIT_VIN,
           "UNSAFE with VIN_LOW waits for the rail");
    Expect(Ldo_RejectOutOn(LDO_G0_NACK_UNSAFE, true, 0U, 0U, 12000U, 800U,
                           4500U, 250U, 0U, 4U) == LDO_ON_WAIT_ZERO,
           "UNSAFE with Vout still up waits for zero");
    Expect(Ldo_RejectOutOn(LDO_G0_NACK_UNSAFE, true, LDO_G0_FAULT_MEAS_LOST,
                           0U, 0U, 0U, 4500U, 250U, 0U, 4U) == LDO_ON_RETRY,
           "cleared samples are not treated as VIN_LOW");
    Expect(Ldo_RejectOutOn(LDO_G0_NACK_UNSAFE, false, LDO_G0_FAULT_POWER_KILL,
                           1U, 0U, 0U, 4500U, 250U, 3U, 4U) == LDO_ON_FAIL,
           "a stale frame is not a reason to chase kill");

    if (g_failures != 0) {
        printf("%d failure(s)\n", g_failures);
        return 1;
    }
    printf("test_host_set: PASS\n");
    return 0;
}
