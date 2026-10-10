#ifndef LDO_CTRL_POLICY_H
#define LDO_CTRL_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * First-start SET may wait for Vout≈0 (G0 historically NACK OUT ON
 * with VOUT_NOT_ZERO). A live SET while already RUNNING must not drop
 * the rail with OUT OFF — the H7 slider sends SET on every tick.
 */
static inline bool Ldo_SetAckRequiresVoutZero(bool already_running)
{
    return !already_running;
}

/*
 * Live SET is not time-coalesced. One G4→G0 transaction may be in flight.
 * A second complete V+I overwrites the single pending slot.
 * The same SEQ is a replay and must not be applied again.
 */
typedef struct {
    bool valid;
    uint32_t mv;
    uint32_t ma;
    uint8_t seq;
} LdoPendingSet;

static inline void Ldo_PendingSetStore(LdoPendingSet *pending, uint32_t mv,
                                       uint32_t ma, uint8_t seq)
{
    pending->valid = true;
    pending->mv = mv;
    pending->ma = ma;
    pending->seq = seq;
}

static inline bool Ldo_SameSeqReplay(bool have_result, uint8_t stored_seq,
                                     uint8_t seq)
{
    return have_result && (stored_seq == seq);
}

/*
 * One host result is in the slot HostLink drains on the 5 ms METER.
 * A second completion must not replace it. The next G0 SET that belongs
 * to H7 waits until that slot has been taken, so a lost ACK can still
 * be replayed from the same SEQ.
 */
static inline bool Ldo_TryPostHostResult(bool *ready, uint8_t *seq,
                                        uint8_t *ack, uint8_t *reason,
                                        uint8_t new_seq, bool new_ack,
                                        uint8_t new_reason)
{
    if ((ready == 0) || (seq == 0) || (ack == 0) || (reason == 0)) {
        return false;
    }
    if (*ready) {
        return false;
    }
    *ready = true;
    *seq = new_seq;
    *ack = new_ack ? 1U : 0U;
    *reason = new_reason;
    return true;
}

static inline bool Ldo_MayDispatchHostSet(bool result_waiting)
{
    return !result_waiting;
}

#define LDO_G0_NACK_UNSAFE           4U
#define LDO_G0_FAULT_POWER_KILL      (1UL << 2)
#define LDO_G0_FAULT_VIN_LOW         (1UL << 3)
#define LDO_G0_FAULT_MEAS_LOST       (1UL << 8)

typedef enum {
    LDO_ON_RETRY = 0,
    LDO_ON_WAIT_PERMIT,
    LDO_ON_WAIT_VIN,
    LDO_ON_WAIT_ZERO,
    LDO_ON_FAIL
} LdoOnReject_t;

/*
 * Binary NACK carries a reason code, not an ASCII fault name.
 * UNSAFE is interpreted from the latest G0 telemetry. A stale frame
 * is not a reason to guess. MEAS_LOST must not be treated as VIN_LOW
 * just because the cleared samples read as 0 V.
 */
static inline LdoOnReject_t Ldo_RejectOutOn(uint8_t nack_reason, bool tlm_fresh,
                                           uint32_t fault_flags, uint8_t kill,
                                           uint32_t vin_mv, uint32_t vout_mv,
                                           uint32_t vin_min_mv,
                                           uint32_t vout_zero_mv,
                                           unsigned retries, unsigned retry_max)
{
    bool give_up = (retries + 1U) >= retry_max;

    if (!tlm_fresh || (nack_reason != LDO_G0_NACK_UNSAFE) ||
        ((fault_flags & LDO_G0_FAULT_MEAS_LOST) != 0U)) {
        return give_up ? LDO_ON_FAIL : LDO_ON_RETRY;
    }
    if ((kill != 0U) || ((fault_flags & LDO_G0_FAULT_POWER_KILL) != 0U)) {
        return LDO_ON_WAIT_PERMIT;
    }
    if (((fault_flags & LDO_G0_FAULT_VIN_LOW) != 0U) || (vin_mv < vin_min_mv)) {
        return LDO_ON_WAIT_VIN;
    }
    if (vout_mv > vout_zero_mv) {
        return LDO_ON_WAIT_ZERO;
    }
    return give_up ? LDO_ON_FAIL : LDO_ON_RETRY;
}

/* Analogue KILL removes drive immediately. G4 confirms a raw telemetry
 * indication for the same 50 ms as G0 before latching the whole PSU off. */
#define LDO_KILL_CONFIRM_MS 50U
typedef struct { bool active; uint32_t since_ms; } LdoKillConfirm;
static inline bool Ldo_KillConfirmed(LdoKillConfirm *state, bool asserted,
                                     uint32_t now_ms)
{
    if (!asserted) { state->active = false; return false; }
    if (!state->active) { state->active = true; state->since_ms = now_ms; }
    return (uint32_t)(now_ms - state->since_ms) >= LDO_KILL_CONFIRM_MS;
}
#endif /* LDO_CTRL_POLICY_H */
