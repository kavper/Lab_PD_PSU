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

#endif /* LDO_CTRL_POLICY_H */
