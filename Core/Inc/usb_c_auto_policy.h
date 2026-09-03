#ifndef USB_C_AUTO_POLICY_H
#define USB_C_AUTO_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/* PORT_CONTROL (0x29) byte 0: PR_SWAP bits from TPS25751 TRM table 3-15. */
#define USB_C_PC_PROCESS_TO_SINK     0x10U
#define USB_C_PC_INITIATE_TO_SINK    0x20U
#define USB_C_PC_PROCESS_TO_SOURCE   0x40U
#define USB_C_PC_INITIATE_TO_SOURCE  0x80U
#define USB_C_PC_PR_SWAP_MASK        0xF0U

/* AUTO: sink from a partner that can source more than USB 5 V; otherwise source. */
static inline bool UsbC_AutoShouldSink(uint32_t partner_source_max_mv)
{
    return partner_source_max_mv > 5000U;
}

/* AUTO never accepts PR_SWAP to sink. That swap is what Apple DRP uses to
 * steal VBUS after we have already started charging a gadget. */
static inline void UsbC_AutoDefaultSwapAccept(bool *accept_to_source,
                                              bool *accept_to_sink)
{
    *accept_to_source = true;
    *accept_to_sink = false;
}

/* Partner asked us to become sink after we are sourcing. Must be rejected. */
static inline bool UsbC_AutoRejectsIncomingSwapToSink(uint8_t port_control_byte0)
{
    return (port_control_byte0 & USB_C_PC_PROCESS_TO_SINK) == 0U;
}

/* Desired sink only when we are already sink AND partner advertises >5 V.
 * If we are already source, stay source (stale Source PDOs must not SWSk). */
static inline bool UsbC_AutoDesiredSink(bool we_are_source,
                                        uint32_t partner_source_max_mv)
{
    if (we_are_source) {
        return false;
    }
    return UsbC_AutoShouldSink(partner_source_max_mv);
}

/* Stay sink when the partner is a >5 V PD source, a dedicated 5 V supply
 * (no Source PDOs), or caps have not arrived yet. Never SWSk. */
static inline bool UsbC_AutoStaySink(bool we_are_source,
                                     bool partner_source_caps_current,
                                     uint32_t partner_source_max_mv)
{
    if (we_are_source) {
        return false;
    }
    if (UsbC_AutoShouldSink(partner_source_max_mv)) {
        return true;
    }
    return !partner_source_caps_current;
}

/* Sink on a partner that advertised 5 V-only Source PDOs: SWSr, capped.
 * Unknown/missing PDOs are not treated as a gadget (that would SWSr a
 * 5 V wall wart). Never SWSk. */
static inline bool UsbC_AutoNeedSwapToSource(bool we_are_source,
                                             bool partner_source_caps_current,
                                             uint32_t partner_source_max_mv,
                                             uint8_t swap_attempts,
                                             uint8_t max_attempts)
{
    if (UsbC_AutoStaySink(we_are_source,
                          partner_source_caps_current,
                          partner_source_max_mv)) {
        return false;
    }
    if (we_are_source) {
        return false;
    }
    return swap_attempts < max_attempts;
}

/* Clear both Initiate bits. Keep TypeC Current in the low nibble. */
static inline uint8_t UsbC_PortControlSwapBits(uint8_t current,
                                               bool accept_to_source,
                                               bool accept_to_sink)
{
    uint8_t bits = 0U;

    if (accept_to_source) {
        bits |= USB_C_PC_PROCESS_TO_SOURCE;
    }
    if (accept_to_sink) {
        bits |= USB_C_PC_PROCESS_TO_SINK;
    }
    return (uint8_t)((current & (uint8_t)~USB_C_PC_PR_SWAP_MASK) | bits);
}

#endif
