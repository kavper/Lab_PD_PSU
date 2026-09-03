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

/* Programmed TPS25751 EEPROM PORT_CONTROL (0x29) payload. Byte 0 reset
 * 0x52 accepts PR_SWAP both ways; AUTO patches that to 0x42. */
#define USB_C_EEPROM_PORT_CONTROL0   0x52U
#define USB_C_EEPROM_PORT_CONTROL1   0x30U
#define USB_C_EEPROM_PORT_CONTROL2   0x81U
#define USB_C_EEPROM_PORT_CONTROL3   0xDAU

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

typedef enum {
    USB_C_AUTO_HOLD_SINK = 0,
    USB_C_AUTO_HOLD_SOURCE,
    USB_C_AUTO_SWAP_TO_SOURCE
} UsbC_AutoAction_t;

/* Single AUTO role action. Never SWSk.
 * Source: hold (charge the gadget). Sink + >5 V: hold (take power).
 * Sink + 5 V Source PDOs: SWSr, capped. Sink + no PDOs: hold (5 V supply). */
static inline UsbC_AutoAction_t UsbC_AutoAction(
    bool we_are_source,
    bool partner_source_caps_current,
    uint32_t partner_source_max_mv,
    uint8_t swap_attempts,
    uint8_t max_attempts)
{
    if (we_are_source) {
        return USB_C_AUTO_HOLD_SOURCE;
    }
    if (UsbC_AutoShouldSink(partner_source_max_mv)) {
        return USB_C_AUTO_HOLD_SINK;
    }
    if (partner_source_caps_current && (swap_attempts < max_attempts)) {
        return USB_C_AUTO_SWAP_TO_SOURCE;
    }
    return USB_C_AUTO_HOLD_SINK;
}

static inline bool UsbC_AutoDesiredSink(bool we_are_source,
                                        uint32_t partner_source_max_mv)
{
    if (we_are_source) {
        return false;
    }
    return UsbC_AutoShouldSink(partner_source_max_mv);
}

static inline bool UsbC_AutoStaySink(bool we_are_source,
                                     bool partner_source_caps_current,
                                     uint32_t partner_source_max_mv)
{
    return UsbC_AutoAction(we_are_source, partner_source_caps_current,
                           partner_source_max_mv, 0U, 1U) ==
           USB_C_AUTO_HOLD_SINK;
}

static inline bool UsbC_AutoNeedSwapToSource(bool we_are_source,
                                             bool partner_source_caps_current,
                                             uint32_t partner_source_max_mv,
                                             uint8_t swap_attempts,
                                             uint8_t max_attempts)
{
    return UsbC_AutoAction(we_are_source, partner_source_caps_current,
                           partner_source_max_mv, swap_attempts,
                           max_attempts) == USB_C_AUTO_SWAP_TO_SOURCE;
}

/* RX_SOURCE_CAPS is only trustworthy while we are sink. Keep the interrupt
 * pending until STATUS catches up; never read it while sourcing. */
static inline bool UsbC_AutoShouldReadSourceCaps(bool pending,
                                                 bool we_are_sink)
{
    return pending && we_are_sink;
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

/* Seed PORT_CONTROL from the EEPROM image with AUTO swap-to-sink off so
 * the first APP I2C write can reject Apple PR_SWAP without a prior read. */
static inline void UsbC_AutoLoadDefaultPortControl(
    uint8_t port_control[4])
{
    bool accept_to_source = false;
    bool accept_to_sink = true;

    port_control[0] = USB_C_EEPROM_PORT_CONTROL0;
    port_control[1] = USB_C_EEPROM_PORT_CONTROL1;
    port_control[2] = USB_C_EEPROM_PORT_CONTROL2;
    port_control[3] = USB_C_EEPROM_PORT_CONTROL3;
    UsbC_AutoDefaultSwapAccept(&accept_to_source, &accept_to_sink);
    port_control[0] = UsbC_PortControlSwapBits(port_control[0],
                                               accept_to_source,
                                               accept_to_sink);
}

#endif
