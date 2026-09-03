#include "usb_c_auto_policy.h"

#include <stdio.h>

int main(void)
{
    uint8_t reset_byte = 0x52U; /* TRM reset 00015052h, process both, 3 A */
    uint8_t hold_source;
    uint8_t hold_sink;
    uint8_t auto_default;
    bool accept_src = false;
    bool accept_snk = true;

    if (UsbC_AutoShouldSink(0U) ||
        UsbC_AutoShouldSink(5000U)) {
        fprintf(stderr, "5 V-only partner must be sourced\n");
        return 1;
    }
    if (!UsbC_AutoShouldSink(5001U) ||
        !UsbC_AutoShouldSink(9000U) ||
        !UsbC_AutoShouldSink(20000U)) {
        fprintf(stderr, ">5 V partner must be sunk\n");
        return 1;
    }

    UsbC_AutoDefaultSwapAccept(&accept_src, &accept_snk);
    if (!accept_src || accept_snk) {
        fprintf(stderr, "AUTO must allow SWSr and reject swap-to-sink\n");
        return 1;
    }
    auto_default = UsbC_PortControlSwapBits(reset_byte, accept_src, accept_snk);
    if (auto_default != 0x42U) {
        fprintf(stderr, "AUTO default expected 0x42, got 0x%02X\n", auto_default);
        return 1;
    }
    if (!UsbC_AutoRejectsIncomingSwapToSink(auto_default)) {
        fprintf(stderr, "iPad/Mac PR_SWAP to sink would still be accepted\n");
        return 1;
    }

    /* Charger 20 V: we are already sink, lock by also rejecting swap-to-source. */
    hold_sink = UsbC_PortControlSwapBits(auto_default, false, true);
    if (UsbC_AutoShouldSink(20000U) == false) {
        fprintf(stderr, "20 V charger must be sunk\n");
        return 1;
    }
    if ((hold_sink & USB_C_PC_PROCESS_TO_SOURCE) != 0U) {
        fprintf(stderr, "locked sink still accepts swap to source\n");
        return 1;
    }

    /* 5 V gadget won CC: SWSr once, hold already rejects reverse swap. */
    if (UsbC_AutoShouldSink(5000U)) {
        fprintf(stderr, "5 V gadget must be sourced\n");
        return 1;
    }
    if (!UsbC_AutoRejectsIncomingSwapToSink(
            UsbC_PortControlSwapBits(reset_byte, true, false))) {
        fprintf(stderr, "after SWSr Apple could still steal source\n");
        return 1;
    }

    hold_source = UsbC_PortControlSwapBits(reset_byte, true, false);
    if (hold_source != 0x42U) {
        fprintf(stderr, "HOLD_SOURCE expected 0x42, got 0x%02X\n", hold_source);
        return 1;
    }
    hold_sink = UsbC_PortControlSwapBits(reset_byte, false, true);
    if (hold_sink != 0x12U) {
        fprintf(stderr, "HOLD_SINK expected 0x12, got 0x%02X\n", hold_sink);
        return 1;
    }

    /* Type-C current nibble must survive a hold/open cycle. */
    if ((UsbC_PortControlSwapBits(0x53U, true, false) & 0x0FU) != 0x03U) {
        fprintf(stderr, "TypeC current nibble was overwritten\n");
        return 1;
    }

    return 0;
}
