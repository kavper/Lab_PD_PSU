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

    /* Charger 20 V: lock by rejecting both incoming PR_SWAPs. */
    hold_sink = UsbC_PortControlSwapBits(auto_default, false, false);
    if (UsbC_AutoShouldSink(20000U) == false) {
        fprintf(stderr, "20 V charger must be sunk\n");
        return 1;
    }
    if ((hold_sink & USB_C_PC_PROCESS_TO_SOURCE) != 0U ||
        (hold_sink & USB_C_PC_PROCESS_TO_SINK) != 0U) {
        fprintf(stderr, "locked sink still accepts a PR_SWAP\n");
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

    if (UsbC_AutoDesiredSink(true, 20000U)) {
        fprintf(stderr, "stale 20 V PDOs must not SWSk while sourcing a gadget\n");
        return 1;
    }
    if (!UsbC_AutoDesiredSink(false, 20000U)) {
        fprintf(stderr, "sink + 20 V charger must stay sink\n");
        return 1;
    }
    if (UsbC_AutoDesiredSink(false, 5000U)) {
        fprintf(stderr, "sink + 5 V partner must become source\n");
        return 1;
    }
    if (UsbC_AutoNeedSwapToSource(true, 5000U, 0U, 2U) ||
        UsbC_AutoNeedSwapToSource(false, 20000U, 0U, 2U) ||
        UsbC_AutoNeedSwapToSource(false, 5000U, 2U, 2U)) {
        fprintf(stderr, "swap-to-source guard failed\n");
        return 1;
    }
    if (!UsbC_AutoNeedSwapToSource(false, 5000U, 0U, 2U) ||
        !UsbC_AutoNeedSwapToSource(false, 0U, 1U, 2U)) {
        fprintf(stderr, "must SWSr a 5 V-only sink partner\n");
        return 1;
    }

    /* Mac refuses SWSr: at most two attempts, then lock. Never SWSk. */
    {
        uint8_t attempts = 0U;
        unsigned swsr = 0U;
        unsigned i;

        for (i = 0U; i < 8U; ++i) {
            if (UsbC_AutoNeedSwapToSource(false, 5000U, attempts, 2U)) {
                ++swsr;
                ++attempts;
            }
        }
        if (swsr != 2U) {
            fprintf(stderr, "Mac 5 V loop must stop after 2 SWSr, got %u\n", swsr);
            return 1;
        }
        if (UsbC_AutoDesiredSink(true, 9000U) ||
            UsbC_AutoNeedSwapToSource(true, 9000U, 0U, 2U)) {
            fprintf(stderr, "ping-pong: source gadget must not SWSk\n");
            return 1;
        }
    }
    if ((UsbC_PortControlSwapBits(0x53U, true, false) & 0x0FU) != 0x03U) {
        fprintf(stderr, "TypeC current nibble was overwritten\n");
        return 1;
    }

    return 0;
}
