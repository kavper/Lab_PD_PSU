#include "usb_c_auto_policy.h"

#include <stdio.h>

int main(void)
{
    uint8_t reset_byte = 0x52U; /* TRM reset 00015052h, process both, 3 A */
    uint8_t hold_source;
    uint8_t hold_sink;
    uint8_t open_auto;

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

    open_auto = UsbC_PortControlSwapBits(reset_byte, true, true);
    if (open_auto != reset_byte) {
        fprintf(stderr, "OPEN should keep 0x52, got 0x%02X\n", open_auto);
        return 1;
    }
    if ((open_auto & USB_C_PC_INITIATE_TO_SOURCE) != 0U ||
        (open_auto & USB_C_PC_INITIATE_TO_SINK) != 0U) {
        fprintf(stderr, "Initiate bits must stay clear\n");
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
