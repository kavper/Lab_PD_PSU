#ifndef LDO_TLM_LAYOUT_H
#define LDO_TLM_LAYOUT_H

#include <stdint.h>

/*
 * G0 telemetry 0x80, 68 bytes. Same order as
 * ldo_controller docs/G4_G0_UART_PROTOCOL_V2.md.
 * Temperatures forwarded to H7 are the int16 °C×10 block, not the
 * ADC counts that sit in front of it. 253 = 25.3 °C.
 */

#define LDO_TLM_BYTES                68U
#define LDO_TLM_DAC_CV_MV            12U
#define LDO_TLM_DAC_CC_MV            16U
#define LDO_TLM_BLEED                34U
#define LDO_TLM_PGOOD                35U
#define LDO_TLM_TEMP_RAW             40U /* 4 x u16 ADC counts */
#define LDO_TLM_TEMP_FILTERED        48U /* 4 x u16 ADC counts */
#define LDO_TLM_TEMP_CENTI           56U /* 4 x i16, °C x 10; INT16_MIN invalid */
#define LDO_TLM_FAN                  64U

static inline int16_t LdoTlm_TempCenti(const uint8_t *payload, uint8_t index)
{
    const uint8_t *field = &payload[LDO_TLM_TEMP_CENTI + (uint16_t)index * 2U];

    return (int16_t)((uint16_t)field[0] | ((uint16_t)field[1] << 8));
}

#endif /* LDO_TLM_LAYOUT_H */
