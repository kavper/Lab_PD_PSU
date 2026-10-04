#ifndef PREREG_REQUEST_H
#define PREREG_REQUEST_H

#include <stdbool.h>

/*
 * Voltage the G4 DCDC should aim at.
 *
 * CV, output wanted but still off, and a missing CC request all stay at
 * cv_floor_v (Vset + dropout, already not below the VIN floor).
 *
 * Confirmed CC: honor G0's lower vpre (measured Vout + dropout). Do not
 * lift that request back to Vset + dropout — that is the extra voltage
 * across the LDO. Still refuse anything under vin_floor_v. A collapsed
 * Vout used to be followed all the way to vpre_min and the rail tripped
 * VIN_LOW on the way down.
 */
static inline float Prereg_SelectRequestV(bool output_on,
                                          bool output_wanted,
                                          bool in_cc,
                                          bool vpre_present,
                                          float vpre_v,
                                          float vout_v,
                                          float cv_floor_v,
                                          float margin_v,
                                          float vin_floor_v,
                                          float vpre_min_v,
                                          float vpre_max_v)
{
    float request;

    if (!output_on) {
        request = output_wanted ? cv_floor_v : vpre_min_v;
    } else if (in_cc) {
        request = vpre_present ? vpre_v : (vout_v + margin_v);
        if (request < vin_floor_v) {
            request = vin_floor_v;
        }
        if (request > cv_floor_v) {
            request = cv_floor_v;
        }
    } else if (vpre_present) {
        request = vpre_v;
        if (request < cv_floor_v) {
            request = cv_floor_v;
        }
    } else {
        request = cv_floor_v;
    }

    if (request < vpre_min_v) {
        request = vpre_min_v;
    }
    if (request > vpre_max_v) {
        request = vpre_max_v;
    }
    return request;
}

#endif /* PREREG_REQUEST_H */
