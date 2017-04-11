#ifndef FREESRP_UTIL_H
#define FREESRP_UTIL_H

#include "ad9364/ad9361.h"

void print_ensm_state(struct ad9361_rf_phy *phy);
void ctrl_set_value(uint8_t gpio, uint8_t value);
void rx_band_select(uint32_t port);
void tx_band_select(uint32_t port);

#endif
