#ifndef BLE_SUPPORT_C_H
#define BLE_SUPPORT_C_H

#include "ble_nus.h"
#include "ble_bas.h"

void ble_application_init();

ble_bas_t* GetBatteryInstance();

ble_nus_t* GetNusInstance();

uint16_t GetConnectionHandler();

bool IsRequestWeightData();


#endif  // BLE_SUPPORT_C_H