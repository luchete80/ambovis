//
// Created by Mirian Quinteros on 11/09/2022.
//

#include "data_persistence.h"

SystemConfiguration_t read_memory() {
    SystemConfiguration_t data;
    int eeAddress = 0;
    EEPROM.get(0, data.last_cycle); eeAddress += sizeof(unsigned long);
    EEPROM.get(eeAddress, data.autopid);   eeAddress += sizeof(data.autopid);
    EEPROM.get(eeAddress, data.alarm_vt);  eeAddress += sizeof(data.alarm_vt);
    EEPROM.get(eeAddress, data.filter);
    return data;
}

void write_memory(SystemConfiguration_t data) {
    int eeAddress = 0;
    EEPROM.put(0, data.last_cycle); eeAddress += sizeof(unsigned long);
    EEPROM.put(eeAddress, data.autopid); eeAddress += sizeof(data.autopid);
    EEPROM.put(eeAddress, data.alarm_vt); eeAddress += sizeof(data.alarm_vt);
    EEPROM.put(eeAddress, data.filter);
}