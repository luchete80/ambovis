//
// Created by Mirian Quinteros on 11/09/2022.
//

#ifndef AMBOVIS_DATA_PERSISTENCE_H
#define AMBOVIS_DATA_PERSISTENCE_H

#include <EEPROM.h>
#include <Arduino.h>

typedef struct system_configuration {
    unsigned long last_cycle;
    byte filter;
    byte autopid;
    byte alarm_vt;
} SystemConfiguration_t;

SystemConfiguration_t read_memory();
void write_memory(SystemConfiguration_t data);

#endif //AMBOVIS_DATA_PERSISTENCE_H
