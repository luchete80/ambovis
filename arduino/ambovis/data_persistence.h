//
// Created by Mirian Quinteros on 11/09/2022.
//

#ifndef AMBOVIS_DATA_PERSISTENCE_H
#define AMBOVIS_DATA_PERSISTENCE_H

#include <EEPROM.h>

typedef struct system_configuration {
    unsigned long last_cycle;
    bool filter;
    bool autopid;
    int alarm_vt;
} SystemConfiguration_t;

SystemConfiguration_t read_memory();
void write_memory(SystemConfiguration_t data);

#endif //AMBOVIS_DATA_PERSISTENCE_H
