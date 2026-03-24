#include "Settings.h"
#include <EEPROM.h>

NvmSettings Settings::s;

void Settings::begin() {
    EEPROM.begin(NVM_SIZE);
    if (!load()) {
        defaults();
        Serial.println("[NVM] No valid settings found — factory defaults applied");
        Serial.println("[NVM] Type 'save' to persist current settings");
    }
}

bool Settings::load() {
    NvmSettings tmp;
    EEPROM.get(NVM_ADDR, tmp);
    if (tmp.magic != NVM_MAGIC || tmp.version != NVM_VERSION) return false;
    s = tmp;
    return true;
}

void Settings::save() {
    s.magic   = NVM_MAGIC;
    s.version = NVM_VERSION;
    EEPROM.put(NVM_ADDR, s);
    EEPROM.commit();
}

void Settings::defaults() {
    s.magic    = NVM_MAGIC;
    s.version  = NVM_VERSION;
    s._pad[0]  = 0;
    s._pad[1]  = 0;
    s._pad[2]  = 0;

    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        NvmSettings::MotorSettings& ms = s.motor[m];
        ms._mpad[0]    = 0;
        ms._mpad[1]    = 0;
        ms._mpad[2]    = 0;

        ms.velKp       = 0.6f;
        ms.velKi       = 0.4f;
        ms.velKd       = 0.0f;
        ms.velAccel    = 200.0f;

        ms.posKp       = 3.0f;
        ms.posKi       = 0.0f;
        ms.posKd       = 0.0f;
        ms.traverseVel = 90.0f;
        ms.posPathMode = 1;       // POS_PATH_CONSTRAINED
        ms.posMin      = 0.0f;
        ms.posMax      = 355.0f;

        ms.zeroOffset  = 0.0f;
    }
}
