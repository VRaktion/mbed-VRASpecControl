#ifndef VRA_SPEC_CONTROL_H
#define VRA_SPEC_CONTROL_H

#include "BLEService.h"
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "mbed.h"

#include "ADS1x15.h"

// #include "VRASettings.h"
// #include "VRAStorage.h"
#include "IntervalEvent.h"



class VRASpecControl : public BLEService
{
public:
    enum Characteristics
    {
        SENSORDATA,
        COUNT
    };

    VRASpecControl(UUID *p_uuid, EventQueue *p_eq, StateChain *p_stateChain, I2C *p_i2c);

    void init();
    void initCharacteristics();
    void pastBleInit();
    
private:

    void onStateOff();
    void onStateStandby();
    void onStateOn();

    void getAdc();

    void toggleSpecSensorNotify(bool enable);

    EventQueue *eq;
    // VRASettings *settings;
    // VRAStorage *storage;

    IntervalEvent *getAdcEvent;
    I2C *i2c;
    ADS1115 *adc;
};

#endif //