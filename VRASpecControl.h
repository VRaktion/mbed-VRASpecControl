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
#include "BLENotifyCharacteristic.h"

class VRASpecControl : public BLEService
{
public:
    enum class Characteristics: uint16_t
    {
        Spec = 0xFF00
    };

    enum class SpecSensors
    {
        CO = chan_0,
        NO2 = chan_1,
        O3 = chan_2
    };

    VRASpecControl(UUID *p_uuid, EventQueue *p_eq, StateChain *p_stateChain, I2C *p_i2c);

    void init();
    void initCharacteristics();
    void pastBleInit();

    void startSpecSensor(SpecSensors spec);
    void readSpecSensor(SpecSensors spec);
    
private:

    void onStateOff();
    void onStateStandby();
    void onStateOn();

    void getAdc();

    EventQueue *eq;
    // VRASettings *settings;
    // VRAStorage *storage;

    I2C *i2c;
    ADS1115 *adc;

    const adsVR_t voltageRange = VR_p_m_4_096V;
    const adsDR_t dataRate = ADS1115_DR_8SPS;
    int conversationDelay{0};

    float vO3{.0};
    float vCO{.0};
    float vNO2{.0};
};

#endif //