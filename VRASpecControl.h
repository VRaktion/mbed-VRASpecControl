#ifndef VRA_SPEC_CONTROL_H
#define VRA_SPEC_CONTROL_H

#include "BLEService.h"

#include "VRAEnvironmentControl.h"
#include "VRATvocControl.h"
#include "VRAStorage.h"

// #include "VRASettings.h"
// #include "VRAStorage.h"
#include "IntervalEvent.h"
#include "BLENotifyCharacteristic.h"
#include "FloatingAverage.h"
#include "SavLayFilter.h"

#include "ADS1x15.h"
#include "mbed.h"

class VRASpecControl : public BLEService
{
public:
    enum class Characteristics : uint16_t
    {
        Spec = 0xFF00,
        RawSpec = 0xFF01,
        ZeroVoltage = 0xFF02,
        AvgSpec = 0xFF03
    };

    VRASpecControl(UUID *p_uuid, EventQueue *p_eq, StateChain *p_stateChain, I2C *p_i2c, VRAStorage *storage, VRAEnvironmentControl *envCtl, VRATvocControl *tvocCtl);

    void init();
    void initCharacteristics();
    void pastBleInit();

    void adc0RiseISR();
    void adc1RiseISR();

    void adc0FallISR();
    void adc1FallISR();

private:
    void onStateOff();
    void onStateStandby();
    void onStateOn();

    void getAdc();
    void getAdc2();

    void writeSpecToGatt();

    void adc0Ready();
    void adc1Ready();

    void checkZeroVoltages(float vCO, float vNO2, float vO3);

    EventQueue *eq;
    IntervalEvent *interval;
    // VRASettings *settings;
    // VRAStorage *storage;

    I2C *i2c;
    ADS1115 *adc0;
    ADS1115 *adc1;

    InterruptIn *adc0Int;
    InterruptIn *adc1Int;

    adsVR_t vrO3{VR_p_m_4_096V};
    adsVR_t vrNO2{VR_p_m_4_096V};
    adsVR_t vrCO{VR_p_m_4_096V};
    static const adsDR_t dataRate{ADS1115_DR_8SPS}; //averaging inchip ADS1115_DR_8SPS

    int conversationDelay{0};

    double vO3{.5};
    double vCO{.5};
    double vNO2{.5};

    FloatingAverage *favO3;
    FloatingAverage *favCO;
    FloatingAverage *favNO2;

    SavLayFilter *sgO3;
    SavLayFilter *sgCO;
    SavLayFilter *sgNO2;

    bool voltageRangeUpdated = false;

    const int maxConv{20};
    volatile int convCnt0{0};
    volatile int convCnt1{0};

    double vO3sum{.0};
    double vCOsum{.0};
    double vNO2sum{.0};

    float zeroCO{0.00004};
    float zeroNO2{0.05};
    float zeroO3{0.008};

    int testCnt{0};

    volatile bool firstHit{false};

    VRAEnvironmentControl *envCtl;
    VRATvocControl *tvocCtl;
    VRAStorage *storage;
};

#endif //