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

#define CALIB_START_CO 0.001
#define CALIB_END_CO -0.01//
#define MANTISSE_CO 1E4//0//
#define CALIB_START_NO2 0.1
#define CALIB_END_NO2 0.001
#define MANTISSE_NO2 1E2
#define CALIB_START_O3 0.01
#define CALIB_END_O3 0.0001
#define MANTISSE_O3 1E3
#define SPEC_INIT_TIME 720//0//
#define AV_INIT_TIME SPEC_INIT_TIME + 300// ... min 500

#define AV_SIZE 1500
#define AV2_SIZE 100
#define MAX_CONV_SIZE 500

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

    void setBattery(uint8_t val);

private:
    void onStateOff();
    void onStateStandby();
    void onStateOn();

    void getAdcCh1();
    void getAdcCh2();

    void writeSpecToGatt();

    void adc0Ready();
    void adc1Ready();

    void checkZeroVoltages(double vCO, double vNO2, double vO3);

    void saveZeroVoltages();
    void publishZeroVoltages();

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
    static const adsDR_t dataRate{ADS1115_DR_128SPS}; //averaging inchip ADS1115_DR_8SPS

    int conversationDelay{0};

    double vO3{.5};
    double vCO{.5};
    double vNO2{.5};

    FloatingAverage *favO3;
    FloatingAverage *favCO;
    FloatingAverage *favNO2;

    FloatingAverage *fav2O3;
    FloatingAverage *fav2CO;
    FloatingAverage *fav2NO2;

    SavLayFilter *sgO3;
    SavLayFilter *sgCO;
    SavLayFilter *sgNO2;

    bool voltageRangeUpdated = false;

    const int maxConv{MAX_CONV_SIZE};//{300};//
    volatile int convCnt0{0};
    volatile int convCnt1{0};

    double zeroCO{CALIB_START_CO};
    double zeroNO2{CALIB_START_NO2};
    double zeroO3{CALIB_START_O3};

    int testCnt{0};

    volatile bool firstHit{false};

    VRAEnvironmentControl *envCtl;
    VRATvocControl *tvocCtl;
    VRAStorage *storage;

    uint8_t battery{100};
};

#endif //