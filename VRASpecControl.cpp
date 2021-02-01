#include "VRASpecControl.h"

VRASpecControl::VRASpecControl(UUID *p_uuid, EventQueue *p_eq, StateChain *p_stateChain, I2C *p_i2c):
BLEService("specControl", p_uuid, (uint8_t)COUNT, p_eq, p_stateChain), 
eq(p_eq),
// storage(p_storage),
// settings(p_settings),
i2c(p_i2c)
{
    this->adc = new ADS1115(this->i2c, 0x49);

        this->getAdcEvent = new IntervalEvent(
        this->eq,
        10000,
        callback(this, &VRASpecControl::getAdc));
}

void VRASpecControl::init(){
    printf("[specCtrl] init\r\n");
}

void VRASpecControl::initCharacteristics(){
    printf("[specCtrl] init Characteristics\r\n");

        this->initCharacteristic(
        SENSORDATA, 0xFF00,
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY,
        12);
    this->toggleSpecSensorNotifyCb =
        callback(this, &VRASpecControl::toggleSpecSensorNotify);
    this->setNotifyRegisterCallback(SENSORDATA,
                                    &this->toggleSpecSensorNotifyCb);
    // this->initService();
}

void VRASpecControl::pastBleInit(){
    printf("[specCtrl] pastBleInit\r\n");
}

void VRASpecControl::toggleSpecSensorNotify(bool enable){
    if(enable){
        this->getAdcEvent->start();
    }else{
        this->getAdcEvent->stop();
    }
}

void VRASpecControl::onStateOff(){
    printf("[specCtrl] off\r\n");
}

void VRASpecControl::onStateStandby(){
    printf("[specCtrl] standby\r\n");
}

void VRASpecControl::onStateOn(){
    printf("[specCtrl] on\r\n");
}

void VRASpecControl::getAdc(){
    printf("[specCtrl] get adc\r\n");
}