#include "VRASpecControl.h"

VRASpecControl::VRASpecControl(UUID *p_uuid, EventQueue *p_eq, StateChain *p_stateChain, I2C *p_i2c):
BLEService("specCtrl", p_uuid, p_eq, p_stateChain),
eq(p_eq),
// storage(p_storage),
// settings(p_settings),
i2c(p_i2c)
{
    this->adc = new ADS1115(this->i2c, 0x49);
}

void VRASpecControl::init(){
    printf("[specCtrl] init\r\n");
    this->conversationDelay = this->adc->calcConversationDelay(this->dataRate)/1000;
}

void VRASpecControl::initCharacteristics(){
    printf("[specCtrl] init Characteristics\r\n");
    this->addCharacteristic(
        new BLENotifyCharacteristic(
            (uint16_t) VRASpecControl::Characteristics::Spec,
            12,//size
            this->eq,
            10000,//interval
            1000,//min
            600000,//max
            callback(this, &VRASpecControl::getAdc)
        )
    );
}

void VRASpecControl::pastBleInit(){
    printf("[specCtrl] pastBleInit\r\n");
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
    int delay = this->conversationDelay + 50;
    this->eq->call(callback(this, &VRASpecControl::startSpecSensor), VRASpecControl::SpecSensors::CO);
    this->eq->call_in(delay, callback(this, &VRASpecControl::startSpecSensor), VRASpecControl::SpecSensors::NO2);
    this->eq->call_in(2 * delay, callback(this, &VRASpecControl::startSpecSensor), VRASpecControl::SpecSensors::O3);
}

void VRASpecControl::startSpecSensor(VRASpecControl::SpecSensors spec){
    this->adc->startConversation((chan_t) spec, this->voltageRange, this->dataRate);
    int delay = this->conversationDelay + 10;
    this->eq->call_in(delay, callback(this, &VRASpecControl::readSpecSensor), spec);
}

void VRASpecControl::readSpecSensor(VRASpecControl::SpecSensors spec){
    switch(spec){
        case VRASpecControl::SpecSensors::CO://first
            this->vCO = this->adc->getLastConversionResults_V(this->voltageRange);
            printf("CO: %d\r\n", (int)(this->vCO*1E4));
        break;
        case VRASpecControl::SpecSensors::NO2://second
            this->vNO2 = this->adc->getLastConversionResults_V(this->voltageRange);
            printf("NO2: %d\r\n", (int)(this->vNO2*1E4));
        break;
        case VRASpecControl::SpecSensors::O3://last
            this->vO3 = this->adc->getLastConversionResults_V(this->voltageRange);
            float spec[3]{
                this->vCO,
                this->vNO2,
                this->vO3
            };
            this->setGatt((uint16_t) VRASpecControl::Characteristics::Spec, spec, 3);
            printf("O3: %d\r\n", (int)(this->vO3*1E4));
        break;
        default:
        break;
    }
}