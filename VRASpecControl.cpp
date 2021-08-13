#include "VRASpecControl.h"

VRASpecControl::VRASpecControl(
    UUID *p_uuid,
    EventQueue *p_eq,
    StateChain *p_stateChain,
    I2C *p_i2c,
    VRAStorage *storage,
    VRAEnvironmentControl *envCtl,
    VRATvocControl *tvocCtl) : BLEService("specCtrl", p_uuid, p_eq, p_stateChain),
                               eq(p_eq),
                               // storage(p_storage),
                               // settings(p_settings),
                               i2c(p_i2c),
                               storage(storage),
                               envCtl(envCtl),
                               tvocCtl(tvocCtl)
{
    this->interval = new IntervalEvent(this->eq, 10000, callback(this, &VRASpecControl::getAdc));
    this->adc0 = new ADS1115(this->i2c, 0x49); //AIN0/AIN1: O3, AIN2/AIN3: NO2
    this->adc1 = new ADS1115(this->i2c, 0x48); //AIN0/AIN1: CO

    this->adc0Int = new InterruptIn(INT_ADC0, PullUp);
    this->adc0Int->fall(callback(this, &VRASpecControl::adc0FallISR));
    this->adc0Int->disable_irq();

    this->adc1Int = new InterruptIn(INT_ADC1, PullUp);
    this->adc1Int->fall(callback(this, &VRASpecControl::adc1FallISR));
    this->adc1Int->disable_irq();

    this->favO3 = new FloatingAverage(6);
    this->favCO = new FloatingAverage(6);
    this->favNO2 = new FloatingAverage(6);

    this->sgO3 = new SavLayFilter(&(this->vO3), 0, 25);   //9);
    this->sgCO = new SavLayFilter(&(this->vCO), 0, 25);   //9);
    this->sgNO2 = new SavLayFilter(&(this->vNO2), 0, 25); //9);
}

void VRASpecControl::init()
{
    printf("[specCtrl] init\r\n");
    this->adc0->setDataRate(this->dataRate);
    this->adc0->setVoltageRange(this->vrO3);
    this->adc0->enableConvReadyPin(1);

    this->adc1->setDataRate(this->dataRate);
    this->adc1->setVoltageRange(this->vrCO);
    this->adc1->enableConvReadyPin(1);

    this->adc0Int->enable_irq();
    this->adc1Int->enable_irq();
}

void VRASpecControl::adc0FallISR()
{
    this->eq->call(callback(this, &VRASpecControl::adc0Ready));
}

void VRASpecControl::adc1FallISR()
{
    this->eq->call(callback(this, &VRASpecControl::adc1Ready));
}

void VRASpecControl::initCharacteristics()
{
    printf("[specCtrl] init Characteristics\r\n");
    // this->addCharacteristic(
    //     new BLENotifyCharacteristic(
    //         (uint16_t)VRASpecControl::Characteristics::Spec,
    //         12, //size
    //         this->eq,
    //         10000,  //interval
    //         10000,  //min
    //         600000, //max
    //         callback(this, &VRASpecControl::getAdc)));

    this->addCharacteristic(
        new BLECharacteristic(
            (uint16_t)VRASpecControl::Characteristics::Spec,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY,
            12));

    this->addCharacteristic(
        new BLECharacteristic(
            (uint16_t)VRASpecControl::Characteristics::RawSpec,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY,
            12));

    this->addCharacteristic(
        new BLECharacteristic(
            (uint16_t)VRASpecControl::Characteristics::ZeroVoltage,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY,
            12));

    this->addCharacteristic(
        new BLECharacteristic(
            (uint16_t)VRASpecControl::Characteristics::AvgSpec,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY,
            12));
}

void VRASpecControl::pastBleInit()
{
    printf("[specCtrl] pastBleInit\r\n");

    float zeroVoltages[3];
    this->storage->get("zero", (void *)zeroVoltages, 12);

    if (zeroVoltages[0] > 0 && zeroVoltages[0] < 0.00004)
    {
        this->zeroCO = zeroVoltages[0];
        printf("[specCtrl] using CO zero from storage %d\r\n", (int)(this->zeroCO * 1E6));
    }
    else
    {
        zeroVoltages[0] = this->zeroCO;
    }

    if (zeroVoltages[1] > 0.04 && zeroVoltages[1] < 0.05)
    {
        this->zeroNO2 = zeroVoltages[1];
        printf("[specCtrl] using NO2 zero from storage %d\r\n", (int)(this->zeroNO2 * 1E6));
    }
    else
    {
        zeroVoltages[1] = this->zeroNO2;
    }

    if (zeroVoltages[2] > 0.007 && zeroVoltages[2] < 0.008)
    {
        this->zeroO3 = zeroVoltages[2];
        printf("[specCtrl] using O3 zero from storage %d\r\n", (int)(this->zeroO3 * 1E6));
    }
    else
    {
        zeroVoltages[2] = this->zeroO3;
    }

    this->favCO->flush(this->zeroCO);
    this->favNO2->flush(this->zeroNO2);
    this->favO3->flush(this->zeroO3);

    this->setGatt((uint16_t)VRASpecControl::Characteristics::Spec, zeroVoltages, 3);
    this->setGatt((uint16_t)VRASpecControl::Characteristics::RawSpec, zeroVoltages, 3);
    this->setGatt((uint16_t)VRASpecControl::Characteristics::ZeroVoltage, zeroVoltages, 3);
    this->setGatt((uint16_t)VRASpecControl::Characteristics::AvgSpec, zeroVoltages, 3);

    this->interval->start();
}

void VRASpecControl::onStateOff()
{
    printf("[specCtrl] off\r\n");
}

void VRASpecControl::onStateStandby()
{
    printf("[specCtrl] standby\r\n");
}

void VRASpecControl::onStateOn()
{
    printf("[specCtrl] on\r\n");
}

void VRASpecControl::getAdc()
{
    printf("[specCtrl] get adc\r\n");

    //this->adc0->setVoltageRange(this->vrO3);//VR
    this->adc0->startConversation(chan_0_1);
    this->adc1->startConversation(chan_0_1);
}

void VRASpecControl::getAdc2()
{
    // this->adc0->setVoltageRange(this->vrNO2);//VR
    this->adc0->startConversation(chan_2_3);
}

void VRASpecControl::adc0Ready()
{
    float val = this->adc0->getLastConversionResults_V();

    // adsVR_t vr = this->adc0->setVoltageRangeByVal(val);//VR

    // printf(">> [%d] %d adc0 ready %d\r\n", this->convCnt0, (uint32_t)time(NULL), (int)(val * 1E5));
    this->convCnt0++;
    if (this->convCnt0 <= this->maxConv)
    {

        this->vO3sum += (double)val;
        // printf("vr O3 %04X\r\n", vr);

        // printf("O3 %d %d\r\n", this->testCnt, (int)(val * 1E5));
        // this->testCnt++;

        if (this->convCnt0 < this->maxConv)
        {
            // this->vrO3 = vr;//VR
            this->adc0->startConversation(chan_0_1);
        }
        else //switch condition
        {
            this->eq->call_in(1000, callback(this, &VRASpecControl::getAdc2)); //delay next channel
            // this->adc0->setVoltageRange(this->vrNO2);
            // this->adc0->startConversation(chan_2_3);
        }
    }
    else if (this->convCnt0 <= 2 * this->maxConv)
    {
        this->vNO2sum += (double)val;
        // printf("vr NO2 %04X\r\n", vr);
        // printf("NO2 %d %d\r\n", this->testCnt, (int)(val * 1E5));
        // this->testCnt++;

        if (this->convCnt0 < 2 * this->maxConv)
        {
            this->adc0->startConversation(chan_2_3);
            // this->vrNO2 = vr;//VR
        }
        else //stop condition
        {
            // printf("STOP COND\r\n");
            // this->adc0->setVoltageRange(this->vrO3);//VR
            this->testCnt = 0;

            this->vO3 = this->vO3sum / (double)this->maxConv;
            this->vO3sum = 0;

            this->vNO2 = this->vNO2sum / (double)this->maxConv;
            this->vNO2sum = 0;

            this->convCnt0 = 0;

            this->eq->call(callback(this, &VRASpecControl::writeSpecToGatt));
        }
    }
    else
    {
    }
}

void VRASpecControl::adc1Ready()
{
    float val = this->adc1->getLastConversionResults_V();
    // adsVR_t vr = this->adc1->setVoltageRangeByVal(val);//VR

    // printf(">> [%d] %d adc1 ready %d\r\n", this->convCnt1, (uint32_t)time(NULL), (int)(val * 1E5));
    this->convCnt1++;
    if (this->convCnt1 < this->maxConv)
    {
        // printf("vr CO %04X\r\n", vr);
        this->vCOsum += (double)val;
        this->adc1->startConversation(chan_0_1);
        // this->vrCO = vr;//VR
    }
    else
    {
        this->vCO = this->vCOsum / (double)this->maxConv;
        this->vCOsum = 0;

        this->convCnt1 = 0;
    }
}

void VRASpecControl::writeSpecToGatt()
{
    printf("V CO %d\r\n", (int)(this->vCO * 1E6));
    printf("V NO2 %d\r\n", (int)(this->vNO2 * 1E6));
    printf("V O3 %d\r\n", (int)(this->vO3 * 1E6));

    float rawSpec[3]{
        (float)this->vCO,
        (float)this->vNO2,
        (float)this->vO3};

    this->setGatt((uint16_t)VRASpecControl::Characteristics::RawSpec, rawSpec, 3);

    float smoothCO = (float)this->sgCO->Compute();
    float smoothNO2 = (float)this->sgNO2->Compute();
    float smoothO3 = (float)this->sgO3->Compute();

    float spec[3]{
        smoothCO,
        smoothNO2,
        smoothO3};

    printf("sV CO %d\r\n", (int)(smoothCO * 1E6));
    printf("sV NO2 %d\r\n", (int)(smoothNO2 * 1E6));
    printf("sV O3 %d\r\n", (int)(smoothO3 * 1E6));

    if ((uint32_t)time(NULL) > 600) // ggf um 120sec erhoehen
    {
        this->favCO->add(smoothCO);
        this->favNO2->add(smoothNO2);
        this->favO3->add(smoothO3);

        float avgCO = this->favCO->get();
        float avgNO2 = this->favNO2->get();
        float avgO3 = this->favO3->get();

        float avgSpec[3]{
            avgCO,
            avgNO2,
            avgO3};

        printf("aV CO %d\r\n", (int)(avgCO * 1E6));
        printf("aV NO2 %d\r\n", (int)(avgNO2 * 1E6));
        printf("aV O3 %d\r\n", (int)(avgO3 * 1E6));

        this->setGatt((uint16_t)VRASpecControl::Characteristics::Spec, spec, 3);
        this->setGatt((uint16_t)VRASpecControl::Characteristics::AvgSpec, avgSpec, 3);
        this->eq->call(callback(this, &VRASpecControl::checkZeroVoltages), avgCO, avgNO2, avgO3);
    }
}

void VRASpecControl::checkZeroVoltages(float vCO, float vNO2, float vO3)
{
    uint32_t time_s = (uint32_t)time(NULL);
    float temperature = this->envCtl->getTemperature();
    float airQuality = this->tvocCtl->getTvoc();

    printf("[specCtrl] zero voltage check. time %d temperature %d airQuali %d\r\n", time_s, (int)(temperature), (int)(airQuality));

    if (time_s > 600 && temperature > 17.0 && temperature < 23.0 && airQuality < 3.0) //grundvoraussetzung
    {
        bool change = false;
        if (vCO > 0 && vCO < this->zeroCO)
        {
            change = true;
            this->zeroCO = vCO;
            printf("[specCtrl] new CO zero voltage: %d\r\n", (int)(this->zeroCO * 1E6));
        }
        if (vNO2 > 0.04 && vNO2 < this->zeroNO2)
        {
            change = true;
            this->zeroNO2 = vNO2;
            printf("[specCtrl] new NO2 zero voltage: %d\r\n", (int)(this->zeroNO2 * 1E6));
        }
        if (vO3 > 0.007 && vO3 < this->zeroO3)
        {
            change = true;
            this->zeroO3 = vO3;
            printf("[specCtrl] new O3 zero voltage: %d\r\n", (int)(this->zeroO3 * 1E6));
        }

        if (change)
        {
            printf("[specCtrl] post new zero voltage\r\n");
            float zeroVoltages[3]{
                this->zeroCO,
                this->zeroNO2,
                this->zeroO3};
            this->setGatt((uint16_t)VRASpecControl::Characteristics::ZeroVoltage, zeroVoltages, 3);
            this->storage->set("zero", (void *)zeroVoltages, 12);
        }
    }
}