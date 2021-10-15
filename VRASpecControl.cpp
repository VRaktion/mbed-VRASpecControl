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
    this->interval = new IntervalEvent(this->eq, 15000, callback(this, &VRASpecControl::getAdcCh1));
    this->adc0 = new ADS1115(this->i2c, 0x49); //AIN0/AIN1: O3, AIN2/AIN3: NO2
    this->adc1 = new ADS1115(this->i2c, 0x48); //AIN0/AIN1: CO

    this->adc0Int = new InterruptIn(INT_ADC0, PullUp);
    this->adc0Int->fall(callback(this, &VRASpecControl::adc0FallISR));
    this->adc0Int->disable_irq();

    this->adc1Int = new InterruptIn(INT_ADC1, PullUp);
    this->adc1Int->fall(callback(this, &VRASpecControl::adc1FallISR));
    this->adc1Int->disable_irq();

    this->favCO = new FloatingAverage(AV_SIZE);
    this->favNO2 = new FloatingAverage(AV_SIZE);
    this->favO3 = new FloatingAverage(AV_SIZE);

    this->fav2O3 = new FloatingAverage(AV2_SIZE);
    this->fav2CO = new FloatingAverage(AV2_SIZE);
    this->fav2NO2 = new FloatingAverage(AV2_SIZE);

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
    //         callback(this, &VRASpecControl::getAdcCh1)));

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
    double zeroVoltagesShifted[3] = {
        (double)zeroVoltages[0] / MANTISSE_CO,
        (double)zeroVoltages[1] / MANTISSE_NO2,
        (double)zeroVoltages[2] / MANTISSE_O3};

    if (zeroVoltagesShifted[0] !=0 && zeroVoltagesShifted[0] < CALIB_START_CO && zeroVoltagesShifted[0] > CALIB_END_CO)
    {
        this->zeroCO = zeroVoltagesShifted[0];
        printf("[specCtrl] using CO zero from storage %d\r\n", (int)(this->zeroCO * 1E6));
    }
    else
    {
        zeroVoltages[0] = (float)(this->zeroCO * MANTISSE_CO);
        printf("[specCtrl] using default CO zero \r\n");
    }

    if (zeroVoltagesShifted[1] < CALIB_START_NO2 && zeroVoltagesShifted[1] > CALIB_END_NO2)
    {
        this->zeroNO2 = zeroVoltagesShifted[1];
        printf("[specCtrl] using NO2 zero from storage %d\r\n", (int)(this->zeroNO2 * 1E6));
    }
    else
    {
        zeroVoltages[1] = (float)(this->zeroNO2 * MANTISSE_NO2);
        printf("[specCtrl] using default NO2 zero \r\n");
    }

    if (zeroVoltagesShifted[2] < CALIB_START_O3 && zeroVoltagesShifted[2] > CALIB_END_O3)
    {
        this->zeroO3 = zeroVoltagesShifted[2];
        printf("[specCtrl] using O3 zero from storage %d\r\n", (int)(this->zeroO3 * 1E6));
    }
    else
    {
        zeroVoltages[2] = (float)(this->zeroO3 * MANTISSE_O3);
        printf("[specCtrl] using default O3 zero \r\n");
    }

    this->favCO->flush(this->zeroCO);
    this->favNO2->flush(this->zeroNO2);
    this->favO3->flush(this->zeroO3);

    this->fav2CO->flush(this->zeroCO);
    this->fav2NO2->flush(this->zeroNO2);
    this->fav2O3->flush(this->zeroO3);

    printf("[specCtrl] zero Gatt %d %d %d\r\n", (int)(zeroVoltages[0]), (int)(zeroVoltages[1]), (int)(zeroVoltages[2]));
    this->setGatt((uint16_t)VRASpecControl::Characteristics::ZeroVoltage, zeroVoltages, 3);

    zeroVoltages[0] = zeroVoltagesShifted[0];
    zeroVoltages[1] = zeroVoltagesShifted[1];
    zeroVoltages[2] = zeroVoltagesShifted[2];

    this->setGatt((uint16_t)VRASpecControl::Characteristics::Spec, zeroVoltages, 3);
    this->setGatt((uint16_t)VRASpecControl::Characteristics::RawSpec, zeroVoltages, 3);
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

void VRASpecControl::getAdcCh1()
{
    printf("[specCtrl] get adc\r\n");

    this->adc0->startConversation(chan_0_1);
    this->adc1->startConversation(chan_0_1);
}

void VRASpecControl::getAdcCh2()
{
    this->adc0->startConversation(chan_2_3);
}

void VRASpecControl::adc0Ready()
{
    if (this->tvocCtl->isMeassureBlocked()) //TVOC disturbs Signal!!
    {
        if (this->convCnt0 < this->maxConv)
        {
            this->eq->call_in(1000, callback(this, &VRASpecControl::getAdcCh1));
        }
        else
        {
            this->eq->call_in(1000, callback(this, &VRASpecControl::getAdcCh2));
        }

        return;
    }

    double val = this->adc0->getLastConversionResults_V();

    this->convCnt0++;
    if (this->convCnt0 <= this->maxConv)
    {
        this->favO3->add((double)val);

        if (this->convCnt0 < this->maxConv)
        {
            this->adc0->startConversation(chan_0_1);
        }
        else //switch condition
        {
            this->eq->call_in(1000, callback(this, &VRASpecControl::getAdcCh2)); //delay next channel
        }
    }
    else if (this->convCnt0 <= 2 * this->maxConv)
    {
        this->favNO2->add((double)val);

        if (this->convCnt0 < 2 * this->maxConv)
        {
            this->adc0->startConversation(chan_2_3);
        }
        else //stop condition
        {
            this->convCnt0 = 0;

            this->eq->call(callback(this, &VRASpecControl::writeSpecToGatt));
        }
    }
}

void VRASpecControl::adc1Ready()
{
    if (this->tvocCtl->isMeassureBlocked())
    {
        return;
    }

    double val = this->adc1->getLastConversionResults_V();

    this->convCnt1++;
    if (this->convCnt1 < this->maxConv)
    {
        this->favCO->add((double)val);
        this->adc1->startConversation(chan_0_1);
    }
    else
    {
        this->convCnt1 = 0;
    }
}

void VRASpecControl::writeSpecToGatt()
{
    this->vCO = this->favCO->get();
    this->vNO2 = this->favNO2->get();
    this->vO3 = this->favO3->get();

    printf("V CO %d\r\n", (int)(this->vCO * 1E6));
    printf("V NO2 %d\r\n", (int)(this->vNO2 * 1E6));
    printf("V O3 %d\r\n", (int)(this->vO3 * 1E6));

    float rawSpec[3]{
        (float)this->vCO,
        (float)this->vNO2,
        (float)this->vO3};

    // this->setGatt((uint16_t)VRASpecControl::Characteristics::RawSpec, rawSpec, 3);

    double smoothCO = this->sgCO->Compute();
    double smoothNO2 = this->sgNO2->Compute();
    double smoothO3 = this->sgO3->Compute();

    float spec[3]{
        (float)smoothCO,
        (float)smoothNO2,
        (float)smoothO3};

    printf("sV CO %d\r\n", (int)(smoothCO * 1E6));
    printf("sV NO2 %d\r\n", (int)(smoothNO2 * 1E6));
    printf("sV O3 %d\r\n", (int)(smoothO3 * 1E6));

    if ((uint32_t)time(NULL) > SPEC_INIT_TIME)
    {

        this->fav2CO->add(smoothCO);
        this->fav2NO2->add(smoothNO2);
        this->fav2O3->add(smoothO3);

        double avCO = this->fav2CO->get();
        double avNO2 = this->fav2NO2->get();
        double avO3 = this->fav2O3->get();

        printf("aV CO %d\r\n", (int)(avCO * 1E6));
        printf("aV NO2 %d\r\n", (int)(avNO2 * 1E6));
        printf("aV O3 %d\r\n", (int)(avO3 * 1E6));

        float avSpec[3]{
            (float)avCO,
            (float)avNO2,
            (float)avO3};

        this->setGatt((uint16_t)VRASpecControl::Characteristics::Spec, spec, 3);
        this->setGatt((uint16_t)VRASpecControl::Characteristics::AvgSpec, avSpec, 3);
        this->eq->call(callback(this, &VRASpecControl::checkZeroVoltages), avCO, avNO2, avO3);
    }
}

void VRASpecControl::checkZeroVoltages(double vCO, double vNO2, double vO3)
{
    uint32_t time_s = (uint32_t)time(NULL);
    float temperature = this->envCtl->getTemperature();
    float airQuality = this->tvocCtl->getTvoc();

    printf("[specCtrl] zero voltage check. time %d temperature %d airQuali %d\r\n", time_s, (int)(temperature), (int)(airQuality));

    if (time_s > AV_INIT_TIME && temperature > 18.0 && airQuality < 3.0 && this->battery > 5) //grundvoraussetzung  && temperature < 23.0 entfällt da tendenziell größer
    {
        bool change = false;
        if (vCO < this->zeroCO && vCO > CALIB_END_CO)
        {
            change = true;
            this->zeroCO = vCO;
            printf("[specCtrl] new CO zero voltage: %d\r\n", (int)(this->zeroCO * 1E6));
        }
        if (vNO2 < this->zeroNO2 && vNO2 > CALIB_END_NO2)
        {
            change = true;
            this->zeroNO2 = vNO2;
            printf("[specCtrl] new NO2 zero voltage: %d\r\n", (int)(this->zeroNO2 * 1E6));
        }
        if (vO3 < this->zeroO3 && vO3 > CALIB_END_O3)
        {
            change = true;
            this->zeroO3 = vO3;
            printf("[specCtrl] new O3 zero voltage: %d\r\n", (int)(this->zeroO3 * 1E6));
        }

        if (change)
        {
            this->eq->call(callback(this, &VRASpecControl::publishZeroVoltages));
            this->eq->call(callback(this, &VRASpecControl::saveZeroVoltages));
        }
    }
}

void VRASpecControl::saveZeroVoltages()
{
    printf("[specCtrl] save new zero voltage\r\n");
    float zeroVoltages[3]{
        (float)(this->zeroCO * MANTISSE_CO),
        (float)(this->zeroNO2 * MANTISSE_NO2),
        (float)(this->zeroO3 * MANTISSE_O3)};
    this->storage->set("zero", (void *)zeroVoltages, 12);
}

void VRASpecControl::publishZeroVoltages()
{
    printf("[specCtrl] publish new zero voltage\r\n");
    float zeroVoltages[3]{
        (float)(this->zeroCO * MANTISSE_CO),
        (float)(this->zeroNO2 * MANTISSE_NO2),
        (float)(this->zeroO3 * MANTISSE_O3)};
    this->setGatt((uint16_t)VRASpecControl::Characteristics::ZeroVoltage, zeroVoltages, 3); //TEST
}

void VRASpecControl::setBattery(uint8_t val){
    this->battery = val;
}