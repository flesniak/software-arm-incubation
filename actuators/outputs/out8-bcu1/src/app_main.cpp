/*
 *  app_main.cpp - The application's main.
 *
 *  Copyright (c) 2014 Stefan Taferner <stefan.taferner@gmx.at>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3 as
 *  published by the Free Software Foundation.
 */

#include <sblib/analog_pin.h>
#include "app_main.h"
#include "app_out8.h"

#ifdef BUSFAIL
#   include <sblib/math.h>
#endif

#ifndef BI_STABLE
#   include "outputs.h"
#else
#   include "outputsBiStable.h"
#endif

#ifdef BUSFAIL
#    include "bus_voltage.h"
#    include "app_nov_settings.h"
#endif

#ifdef DEBUG_SERIAL
#    include <sblib/serial.h>
#endif

#ifdef DEBUG
#    include <sblib/timer.h>
#endif

#ifdef BUSFAIL
    NonVolatileSetting AppNovSetting(0xEE00, 0x100);  // flash-storage for application relevant parameters
    ApplicationData AppData;
    AppCallback callback;
    AppUsrCallback usrCallback; ///\todo two callbacks? Optimize with busfail integration into the sblib.
    enum class BusfailStatus {running, failed, stopped, returned} busfailStatus = BusfailStatus::stopped;
#endif

APP_VERSION("O08.10  ", "5", "12"); // Don't forget to also change the build-variable sw_version

/**
 * Simple IO test
 */
void ioTest()
{
#ifdef IO_TEST

#   ifdef BI_STABLE
        const int relaySwitchms = ON_DELAY;  // see inc/outputsBiStable.h for more info
        // check, maybe we need to wait 4s, cause PIO_SDA is pulled-up to high on ARM-reset causing the relay coil to drain the bus.
        const int countOutputPins = sizeof(outputPins)/sizeof(outputPins[0]);
        for (unsigned int i = 0; i < countOutputPins; i++)
        {
            if (outputPins[i] == PIO_SDA)
            {
                delay(4000);
                break;
            }
        }
#   else
        const int relaySwitchms = PWM_TIMEOUT;  // see inc/outputs.h for more info
#   endif

    // setup LED's
    const int togglePausems = 250;
#   ifndef HAND_ACTUATION
        const int disablePins[9] = {PIN_LT1, PIN_LT2, PIN_LT3, PIN_LT4, PIN_LT5, PIN_LT6, PIN_LT7, PIN_LT8, PIN_LT9};
        for (unsigned int i = 0; i < sizeof(disablePins)/sizeof(disablePins[0]); i++)
        {
            pinMode(disablePins[i], INPUT);
        }
#   endif /* HAND_ACTUATION */

    // all relay-pins off
    for (unsigned int i = 0; i < sizeof(outputPins)/sizeof(outputPins[0]); i++)
    {
        digitalWrite(outputPins[i], 0);
        pinMode(outputPins[i], OUTPUT);
    }

    // initialize relays for ioTest
    relays.begin(0x00, 0x00, NO_OF_CHANNELS);
#   ifdef HAND_ACTUATION
        HandActuation handActTestIO = HandActuation(&handPins[0], NO_OF_HAND_PINS, READBACK_PIN, BLINK_TIME);
        relays.setHandActuation(&handActTestIO);
#   endif
    relays.updateOutputs();
    delay(relaySwitchms);
    relays.checkPWM();
    delay(togglePausems);

    // toggle every channel & hand actuation LED with a little delay
    for (unsigned int i = 0; i < relays.channelCount(); i++)
    {
        relays.updateChannel(i, 1);     // relay high/set
        relays.updateOutput(i);         // also sets hand actuation led
        delay(relaySwitchms);
        relays.checkPWM();

        delay(togglePausems);

        relays.updateChannel(i, 0);     // relay low/reset
        relays.updateOutput(i);         // also sets hand actuation led
        delay(relaySwitchms);
        relays.checkPWM();
        delay(togglePausems);
    }
#endif /* IO_TEST */
}

void initSerial()
{
#ifdef DEBUG_SERIAL
    serial.setRxPin(APP_OUT8X_PIN_DEBUG_RX);
    serial.setTxPin(APP_OUT8X_PIN_DEBUG_TX);
    serial.begin(115200);
    serial.println("out8 serial debug started");
#endif
}

void printSerialBusVoltage(const unsigned int onEveryTickMs)
{
#if defined(DEBUG_SERIAL) && defined(BUSFAIL)
    if (millis() % onEveryTickMs == 0)
    {
        unsigned int valueAD = busVoltageMonitor.valueBusVoltageAD();
        unsigned int valuemV = callback.convertADmV(valueAD);
        unsigned int valueADConvertedTwice = callback.convertmVAD(valuemV);
        int diff = int(valueAD - valueADConvertedTwice);
        serial.print("AD;");
        serial.print(valueAD);
        serial.print(";mV;");
        serial.print(valuemV);
        serial.print(";diff;");
        serial.print(diff);
        serial.println();
        delay(1); // makes sure that printSerialBusVoltage(...) is not called twice on the same SysTick
    }
#endif
}

void startBusVoltageMonitoring()
{
#ifdef BUSFAIL
    static Timeout flashTimeout;
    if (busVoltageMonitor.setup(VBUS_AD_PIN, VBUS_AD_CHANNEL, VBUS_ADC_SAMPLE_FREQ,
                                VBUS_THRESHOLD_FAILED, VBUS_THRESHOLD_RETURN,
                                VBUS_VOLTAGE_FAILTIME_MS, VBUS_VOLTAGE_RETURNTIME_MS,
                                &timer32_0, 0, &callback))
    {
        busVoltageMonitor.enable();
        while (busVoltageMonitor.busFailed())
        {
            delay(1);
            if ((flashTimeout.expired() || flashTimeout.stopped()))
            {
                bool state = digitalRead(APP_OUT8X_PIN_INFO);
                digitalWrite(APP_OUT8X_PIN_INFO, !state);
                flashTimeout.start(state ? 250 : 250);
            }
#           ifdef DEBUG
                digitalWrite(APP_OUT8X_PIN_RUN, !digitalRead(APP_OUT8X_PIN_RUN));
#           endif
            printSerialBusVoltage(100);
        }
#       ifdef DEBUG
            digitalWrite(APP_OUT8X_PIN_RUN, 1);
#       endif
    }
#endif
}

bool recallAppData()
{
    bool result = true;
#ifdef BUSFAIL
    result = AppNovSetting.RecallAppData((unsigned char*)&AppData, sizeof(ApplicationData)); // load custom application settings
    if (!result)
    {
#   ifdef DEBUG
        digitalWrite(APP_OUT8X_PIN_INFO, 1);
        delay(1000);
        digitalWrite(APP_OUT8X_PIN_INFO, 0);
#   endif
    }
#endif
    return result;
}

// configure watchdog to trigger after ~3 seconds for the default WDTOSCCTRL setting of 0xA0
void enableWatchdog()
{
    LPC_WDT->TC = 1050000 * 3;
    LPC_WDT->MOD = 0b011;
}

void feedWatchdog()
{
    LPC_WDT->FEED = 0xAA;
    LPC_WDT->FEED = 0x55;
}

/**
 * Initialize the application.
 */
BcuBase* setup()
{
    // first set pin mode for Info & Run LED
    pinMode(APP_OUT8X_PIN_INFO, OUTPUT | OPEN_DRAIN); // this also sets pin to high/true
    pinMode(APP_OUT8X_PIN_RUN, OUTPUT | OPEN_DRAIN); // this also sets pin to high/true
    // then set values
    digitalWrite(APP_OUT8X_PIN_INFO, 0);
#ifdef DEBUG
    digitalWrite(APP_OUT8X_PIN_RUN, 1);
#endif

    // Configure the output pins
    relays.setupOutputs(&outputPins[0], NO_OF_OUTPUTS);

    initSerial();

    ioTest();
    bcu.begin(MANUFACTURER, DEVICETYPE, APPVERSION);
#ifdef BUSFAIL
    bcu.setUsrCallback((UsrCallback *)&usrCallback);
    startBusVoltageMonitoring(); // enable bus voltage monitoring
#endif

#ifdef DEBUG_SERIAL
    int physicalAddress = bcu.ownAddress();
    serial.print("physical address: ", (physicalAddress >> 12) & 0x0F, DEC);
    serial.print(".", (physicalAddress >> 8) & 0x0F, DEC);
    serial.println(".", physicalAddress & 0xFF, DEC);
#endif

    // load previous relay state, reset to 0 in case of crc mismatch
    if (!recallAppData()) {
#ifdef BUSFAIL
        AppData.relaysstate = 0;
#endif
    }

#ifndef BI_STABLE
#   ifdef ZERO_DETECT
       pinInterruptMode(PIO_SDA, INTERRUPT_EDGE_FALLING | INTERRUPT_ENABLED);
       enableInterrupt(EINT0_IRQn);
       pinEnableInterrupt(PIO_SDA);
#   endif
#endif

#ifdef BUSFAIL
    initApplication(AppData.relaysstate);
    busfailStatus = BusfailStatus::running;
    startBusVoltageMonitoring(); // needs to be called again, because Release version is using analog_pin.h functions from sblib which break our ADC Interrupts
#else
    initApplication();
#endif

#ifdef WATCHDOG
    enableWatchdog();
#endif

    return (&bcu);
}

#ifdef BUSFAIL
void handleBusfailAction();
#endif

/**
 * The main processing loop. Will be called by the Selfbus sblib main().
 */
void loop(void)
{
#ifdef WATCHDOG
    feedWatchdog();
#endif

    int objno;
    // Handle updated communication objects
    while ((objno = bcu.comObjects->nextUpdatedObject()) >= 0)
    {
        objectUpdated(objno);
    }
    // check if any of the timeouts for an output has expire and react on them
    checkTimeouts();

    // Sleep up to 1 millisecond if there is nothing to do
    waitForInterrupt();
    printSerialBusVoltage(500);

#ifdef BUSFAIL
    handleBusfailAction();
#endif

#ifdef DEBUG
    static Timeout flashTimeout;
    if (flashTimeout.expired() || flashTimeout.stopped())
    {
        bool state = digitalRead(APP_OUT8X_PIN_INFO);
        digitalWrite(APP_OUT8X_PIN_INFO, !state);
        flashTimeout.start(state ? 100 : 900);
    }
#endif
}

/**
 * Will be called by the Selfbus sblib main(), while no application is loaded.
 * In case we have a HAND_ACTUATION all LED's will blink at 2*BLINK_TIME (~1Hz) to indicate this state
 */
void loop_noapp(void)
{
#ifdef WATCHDOG
    feedWatchdog();
#endif

#if defined(IO_TEST) && defined(HAND_ACTUATION)
    if (!bcu.programmingMode())
    {
        HandActuation::testIO(&handPins[0], NO_OF_HAND_PINS, BLINK_TIME);
    }
#endif
    waitForInterrupt();

#ifdef BUSFAIL
    printSerialBusVoltage(500);
    handleBusfailAction();
#endif

#ifdef DEBUG
    static Timeout flashTimeout;
    if (flashTimeout.expired() || flashTimeout.stopped())
    {
        bool state = digitalRead(APP_OUT8X_PIN_INFO);
        digitalWrite(APP_OUT8X_PIN_INFO, !state);
        flashTimeout.start(state ? 250 : 250);
    }
#endif
}

void ResetDefaultApplicationData()
{
#ifdef BUSFAIL
    AppData.relaysstate = 0x00;
#endif
}

#ifdef BUSFAIL
bool saveRelayState()
{
    AppData.relaysstate = getRelaysState();
    return AppNovSetting.StoreApplData((unsigned char*)&AppData, sizeof(ApplicationData));
}

void handleBusfailAction()
{
    switch (busfailStatus) {
        case BusfailStatus::failed:
            #ifdef DEBUG_SERIAL
                serial.println("BusVoltageFail");
            #endif
            // write application settings to flash
            digitalWrite(APP_OUT8X_PIN_INFO, !saveRelayState());
            stopApplication();
            busfailStatus = BusfailStatus::stopped;
            break;
        case BusfailStatus::returned:
            #ifdef DEBUG_SERIAL
                serial.println("BusVoltageReturn");
            #endif
            //restore application settings
            if (!recallAppData()) // load custom application settings
            {
                // load default values
                ResetDefaultApplicationData();
            }
            bcu.begin(MANUFACTURER, DEVICETYPE, APPVERSION);
            initApplication(AppData.relaysstate);
            busfailStatus = BusfailStatus::running;
            break;
        default:
            break;
    }
}

void AppCallback::BusVoltageFail()
{
    pinMode(APP_OUT8X_PIN_INFO, OUTPUT | OPEN_DRAIN); // even in non DEBUG flash Info LED to display app data storing
    digitalWrite(APP_OUT8X_PIN_INFO, 1);

    busfailStatus = BusfailStatus::failed;

#ifdef DEBUG
    digitalWrite(APP_OUT8X_PIN_RUN, 1); // switch RUN-LED off, to save some power
#endif
}

void AppCallback::BusVoltageReturn()
{
#ifdef DEBUG
    digitalWrite(APP_OUT8X_PIN_RUN, 0); // switch RUN-LED ON
#endif
    busfailStatus = BusfailStatus::returned;
}

int AppCallback::convertADmV(int valueAD)
{
#ifdef APP_OUT_8X_16A_BISTAB_4MU_KICAD
    // good approximation between 15 & 30V for the 4TE-ARM controller
    // using 43k / 6k2 and 2+3 bridged on SJ1
    if (valueAD > 2056)
        return 33070;
    else if (valueAD < 252)
        return 0;

    const float a = 0.0;
    const float b = 0.00199302;
    const float c = 5.70314758;
    const float d = 12204.62226368;
#else
    // good approximation between 10 & 30V for the 4TE-ARM controller
    if (valueAD > 2150)
        return 30000;
    else if (valueAD < 872)
        return 0;

    const float a = 0.00000857674926702488f;
    const float b = -0.0310784307851376f;
    const float c = 47.7234335386816f;
    const float d = -14253.9303808124f;
#endif
    float valueADSquared = sq(valueAD);
    return a*valueADSquared*valueAD + // a*x^3
           b*valueADSquared +         // b*x^2
           c*valueAD +                // c*x
           d;                         // d
    /*
     *  4TE ARM-Controller coefficients found with following measurements:
     *  ---------------------
     *  | Bus mV  ADC-Value |
     *  ---------------------
     *  |  9542    872      |
     *  | 10043    920      |
     *  | 11047   1017      |
     *  | 12008   1108      |
     *  | 13013   1202      |
     *  | 14012   1293      |
     *  | 15044   1382      |
     *  | 16024   1465      |
     *  | 17024   1545      |
     *  | 18012   1615      |
     *  | 19031   1686      |
     *  | 20044   1749      |
     *  | 20959   1801      |
     *  | 22003   1854      |
     *  | 23001   1901      |
     *  | 23987   1945      |
     *  | 24979   1985      |
     *  | 25999   2022      |
     *  | 26978   2055      |
     *  | 28006   2087      |
     *  | 29009   2115      |
     *  | 29873   2140      |
     *  | 30184   2150      |
     *  ---------------------
     *
    */
}

int AppCallback::convertmVAD(int valuemV)
{
#ifdef APP_OUT_8X_16A_BISTAB_4MU_KICAD
    if (valuemV >= 33070)
        return 2056;
    else if (valuemV < 13360)
        return 0;

    const float a = -1.71740561e-06;
    const float b = 0.17427461;
    const float c = -1798.06373933;
#else
    // good approximation between 10 & 30V for the 4TE-ARM controller
    if (valuemV >= 30184)
        return 2150;
    else if (valuemV < 9542)
        return 0;

    const float a = -0.00000214162532145905f;
    const float b = 0.146795202310839f;
    const float c = -339.582791686125f;
#endif
    return a*sq(valuemV) + // a*x^2
           b*valuemV +     // b*x
           c;              // c
    /*
     *  4TE ARM-Controller coefficients found with following measurements:
     *  ---------------------
     *  | Bus mV  ADC-Value |
     *  ---------------------
     *  |  9542    872      |
     *  | 10043    920      |
     *  | 11047   1017      |
     *  | 12008   1108      |
     *  | 13013   1202      |
     *  | 14012   1293      |
     *  | 15044   1382      |
     *  | 16024   1465      |
     *  | 17024   1545      |
     *  | 18012   1615      |
     *  | 19031   1686      |
     *  | 20044   1749      |
     *  | 20959   1801      |
     *  | 22003   1854      |
     *  | 23001   1901      |
     *  | 23987   1945      |
     *  | 24979   1985      |
     *  | 25999   2022      |
     *  | 26978   2055      |
     *  | 28006   2087      |
     *  | 29009   2115      |
     *  | 29873   2140      |
     *  | 30184   2150      |
     *  ---------------------
     *
    */
}

void AppUsrCallback::Notify(UsrCallbackType type)
{
    switch (type)
    {
        case UsrCallbackType::reset : // Reset after an ETS-application download or simple @ref APCI_BASIC_RESTART_PDU
            saveRelayState();
            break;

        default :
            break;
    }
}

#endif /* BUSFAIL */
