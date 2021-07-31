#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "MAX31856.hpp"

#define LED_PIN 25

#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

void core1_entry()
{
    //Configure LED Pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (1)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }
}

int main()
{
    stdio_init_all();

    //Launch led blink on core1
    multicore_launch_core1(core1_entry);

    sleep_ms(5000);
    printf("Starting\n");

    //Initialize max31856
    MAX31856 max31856(PIN_MISO, PIN_SCK, PIN_MOSI, PIN_CS);

    //Set conversion mode
    max31856.setConversionMode(MAX31856_ONESHOT);

    //Set termocouple type
    max31856.setThermocoupleType(MAX31856_TCTYPE_K);

    printf("Thermocouple type: ");
    switch (max31856.getThermocoupleType())
    {
    case MAX31856_TCTYPE_B:
        printf("B Type\n");
        break;
    case MAX31856_TCTYPE_E:
        printf("E Type\n");
        break;
    case MAX31856_TCTYPE_J:
        printf("J Type\n");
        break;
    case MAX31856_TCTYPE_K:
        printf("K Type\n");
        break;
    case MAX31856_TCTYPE_N:
        printf("N Type\n");
        break;
    case MAX31856_TCTYPE_R:
        printf("R Type\n");
        break;
    case MAX31856_TCTYPE_S:
        printf("S Type\n");
        break;
    case MAX31856_TCTYPE_T:
        printf("T Type\n");
        break;
    case MAX31856_VMODE_G8:
        printf("Voltage x8 Gain mode\n");
        break;
    case MAX31856_VMODE_G32:
        printf("Voltage x8 Gain mode\n");
        break;
    default:
        printf("Unknown\n");
        break;
    }

    while (true)
    {

        float temperature = max31856.readThermocoupleTemperature();

        printf("Temp: %0.2fC\n", temperature);

        sleep_ms(1000);
    }
}
