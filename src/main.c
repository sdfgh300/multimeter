
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
// #include <avr/interrupt.h>

// #include "i2cmaster.h"
#include "usart.h"
#include <math.h>
// #include "lcd.h"


float read_voltage(uint8_t chosenPIN);
float CalculateVoltage();
float CalculateResistance();
void SelectMeasurement(uint8_t);


#define channel7 7
#define channel6 6
#define channel5 5
#define channel4 4
#define channel3 3
#define channel2 2
#define channel1 1
#define channel0 0

#define AVG_STRENGTH 50

int main(void)
{
    uart_init();   // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication
    // I2C communication enabled
    i2c_init(); //??? not sure about the order
                // LCD_init();
    int measurementSelecter = 0;
    SelectMeasurement(measurementSelecter);
    
    while (1)
    {
        
        SelectMeasurement(1);
        if(measurementSelecter == 1){
            float resistance = CalculateResistance();
            //printf 
        }
        else{
            CalculateVoltage();
            //printf
        }
       
       
    }

    return 0;
}


float read_voltage(uint8_t channel)
{
    // ADMUX &= 0xf0;
    // ADMUX = 0x00;//(1 << REFS0); // select internal 5V as reference voltage
    ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); // selecting the prescaler for ADC conversion

    ADMUX &= 0xF0;    // clear the lower 4 bits of ADMUX to select ADC channel
    ADMUX |= channel; // select ADC channel 1 (replace ADC_CHANNEL_1 with your defined constant)

    ADCSRA |= (1 << ADEN) | (1 << ADSC); // enable ADC and start conversion
    ADMUX |= (1 << REFS0);
    // Wait for the conversion to complete
    while (ADCSRA & (1 << ADSC))
    {
        // Wait until ADSC becomes 0 (conversion is complete)
    }

    // You may want to clear the ADSC bit here, although it's not necessary for single conversions
    // ADCSRA &= ~(1 << ADSC);

    // Return the ADC result
    return ADC;
}

/// @brief Select whether to measure voltage or resistance
/// @param desired [1] To select Voltage    [0] To select Resistance
void SelectMeasurement(uint8_t desired)
{
    if (desired == 1)
    {
        DDRD |= (1 << PD7);
        PORTD |= (1 << PD7);
    }
    else
    {
        DDRD &= ~(1 << PD7);
        PORTD &= ~(1 << PD7);
    }
}



/// @brief Calculates resistance based on voltage measured.
/// @return Calculated value of resistance in ohms(?)
float CalculateResistance(){


    float avgVoltage = 0;
        for (uint8_t i = 0; i < AVG_STRENGTH; i++)
        {
            float value = read_voltage(channel0);
            float voltage = (((value * 4.9) / 1024.0f) - 2.4) * 2;
            avgVoltage += voltage;
        }
        avgVoltage /= AVG_STRENGTH;
        SelectMeasurement(0);

        float value = read_voltage(channel1);
        float resistance = ((1024.0f / value) - 1) * 986;

        // printf("Volts: %.1f\n", avgResistance);

        printf("Volts: %.1f  || Resistance: %.1f\n", avgVoltage, resistance);
        return resistance;

}

/// @brief Caluclates the read voltage according to the reference voltage.
/// @return Calculated voltage (float).
float CalculateVoltage(){
    int adc = read_voltage(channel0);
    float power_source = 5; //volts
    float voltage = adc*power_source/1023; //calculations for ADC
    return voltage;
}



