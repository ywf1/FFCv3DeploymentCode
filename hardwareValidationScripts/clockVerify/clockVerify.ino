#include "Arduino.h"
#include "stm32h7xx_hal.h"

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("STM32H743 Clock Verification");
    
    // Get and print system clocks
    uint32_t sysClock = HAL_RCC_GetSysClockFreq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t hseFreq = HSE_VALUE; // Defined in system_stm32h7xx.h
    uint32_t lseFreq = LSE_VALUE; // Defined in system_stm32h7xx.h

    Serial.print("System Clock (SYSCLK): ");
    Serial.print(sysClock);
    Serial.println(" Hz");

    Serial.print("HCLK: ");
    Serial.print(hclk);
    Serial.println(" Hz");

    Serial.print("HSE Clock: ");
    Serial.print(hseFreq);
    Serial.println(" Hz");

    Serial.print("LSE Clock: ");
    Serial.print(lseFreq);
    Serial.println(" Hz");
}

void loop() {
    delay(5000); // Print every 5 seconds
}