#include "FFCV3_PIN_DEFINITION.h"

TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(BUZ, PinMap_PWM);
uint32_t channel = STM_PIN_CHANNEL(pinmap_function(BUZ, PinMap_PWM));
// Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
HardwareTimer *MyTim = new HardwareTimer(Instance);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_B,OUTPUT);
}

void loop() {
  for(int i = 1; i < 11;i++){
    for(int j = 0; j < i;j++){
      digitalWrite(LED_B,HIGH);
      MyTim->setPWM(channel, BUZ, 4000, 50); // 5 Hertz, 10% dutycycle
      delay(250);
      MyTim->setPWM(channel, BUZ, 4000, 0); // 5 Hertz, 10% dutycycle
      digitalWrite(LED_B,LOW);
      delay(250);
    }
    delay(3000);
  }
}
