//Arduino Nano 328
//Arduino Nano 168
//Arduino Arduino 328
//Modular V1
//Robot NN
#include <Modular.h>
DigitalOutput led(3);
DigitalInput  pulsador(4);
void setup() {
  // put your setup code here, to run once:
  led.Init();
  pulsador.Init();
}

void loop() {
  // put your main code here, to run repeatedly:
  led.Write(pulsador.Read());
}
