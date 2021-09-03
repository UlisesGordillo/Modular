#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define ServoLibrary  1
#if ServoLibrary
  #include <Servo.h>
  static Servo myservo;
#else
  static Servo myservo;
#endif
#include <Modular.h>

//-----------------------------------------------------------------------------
// Pines declarados
#define   NC  25
//                      1   2   3   4   5   6   7   8   9   10  11  12  13  14
byte pines_dos[14]    ={A6, A7, 13,  8, NC, NC, NC, NC, NC, NC, A0, A1, A2, A3};
byte pines_cuatro[14] ={ 3,  5, 12,  7, NC, NC, NC, NC, NC, NC, 10, 11,  6,  9};
byte pines_cinco[14]  ={NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC};
byte pines_seis[14]   ={NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC};

//---------------------------------------------------------------------------------------
// Funciones para Entradas Analogicas
AnalogInput::AnalogInput (byte Port){
  pinAnalogIn = pines_dos[Port-1];
}
void AnalogInput::Init(){
  dataAdcLast = Read();
}
int AnalogInput::Read(){
  return analogRead(pinAnalogIn);
}
float AnalogInput::ReadVolt(){
  dataAdc = analogRead(pinAnalogIn); 
  return (float)dataAdc*5.0/1023.0;
}
bool AnalogInput::Change(int Porcentaje){
  dataAdc = Read();
  if (abs(dataAdc - dataAdcLast) >= (1023/(Porcentaje*10))) { dataAdcLast = dataAdc; return 1; } 
  else {                                                                        return 0; }
}

//---------------------------------------------------------------------------------------
// Funciones para Salidas Analogicas
AnalogOutput::AnalogOutput (byte Port){
  pinAnalogOut = pines_cuatro[Port-1];
}
void AnalogOutput::Init(){
  pinMode(pinAnalogOut,OUTPUT);
}
byte AnalogOutput::Read(){
  return dataDac;
}
void AnalogOutput::Write(byte data){
  dataDac = data;
  analogWrite(pinAnalogOut,dataDac);
}

//---------------------------------------------------------------------------------------
// Funciones para Entradas Digitales
DigitalInput::DigitalInput (byte Port){
  pinDigitalIn=pines_cuatro[Port-1]; 
}
void DigitalInput::Init(){
  pinMode(pinDigitalIn,INPUT);
  stateInLast = Read();
}
bool DigitalInput::Read(){
  return digitalRead(pinDigitalIn);
}
bool DigitalInput::Change(){
  stateIn = Read();
  if (stateIn!=stateInLast) { stateInLast=stateIn;  return 1; } 
  else {                                            return 0; }
}
bool DigitalInput::Change(bool state){
  stateIn = Read();
  if (stateIn!=stateInLast) {
    stateInLast=stateIn;
    if (state == stateIn){  return 1; }
    else{                   return 0; }
  } 
  else {                    return 0; }
}

//---------------------------------------------------------------------------------------
// Funciones para Salidas Digitales
DigitalOutput::DigitalOutput (byte Port){
  pinDigitalOut=pines_cuatro[Port-1]; 
}
void DigitalOutput::Init(){
  pinMode(pinDigitalOut,OUTPUT);
}
bool DigitalOutput::Read(){
  return stateOut;
}
void DigitalOutput::Write(bool state){
  stateOut = state;
  digitalWrite(pinDigitalOut,stateOut);
}

//---------------------------------------------------------------------------------------
// Funciones para Salidas de Servomotor
ServoOutput::ServoOutput (byte Port){
  pinServoOut = pines_cuatro[Port-1];
}
void ServoOutput::Init(){
  myservo.attach(pinServoOut);
}
byte ServoOutput::Read(){
  return dataServo;
}
void ServoOutput::Write(byte data){
  if (data>180) { data=180; }
  dataServo = data;
  myservo.write(dataServo);
}
void ServoOutput::Start(){
}
void ServoOutput::Stop(){
}
void ServoOutput::Velocity(byte data){
  dataVelo = data;
}
//------------------------------------------------------
// Funciones para Actuador de Motor DC
MotorDcActuator::MotorDcActuator (byte Port){
  pinMotorDcPwm = pines_cuatro[Port-1];
  pinMotorDcDir = pines_dos[Port-1];
}
void MotorDcActuator::Init(){
  pinMode(pinMotorDcPwm,OUTPUT);
  analogWrite(pinMotorDcPwm,0);
  pinMode(pinMotorDcDir,OUTPUT);
}
void MotorDcActuator::Velocity(byte velo){
  dataVelo = velo;
}
void MotorDcActuator::Forward(){
  digitalWrite(pinMotorDcDir,HIGH);
}
void MotorDcActuator::Backward(){
  digitalWrite(pinMotorDcDir,LOW);
}
void MotorDcActuator::Start(){
  analogWrite(pinMotorDcPwm,dataVelo);
}
void MotorDcActuator::Stop(){
  analogWrite(pinMotorDcPwm,0);
}
//------------------------------------------------------
// Funciones para Sensor de Ultrasonido
UltrasonicSensor::UltrasonicSensor (byte Port){
  pinUltrasonicTrig = pines_cuatro[Port-1];
  pinUltrasonicEcho = pines_dos[Port-1];
}
void UltrasonicSensor::Init(){
  pinMode(pinUltrasonicTrig,OUTPUT);
  pinMode(pinUltrasonicEcho,INPUT);
  digitalWrite(pinUltrasonicTrig,LOW);
}
float UltrasonicSensor::Read(){
  digitalWrite(pinUltrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinUltrasonicTrig, LOW);
  float dataDist = pulseIn(pinUltrasonicEcho,HIGH,30000);
  return dataDist / 29.0 / 2.0 ;
}
/*
//------------------------------------------------------
// Funciones para Puerto Serial
class SerialPort{
  public:
    SerialPort (byte Port);
    void Init();
    void Print(String text);
    void Println(String text);
    void Write(byte letra);
    byte Read();
    String Readline();
    bool Available();
  private:
    byte pinSerialTx, pinSerialRx;
    byte dataLetra;
    String dataText;
};
//------------------------------------------------------
// Funciones para Puerto I2C
class I2CPort{
  public:
    I2CPort (byte Port);
    void Init();
    void Write(byte data);
    byte Read();
  private:
    byte pinI2CData, pinI2CClock;
    byte dataI2C;
};
//------------------------------------------------------
// Funciones para Puerto SPI
class SPIPort{
  public:
    SPIPort (byte Port);
    void Init();
    void Write(byte data);
    byte Read();
  private:
    byte pinI2CData, pinI2CClock;
    byte dataI2C;
};
//------------------------------------------------------
// Funciones para Sensor DHT
class DHTSensor{
  public:
    DHTSensor (byte Port);
    void Init();
    byte ReadTemperature();
    byte ReadHumidity();
  private:
    byte pinDht;
};
//------------------------------------------------------
// Funciones para Sensor DHT
class StepperActuator{
  public:
    StepperActuator (byte Port);
    void Init();
    int Read();
    void Origin();
    void Velocity(byte data);
    void Steps(int steps);
  private:
    byte pinStepOut, pinDirOut;
    byte dataRpm,dataSteps;
};
*/