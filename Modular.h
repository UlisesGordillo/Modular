#ifndef Modular_h
#define Modular_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
//------------------------------------------------------
// Funciones para Entradas Analogicas
class AnalogInput{
  public:
    AnalogInput(byte Port);
    void Init();
    int Read();
    float ReadVolt();
    bool Change(int Porcentaje);
  private:
    byte pinAnalogIn;
    int dataAdc,dataAdcLast;
};
//------------------------------------------------------
// Funciones para Salidas Analogicas
class AnalogOutput{
  public:
    AnalogOutput (byte Port);
    void Init();
    byte Read();
    void Write(byte data);
  private:
    byte pinAnalogOut;
    byte dataDac;
};
//------------------------------------------------------
// Funciones para Entradas Digitales
class DigitalInput{
  public:
    DigitalInput(byte Port);
    void Init();
    bool Read();
    bool Change();
    bool Change(bool state);
  private:
    byte pinDigitalIn;
    bool stateIn, stateInLast;
};
//------------------------------------------------------
// Funciones para Salidas Digitales
class DigitalOutput{
  public:
    DigitalOutput(byte Port);
    void Init();
    bool Read();
    void Write(bool state);
  private:
    byte pinDigitalOut;
    bool stateOut;
};
//------------------------------------------------------
// Funciones para Salidas de Servomotor
class ServoOutput{
  public:
    ServoOutput (byte Port);
    void Init();
    byte Read();
    void Write(byte data);
    void Start();
    void Stop();
    void Velocity(byte velo);
  private:
    byte pinServoOut;
    byte dataServo,dataVelo;
};
//------------------------------------------------------
// Funciones para Actuador de Motor DC
class MotorDcActuator{
  public:
    MotorDcActuator (byte Port);
    void Init();
    void Velocity(byte velo);
    void Forward();
    void Backward();
    void Start();
    void Stop();
  private:
    byte pinMotorDcPwm, pinMotorDcDir;
    byte dataVelo;
};
//------------------------------------------------------
// Funciones para Sensor de Ultrasonido
class UltrasonicSensor{
  public:
    UltrasonicSensor (byte Port);
    void Init();
    float Read();
  private:
    byte pinUltrasonicTrig, pinUltrasonicEcho;
};
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

#endif