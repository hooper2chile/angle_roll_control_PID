#include <library.h>


void setup()
{
  wdt_disable();

  Wire.begin();
  Wire.setClock(800000L);
  Serial.begin(115200);

  // Configurar acelerometro
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);

  // Ports for output and interruption
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4);
  Timer1.initialize(TIME_P);
  Timer1.attachInterrupt(pwm_signals);

  // PID setting
  roll_pid.SetSampleTime(2);
  roll_pid.SetOutputLimits(-255,+255);

  input = afilter[1];
  setpoint = 0;// poner acá angulo variable
  roll_pid.SetMode(AUTOMATIC);

  wdt_enable(WDTO_30MS);
}


void loop()
{ //mTime = micros();
  PORTB |= (1 << PB0);

  // ---  reading IMU and data ---
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
  raw_values();
  // --- end reading

  input = afilter[0];
  roll_pid.Compute();
  output = (int) output;

  //out1: LADO IZQUIERDO
  if (output <= 0) {
    if (-output < vBASE1 )
      out1 = vBASE1;
    else
      out1 = - output;
  }
  //out2: LADO DERECHO
  else if ( output > 0) {
    if (output < vBASE2)
      out2 = vBASE2;
    else
      out2 = + output;
  }

  dt_m1 = map(out1, 0, 255, 0, TIME_B);
  dt_m2 = map(out2, 0, 255, 0, TIME_B);

  PORTB &= ~(1 << PB0);
  //mTime = micros()-mTime;
  info();
}
