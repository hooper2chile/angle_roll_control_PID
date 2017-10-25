#include <library.h>

void setup()
{
  Wire.begin();
  Wire.setClock(800000L);
  Serial.begin(250000);

  // Configurar acelerometro
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);

  // Ports for output and interruption
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4);
  Timer1.initialize(TIME_P);
  Timer1.attachInterrupt(pwm_signals);

  pinMode(A0,input);
  pinMode(A1,input);

  // PID setting
  roll_pid.SetSampleTime(15);
  roll_pid.SetOutputLimits(-255,+255);

  input = afilter[1];
  setpoint = 0;// poner acá angulo variable
  roll_pid.SetMode(AUTOMATIC);
}

int DT1 = 0;
int DT2 = 0;

void loop()
{
  PORTB |= (1 << PB0);
  // ---  reading IMU and data ---
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
  raw_values();
  // --- end reading
  PORTB &= ~(1 << PB0);

  input = afilter[0];
  roll_pid.Compute();

  if (output > +255)
   output = +255;
  else if (output < -255)
   output = -255;

  if (output > 0) {
    out1 = +(int8_t) (output + vBASE);
    dt_m2 = map(out1, 0, 255, 0, TIME_B);
  }

  else if (output < 0) {
    out2 = -(int8_t) (output + vBASE);
    dt_m1 = map(out2, 0, 255, 0, TIME_B);
  }

  info();


  delayMicroseconds(dt);
}

/*
// --- manual power for esc1 and esc2
  DT1 = analogRead(A0);
  DT2 = analogRead(A1);
  dt_m1 = map(DT1, 0, 1023, 0, TIME_B);
  dt_m2 = map(DT2, 0, 1023, 0, TIME_B);

*/
