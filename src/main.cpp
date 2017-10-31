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
  Timer1.initialize(TIME_T);
  Timer1.attachInterrupt(pwm_signals);

  // PID setting
  roll_pid.SetSampleTime(Ts);
  roll_pid.SetOutputLimits(UMIN, UMAX);

  input = afilter[1];
  setpoint = 0;      // poner acá angulo variable
  roll_pid.SetMode(AUTOMATIC);

  delay(7000);    // esperar a los esc ?
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

  out1 = THROTTLE - output;   //out1: LADO DERECHO
  out2 = THROTTLE + output;   //out2: LADO IZQUIERDO


  dt_m1 = map(out1, 0, UMAX, K_MIN1, K_MAX1);
  dt_m2 = map(out2, 0, UMAX, K_MIN2, K_MAX2);

  PORTB &= ~(1 << PB0);
  //mTime = micros()-mTime;
  info();
}
