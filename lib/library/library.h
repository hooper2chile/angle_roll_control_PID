
#include   <Arduino.h>
#include   <TimerOne.h>
#include   <PID_v1.h>
#include   <Wire.h>

#define    MPU9250_ADDRESS           0x68
#define    MAG_ADDRESS               0x0C

#define    GYRO_FULL_SCALE_250_DPS   0x00
#define    GYRO_FULL_SCALE_500_DPS   0x08
#define    GYRO_FULL_SCALE_1000_DPS  0x10
#define    GYRO_FULL_SCALE_2000_DPS  0x18

#define    ACC_FULL_SCALE_2_G   0x00
#define    ACC_FULL_SCALE_4_G   0x08
#define    ACC_FULL_SCALE_8_G   0x10
#define    ACC_FULL_SCALE_16_G  0x18

#define    RATIO_ACCE  16384.0
#define    RATIO_GYRO  131.0
#define    RAD_GRAD    57.295779
#define    us          1.0e-6

#define    A         0.98
#define    B         0.02

#define    TIME_P    20        //20us <=> periodo de interrupcion
#define    TIME_T    1000      //1000 cuentas   // 20e-3/20e-6 <=> (periodo 50 Hz)/(periodo de interrupcion)
#define    TIME_B    TIME_T/20 //50 cuentas ó 1ms, 5% de duty cycle como base para esc.
#define    vBASE     120.0

#define    PWM_PORT  PORTB
#define    PIN1_      PB1
#define    PIN2_      PB2
#define    PIN3_      PB3
#define    PIN4_      PB4

uint16_t count_m1  = 0;
uint16_t count_m2  = 0;
uint16_t count_m3  = 0;
uint16_t count_m4  = 0;

volatile uint16_t dt_m1 = 0;    //dt=0  <=> 5%  duty cycle  //dt=50 <=> 10% duty cycle
volatile uint16_t dt_m2 = 0;
volatile uint16_t dt_m3 = 0;
volatile uint16_t dt_m4 = 0;

int out1 = 0;
int out2 = 0;


uint8_t Buf[14];
int16_t a[3] = {0};  //ax, ay, az;
int16_t g[3] = {0};  //gx, gy, gz;

float dt         = 500;
float coef[6]    = {0};   //Ax, Ay, Az, Gx, Gy, Gz;
float angle[2]   = {0};   //angle_ax, angle_ay;
float afilter[2] = {0};   //angle_x_filter, angle_y_filter;


/**************************************************************************************
*   pid definitions for automatic control
***************************************************************************************/
double setpoint, input, output;
double kp=1.4, ki=1, kd=0.6;
PID roll_pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);



/**************************************************************************************
*   i2c definitions for communication to IMU9250
***************************************************************************************/
//Funcion auxiliar lectura
inline void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}


// Funcion auxiliar de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


inline float filter(float AF_Prev, float Gyro, float Angle)
{
  return ( A*(AF_Prev + Gyro*(dt*us)) + B*Angle );
}


void raw_values()
{ //accelerometer
  a[0] = - (Buf[0] << 8 | Buf[1]);
  a[1] = - (Buf[2] << 8 | Buf[3]);
  a[2] = + (Buf[4] << 8 | Buf[5]);
  //gyroscopio
  g[0] = - (Buf[8]  << 8 | Buf[9] );
  g[1] = - (Buf[10] << 8 | Buf[11]);
  g[2] = + (Buf[12] << 8 | Buf[13]);

  //scaled
  coef[0] = (float) (a[0]/RATIO_ACCE);  //Ax, ax
  coef[1] = (float) (a[1]/RATIO_ACCE);  //Ay
  coef[2] = (float) (a[2]/RATIO_ACCE);  //Az
  coef[3] = (float) (g[0]/RATIO_GYRO);  //Gx
  coef[4] = (float) (g[1]/RATIO_GYRO);  //Gy, gy

  //roll, pitch
  angle[0] = atan(coef[1]/sqrt( coef[0]*coef[0] + coef[2]*coef[2] ))*RAD_GRAD;
  angle[1] = atan(coef[0]/sqrt( coef[1]*coef[1] + coef[2]*coef[2] ))*RAD_GRAD;

  afilter[0] = filter(afilter[0], coef[3], angle[0]);
  afilter[1] = filter(afilter[1], coef[4], angle[1]);
}


void info()
{// --- Mostrar valores  ---
  Serial.print(afilter[0]);
  Serial.print("\t");
  Serial.print((int8_t) output);
  Serial.print("\t");
  Serial.print(out1);
  Serial.print("\t");
  Serial.print(out2);
  Serial.println("");
}


/**************************************************************************************
*   interruption and speed control for esc/motors
***************************************************************************************/
//Function generator for pwm of 50 Hz, and width variable.
inline void pin_updown(uint16_t *count, volatile uint16_t *dt, volatile uint8_t *PORT, uint8_t PIN) {
  if (*count == 0)
    *PORT |= PIN;
  else if (*count == TIME_B + *dt )
    *PORT &= ~PIN;

  if (*count == TIME_T)
    *count = 0;
  else
    (*count)++;
}


void pwm_signals() {
  //PORTB |= (1 << PB0);

  pin_updown(&count_m1, &dt_m1, &PWM_PORT, _BV(PIN1_));
  pin_updown(&count_m2, &dt_m2, &PWM_PORT, _BV(PIN2_));  //motor 2, ON
  pin_updown(&count_m3, &dt_m3, &PWM_PORT, _BV(PIN3_));
  pin_updown(&count_m4, &dt_m4, &PWM_PORT, _BV(PIN4_));

  //PORTB &= ~(1 << PB0);
}