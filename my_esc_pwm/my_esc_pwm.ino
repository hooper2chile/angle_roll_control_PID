#include<TimerOne.h>

#define us       1e-6
#define FREC_B   50.0                          //50 Hz, frecuencia de la señal de comando pwm para el ESC
#define TIME_P   20.0                          //20us <=> periodo de interrupcion
#define TIME_T   (int)((1/FREC_B)/(TIME_P*us))  //1000 cuentas   // 20e-3/20e-6 <=> (periodo 50 Hz)/(periodo de interrupcion)
#define TIME_B   TIME_T/20    


#define PWM_PORT  PORTB
#define PIN1      PB1
#define PIN2      PB2
#define PIN3      PB3
#define PIN4      PB4

uint16_t count_m1  = 0;
uint16_t count_m2  = 0;
uint16_t count_m3  = 0;
uint16_t count_m4  = 0;

volatile uint16_t dt_m1 = 0;    //dt=0  <=> 5%  duty cycle  //dt=50 <=> 10% duty cycle
volatile uint16_t dt_m2 = 0;
volatile uint16_t dt_m3 = 0;
volatile uint16_t dt_m4 = 0;
 
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
  PORTB |= (1 << PB0);
  
  pin_updown(&count_m1, &dt_m1, &PWM_PORT, _BV(PIN1));  
  pin_updown(&count_m2, &dt_m2, &PWM_PORT, _BV(PIN2));  //motor 2, ON
  pin_updown(&count_m3, &dt_m3, &PWM_PORT, _BV(PIN3));
  pin_updown(&count_m4, &dt_m4, &PWM_PORT, _BV(PIN4));

  PORTB &= ~(1 << PB0);
}



void setup() {
  Serial.begin(115200);
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);
  Timer1.initialize(TIME_P);
  Timer1.attachInterrupt(pwm_signals);
}

int DT1 = 0;
int DT2 = 0;
void loop() {
   DT1 = analogRead(A0);
   //DT2 = analogRead(A1);
   dt_m1 = map(DT1, 0, 1023, 0, TIME_B);
   //dt_m2 = map(DT1, 0, 1023, 0, TIME_B);
   delay(1);

   Serial.println(dt_m1);
}


