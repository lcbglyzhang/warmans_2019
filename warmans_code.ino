void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  
  pinMode(11, OUTPUT); //MotorA OC1A
  output_compare_setup_motor1();
  
  pinMode(5, OUTPUT);
  output_compare_setup_motor2();

  pinMode(6, OUTPUT);
  output_compare_setup_motor3();

  pinMode(46, OUTPUT);
  output_compare_setup_motor4();

}

void loop() {
  // put your main code here, to run repeatedly:
  
}

ISR(TIMER1_COMPA_vect) {
  OC1A_ISR();
}

ISR(TIMER3_COMPA_vect) {
  OC3A_ISR();
}

ISR(TIMER4_COMPA_vect) {
  OC4A_ISR();
}

ISR(TIMER5_COMPA_vect) {
  OC5A_ISR();
}

void OC1A_ISR() {
  //Serial.println("aaaa");
  TIFR1 |= (1 << OCF1A);
}

void OC3A_ISR() {
  TIFR3 |= (1 << OCF3A);
}

void OC4A_ISR() {
  TIFR4 |= (1 << OCF4A);
}

void OC5A_ISR() {
  TIFR5 |= (1 << OCF5A);
}

/* MOTOR1, PIN 11, OC1A */
void output_compare_setup_motor1() {
  /*
   * This function configures Timer Registers 1, 3, 4, 5 for output compare mode
   * Desired PWM period is 5ms, duty cycle lasting 1 to 2ms, 1.5ms being neutral
   * 16 MHz oscillator, Prescaler of 8
   * Effective Timer Clock is 2 MHz, TCNT increments every 0.5 us
   * 
   * Desired PERIOD is 5ms, OR 10000 clock cycles
   * Desired DUTY is 1ms to 2ms (2000 to 4000 cycles)
   * NEUTRAL DUTY is 1.5ms (3000 cycles)
   * 
   * Timer n is set to Fast PWM mode
   * 
   */
  noInterrupts(); // Disable interrupts

  // Clear Timers
  TCNT1 = 0; // Clear TCNT
  
  // Clear Control Registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;

  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Set to Fast PWM mode, and prescaler 8

  ICR1 = 10000; // Setting the TOP of the signal, 5ms, 10000 cycles

  OCR1A = 2800; // 

  TIMSK1 |= (1 << OCIE1A); // Enable interrupts on compare nA

  interrupts(); // Enable interrupts
}

/* MOTOR2, PIN 5, OC3A */
void output_compare_setup_motor2() {
  /*
   * This function configures Timer Registers 1, 3, 4, 5 for output compare mode
   * Desired PWM period is 5ms, duty cycle lasting 1 to 2ms, 1.5ms being neutral
   * 16 MHz oscillator, Prescaler of 8
   * Effective Timer Clock is 2 MHz, TCNT increments every 0.5 us
   * 
   * Desired PERIOD is 5ms, OR 10000 clock cycles
   * Desired DUTY is 1ms to 2ms (2000 to 4000 cycles)
   * NEUTRAL DUTY is 1.5ms (3000 cycles)
   * 
   * Timer n is set to Fast PWM mode
   * 
   */
  noInterrupts(); // Disable interrupts

  // Clear Timers
  TCNT3 = 0; // Clear TCNT
  
  // Clear Control Registers
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3C = 0;

  TCCR3A |= (1 << COM3A1) | (1 << WGM31);
  TCCR3B |= (1 << WGM33) | (1 << WGM32) | (1 << CS31); // Set to Fast PWM mode, and prescaler 8

  ICR3 = 10000; // Setting the TOP of the signal, 5ms, 10000 cycles

  OCR3A = 3100; // 

  TIMSK3 |= (1 << OCIE3A); // Enable interrupts on compare nA

  interrupts(); // Enable interrupts
}

/* MOTOR3, PIN 6, OC4A */
void output_compare_setup_motor3() {
  /*
   * This function configures Timer Registers 1, 3, 4, 5 for output compare mode
   * Desired PWM period is 5ms, duty cycle lasting 1 to 2ms, 1.5ms being neutral
   * 16 MHz oscillator, Prescaler of 8
   * Effective Timer Clock is 2 MHz, TCNT increments every 0.5 us
   * 
   * Desired PERIOD is 5ms, OR 10000 clock cycles
   * Desired DUTY is 1ms to 2ms (2000 to 4000 cycles)
   * NEUTRAL DUTY is 1.5ms (3000 cycles)
   * 
   * Timer n is set to Fast PWM mode
   * 
   */
  noInterrupts(); // Disable interrupts

  // Clear Timers
  TCNT4 = 0; // Clear TCNT
  
  // Clear Control Registers
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4C = 0;

  TCCR4A |= (1 << COM4A1) | (1 << WGM41);
  TCCR4B |= (1 << WGM43) | (1 << WGM42) | (1 << CS41); // Set to Fast PWM mode, and prescaler 8

  ICR4 = 10000; // Setting the TOP of the signal, 5ms, 10000 cycles

  OCR4A = 3000; // 

  TIMSK4 |= (1 << OCIE4A); // Enable interrupts on compare nA

  interrupts(); // Enable interrupts
}

/* MOTOR4, PIN 46, OC5A */
void output_compare_setup_motor4() {
  /*
   * This function configures Timer Registers 1, 3, 4, 5 for output compare mode
   * Desired PWM period is 5ms, duty cycle lasting 1 to 2ms, 1.5ms being neutral
   * 16 MHz oscillator, Prescaler of 8
   * Effective Timer Clock is 2 MHz, TCNT increments every 0.5 us
   * 
   * Desired PERIOD is 5ms, OR 10000 clock cycles
   * Desired DUTY is 1ms to 2ms (2000 to 4000 cycles)
   * NEUTRAL DUTY is 1.5ms (3000 cycles)
   * 
   * Timer n is set to Fast PWM mode
   * 
   */
  noInterrupts(); // Disable interrupts

  // Clear Timers
  TCNT5 = 0; // Clear TCNT
  
  // Clear Control Registers
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5C = 0;

  TCCR5A |= (1 << COM5A1) | (1 << WGM51);
  TCCR5B |= (1 << WGM53) | (1 << WGM52) | (1 << CS51); // Set to Fast PWM mode, and prescaler 8

  ICR5 = 10000; // Setting the TOP of the signal, 5ms, 10000 cycles

  OCR5A = 3200; // 

  TIMSK5 |= (1 << OCIE5A); // Enable interrupts on compare nA

  interrupts(); // Enable interrupts
}
