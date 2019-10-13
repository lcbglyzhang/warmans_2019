
const unsigned int forwardDuty = 26368;
const unsigned int backDuty = 21888;
const unsigned int stallDuty = 24000;

int stop_pin = 45;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 

  pinMode(45, INPUT);
  
  pinMode(11, OUTPUT); //Motor1 OC1A
  output_compare_setup_motor1();
  
  pinMode(5, OUTPUT); // Motor2 OC3A
  output_compare_setup_motor2();

  pinMode(6, OUTPUT); // Motor3 OC4A
  output_compare_setup_motor3();

  pinMode(46, OUTPUT); // Motor4 OC5A
  output_compare_setup_motor4();

  pinMode(44, OUTPUT); // ARM OC5C
  output_compare_setup_arm();

}

void loop() {
//  forwards();
//  backwards();
//  left();
//  rotate_clockwise();
//  rotate_anticlockwise();
//  arm_clockwise();
//  arm_anticlockwise();

  if(!digitalRead(45)) {
    rotate_clockwise();
  } else {
    stall();
    Serial.println("stalled");
  }
  
}


void forwards() {
  setMotor1(backDuty);
  setMotor2(forwardDuty);
  setMotor3(backDuty);
  setMotor4(forwardDuty);
}

void backwards() {
  setMotor1(forwardDuty);
  setMotor2(backDuty);
  setMotor3(forwardDuty);
  setMotor4(backDuty);  
}

void left() {
  setMotor1(forwardDuty);
  setMotor2(forwardDuty);
  setMotor3(backDuty);
  setMotor4(backDuty);  
}

void rotate_clockwise() {
  setMotor1(backDuty);
  setMotor2(backDuty);
  setMotor3(backDuty);
  setMotor4(backDuty);
}

void rotate_anticlockwise() {
  setMotor1(forwardDuty);
  setMotor2(forwardDuty);
  setMotor3(forwardDuty);
  setMotor4(forwardDuty);  
}

void rotate_180(int rotate_time) {
  for (int i = 0; i < rotate_time; i++) {
    setMotor1(forwardDuty);
    setMotor2(forwardDuty);
    setMotor3(forwardDuty);
    setMotor4(forwardDuty);
  }
}

void stall() {
  setMotor1(stallDuty);
  setMotor2(stallDuty);
  setMotor3(stallDuty);
  setMotor4(stallDuty); 
}

void arm_clockwise() {
  engageArm(); // Drive motors all stopped, PIN 44 OC5C connected  
  setMotorArm(3038);
}

void arm_anticlockwise() {
  engageArm();
  setMotorArm(2958);
}

void setMotor1(unsigned int duty) {
  OCR1A = duty;
}

void setMotor2(unsigned int duty) {
  OCR3A = duty;
}

void setMotor3(unsigned int duty) {
  OCR4A = duty;
}

void setMotor4(unsigned int duty) {
  OCR5A = duty;
}

void setMotorArm(unsigned int duty) {
  OCR5C = duty;
}

void engageArm() {
  /* 
   * Engage Arm function disengages OC5A PIN 46 MOTOR 4
   * Engage Arm function engages OC5C PIN 44 ARM MOTOR
   * Engage Arm function stops all drive motors
   */
   noInterrupts();
  
   setMotor1(stallDuty);
   setMotor2(stallDuty);
   setMotor3(stallDuty);
   setMotor4(stallDuty); 
    
   TCCR5A = 0b00001010; // Disable COM5A1:0, Enable COM5C1:0, Keep WGM

   TIMSK5 = 0b00001000; // Disable OC5A interrupt Enable OC5C interrupt

   interrupts();
}

void disengageArm() {
  /* 
   * Disengage Arm function engages OC5A PIN 46 MOTOR 4
   * Disengage Arm function disengages OC5C PIN 44 ARM MOTOR
   */
   noInterrupts();

   TCCR5A = 0b10000010; // Disable COM5A1:0, Enable COM5C1:0, Keep WGM

   TIMSK5 = 0b00000010; // Enable OC5A interrupt Disable OC5C interrupt

   interrupts();
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

ISR(TIMER5_COMPC_vect) {
  OC5C_ISR();
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

void OC5C_ISR() {
  TIFR5 |= (1 << OCF5C);
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
  TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS10); // Set to Fast PWM mode, and prescaler 1

  ICR1 = 32000; // Setting the TOP of the signal, 2ms, 32000 cycles

  OCR1A = 24000; // 

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
  TCCR3B |= (1 << WGM33) | (1 << WGM32) | (1 << CS30); // Set to Fast PWM mode, and prescaler 1

  ICR3 = 32000; // Setting the TOP of the signal, 5ms, 10000 cycles

  OCR3A = 24000; // 

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
  TCCR4B |= (1 << WGM43) | (1 << WGM42) | (1 << CS40); // Set to Fast PWM mode, and prescaler 1

  ICR4 = 32000; // Setting the TOP of the signal, 5ms, 10000 cycles

  OCR4A = 24000; // 

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
  TCCR5B |= (1 << WGM53) | (1 << WGM52) | (1 << CS50); // Set to Fast PWM mode, and prescaler 1

  ICR5 = 32000; // Setting the TOP of the signal, 5ms, 10000 cycles

  OCR5A = 24000; // 

  TIMSK5 |= (1 << OCIE5A); // Enable interrupts on compare nA

  interrupts(); // Enable interrupts
}

/* ARM, PIN 44, OC5C */
void output_compare_setup_arm() {
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
  
  // TCCR5A |= (1 << COM5C1);

  // ICR5 = 10000; // Setting the TOP of the signal, 5ms, 10000 cycles

  OCR5B = 3000; // 

  // TIMSK5 |= (1 << OCIE5B); // Enable interrupts on compare nA

  interrupts(); // Enable interrupts
}
