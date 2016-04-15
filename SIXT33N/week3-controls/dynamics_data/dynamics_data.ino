/* dynamics_data.ino
** Collects dynamics data (position and velocity) with
** some varying (bounded) input PWM
** 
** EE16B Spring 2016
** Emily Naviasky & Nathaniel Mailoa
**
*/

// Define pins
#define LEFT_MOTOR P2_0
#define LEFT_ENCODER P2_5
#define RIGHT_MOTOR P1_5
#define RIGHT_ENCODER P1_2

// Define constants
#define SIZE 5*20
#define TIMESTEP 200
#define HIGH_PWM 170
#define LOW_PWM 100
#define PWM_DELTA 6
int i=0;

// Arrays for position, velocity, and pwm data storage
uint16_t lpos[SIZE] = {0};
float lvel[SIZE] = {0};
uint16_t rpos[SIZE] = {0};
float rvel[SIZE] = {0};
uint8_t pwm[SIZE] = {0};
int arr_pointer = 0;
boolean do_loop = 0; // timer signal to increment timestep

// Recording variables
int left_position = 0;
long left_last_time = 0;
int32_t left_history = 0;
int left_num_ticks = 0;
int left_pointer = 0;
int right_position = 0;
long right_last_time = 0;
int32_t right_history = 0;
int right_num_ticks = 0;
int right_pointer = 0;

long temp = 0;
int cur_pwm = (HIGH_PWM + LOW_PWM)/2;
boolean up = 1;

void setup()
{  
  // Left wheel control and encoder
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  
  // Right wheel control and encoder
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  pinMode(RED_LED, OUTPUT);

  // Turn off motors and wait 5 seconds
  analogWrite(LEFT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 0); 
  delay(5000);

  // Start motor
  analogWrite(LEFT_MOTOR, 255);
  analogWrite(RIGHT_MOTOR, 255); 

  // Turn on and set pin interrupts
  P2IE |= BIT5; // P2.5 interrupt enabled
  P2IES = BIT5; // set to high edge
  P2IFG &= ~BIT5; // P2.5 IFG cleared
  P1IE |= BIT2; // P1.2 interrupt enabled
  P1IES = BIT2; // set to high edge
  P1IFG &= ~BIT2; // P1.2 IFG cleared

  Serial.begin(38400);
  
  delay(2000);
  analogWrite(LEFT_MOTOR, cur_pwm);
  analogWrite(RIGHT_MOTOR, cur_pwm); 
  
  // Wait 2 seconds before start recording
  delay(2000);
  reset_blinker();
  
  // Set timer for timestep
  setTimer();
  
  __enable_interrupt();
}



void loop()
{  
  if(do_loop){
    // If not done collecting data
    if(arr_pointer < SIZE){
      lpos[arr_pointer] = left_position;
      rpos[arr_pointer] = right_position;
      lvel[arr_pointer] = 0;
      rvel[arr_pointer] = 0;
      
      // Take velocity readings
      if (left_num_ticks > 0){
        lvel[arr_pointer] = float(left_history)/left_num_ticks;
      } else {
        lvel[arr_pointer] = 200;
      }
      left_history = 0;
      left_num_ticks = 0;

      if (right_num_ticks > 0){
        rvel[arr_pointer] = float(right_history)/right_num_ticks;
      } else {
        rvel[arr_pointer] = 200;
      }
      right_history = 0;
      right_num_ticks = 0;
      
      pwm[arr_pointer] = cur_pwm;
      arr_pointer++;

      // Randomize the input PWM
      if (up){
        if (cur_pwm < HIGH_PWM)
          cur_pwm += random(0, PWM_DELTA);
        else {
          up = 0;
          cur_pwm -= random(0, PWM_DELTA);
        }
      } 
      else {
        if (cur_pwm > LOW_PWM)
          cur_pwm -= random(0, PWM_DELTA);
        else {
          up = 1;
          cur_pwm += random(0, PWM_DELTA);
        }
      }
      
      // Provide new input to system
      analogWrite(LEFT_MOTOR, cur_pwm);
      analogWrite(RIGHT_MOTOR, cur_pwm); 
    } 
    else { // If done collecting data
      // Turn off motors
      analogWrite(LEFT_MOTOR, 0);
      analogWrite(RIGHT_MOTOR, 0); 
      
      // Print out result
      Serial.println("Start");
      for(int i = 0; i < SIZE; i++){
        Serial.print(pwm[i]);
        Serial.print(',');
        Serial.print(lpos[i]);
        Serial.print(',');
        Serial.print(lvel[i], 3);
        Serial.print(',');
        Serial.print(rpos[i]);
        Serial.print(',');
        Serial.print(rvel[i], 3);
        Serial.print('\n');
      }
    }
    do_loop = 0;
  }
}


// Port 2 ISR for reading left encoder
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
  temp = millis();
  left_history += temp - left_last_time;
  left_last_time = temp;
  left_position += 1; //cm
  left_num_ticks += 1;
  P2IFG &= ~BIT5; // P2.5 IFG cleared
}

// Port 1 ISR for reading right encoder
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  temp = millis();
  right_history += temp - right_last_time;
  right_last_time = temp;
  right_position += 1; //cm
  right_num_ticks += 1;
  P1IFG &= ~BIT2; // P1.2 IFG cleared
}


// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(){
  TA2CCR0 = (unsigned int)(32.768*TIMESTEP);       // set the timer based on 32kHz clock
  TA2CCTL0 = CCIE;             // enable interrupts for Timer A
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR( void )
{
  do_loop = 1;
}


void reset_blinker(){
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}
