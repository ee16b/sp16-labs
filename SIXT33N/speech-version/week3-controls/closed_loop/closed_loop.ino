/* closed_loop.ino
** Closed loop control for SIXT33N
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

//define constants
#define SIZE 5*30
#define TIMESTEP 200
#define HIGH_PWM 150
#define LOW_PWM 80
int i=0;

//Control variables
boolean do_loop = 0;
float left_target_speed = 0.06;
float left_current_speed = 0; // in ms
long left_position = 0;
long left_last_time = 0;
int32_t left_history = 0;
int left_num_ticks = 0;
int left_pointer = 0;

float right_target_speed = 0.06;
float right_current_speed = 0; // in ms
long right_position = 0;
long right_last_time = 0;
int32_t right_history = 0;
int right_num_ticks = 0;
int right_pointer = 0;

long temp = 0;
float left_cur_pwm = (HIGH_PWM + LOW_PWM)/2;
float right_cur_pwm = (HIGH_PWM + LOW_PWM)/2;


/*---------------*/
/* CODE BLOCK A3 */
/*---------------*/

// Control vector
float alpha1_left = // YOUR CODE HERE
float alpha2_left = // YOUR CODE HERE
float alpha1_right = // YOUR CODE HERE
float alpha2_right = // YOUR CODE HERE

/*----------------------*/
/* END OF CODE BLOCK A3 */
/*----------------------*/


// Counter for how many times timestep since reset
int count = 0;

void setup()
{  
  // Left wheel control and encoder
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  
  // Right wheel control and encoder
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  pinMode(RED_LED, OUTPUT);

  // Start motor
  analogWrite(LEFT_MOTOR, (int)left_cur_pwm);
  analogWrite(RIGHT_MOTOR, (int)right_cur_pwm); 

  // Turn on and set pin interrupts
  P2IE |= BIT5; // P2.5 interrupt enabled
  P2IES = BIT5; // set to high edge
  P2IFG &= ~BIT5; // P2.5 IFG cleared
  P1IE |= BIT2; // P1.2 interrupt enabled
  P1IES = BIT2; // set to high edge
  P1IFG &= ~BIT2; // P1.2 IFG cleared

  Serial.begin(38400);
  
  // Wait 2 seconds before start operating
  delay(2000);
  reset_blinker();
  
  // Set tier for timestep
  setTimer();
  
  __enable_interrupt();
}



void loop()
{  
  if(do_loop){
    
    // Update speed data
    if (left_num_ticks > 0){
      left_current_speed = (float)left_num_ticks/(float)left_history;
    } else {
      left_current_speed = 1.0/200;
    }
    left_history = 0;
    left_num_ticks = 0;

    if (right_num_ticks > 0){
      right_current_speed = (float)right_num_ticks/(float)right_history;
    } else {
      right_current_speed = 1.0/200;
    }
    right_history = 0;
    right_num_ticks = 0;
    

    /*--------------*/
    /* CODE BLOCK C */
    /*--------------*/

    // Input into open loop (u: change PWM)
    // YOUR CODE HERE
    input_left(...);
    input_right(...);

    /*---------------------*/
    /* END OF CODE BLOCK C */
    /*---------------------*/


    analogWrite(LEFT_MOTOR, (int)left_cur_pwm);
    analogWrite(RIGHT_MOTOR, (int)right_cur_pwm); 
     
    // Debug data - comment out if unneeded
    Serial.println(left_cur_pwm);
    Serial.println(left_current_speed, 6);
    Serial.println(left_position);
    Serial.println();
    Serial.println(right_cur_pwm);
    Serial.println(right_current_speed, 6);
    Serial.println(right_position);
    Serial.println();
    Serial.println();
    
    do_loop = 0;    
    count++;
  }
  
  // Encoder reading for wheel not moving
  long temp1 = millis();
  if (temp1 - right_last_time > 1000) {
    P1IFG |= BIT2;
  }
  if (temp1 - left_last_time > 1000) {
    P2IFG |= BIT5;
  }
}

// New PWM signal for left controller
void input_left(float in){
  left_cur_pwm += in;
  // Protect for saturation
  if(left_cur_pwm > 255) left_cur_pwm = 255;
  if(left_cur_pwm < 0) left_cur_pwm = 0;
}

// New PWM signal for right controller
void input_right(float in){
  right_cur_pwm += in;
  // Protect for saturation
  if(right_cur_pwm > 255) right_cur_pwm = 255;
  if(right_cur_pwm < 0) right_cur_pwm = 0;
}

// Port 2 ISR for left encoder
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

// Port 1 ISR for right encoder
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
