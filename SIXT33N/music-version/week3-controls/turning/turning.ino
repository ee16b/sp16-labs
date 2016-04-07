/* turning.ino
** Implementing turns in SIXT33N
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
#define SIZE 150
#define TIMESTEP 200
#define HIGH_PWM 150
#define LOW_PWM 8
#define INITIAL_PWM 130 // you might need to change this0

// Operation modes
#define MODE_LISTEN 0
#define MODE_DRIVE 1

boolean TIMER_MODE = MODE_LISTEN;
int i=0;

volatile int re_pointer = 0;

// Timer period
#define TIMER_MS 0.35

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

long tempr, templ = 0;
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
int drive_count = 0;
int dir = 0;

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
  // If you change your encoder pins this block
  // needs to be modified
  P2IE |= BIT5; // P2.5 interrupt enabled
  P2IES = BIT5; // set to high edge
  P2IFG &= ~BIT5; // P2.5 IFG cleared
  P1IE |= BIT2; // P1.2 interrupt enabled
  P1IES = BIT2; // set to high edge
  P1IFG &= ~BIT2; // P1.2 IFG cleared

  Serial.begin(38400);

  // Stop wheels
  analogWrite(LEFT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 0); 

  reset_blinker();
  dir = 0;
  
  // Set tier for timestep
  setTimer(MODE_LISTEN);
  __enable_interrupt();
}



void loop()
{  
  if(TIMER_MODE == MODE_LISTEN){

    /*--------------*/
    /* CODE BLOCK D */
    /*--------------*/
   
    if (dir == 0){ // first 3 seconds - go straight
      dir = 1;
      wheel_drive(0.06,0.06);
    } else if (dir == 1){ // second 3 seconds - turn left
      dir = 2;
      // YOUR CODE HERE
      wheel_drive(0.06,0.06);
    } else if (dir == 2){ // third 3 seconds - go straight
      dir = 3;
      wheel_drive(0.06,0.06);
    } else if (dir == 3){ // fourth 3 seconds - turn right
      dir = 0;
      // YOUR CODE HERE
      wheel_drive(0.06,0.06);
    }
  
    /*---------------------*/
    /* END OF CODE BLOCK D */
    /*---------------------*/

  }

  else if(TIMER_MODE == MODE_DRIVE && do_loop){
    
    // Update speed data
    if (left_num_ticks > 0){
      left_current_speed = (float)left_num_ticks/(float)left_history;
    } 
    else {
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

    // Debug data - comment out if unneeded
    //Serial.print(left_current_speed);
    //Serial.print('\t');
    //Serial.print(left_position);
    //Serial.print('\t');
    //Serial.print(right_current_speed);
    //Serial.print('\t');
    //Serial.println(right_position);


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


    // Send new PWM values
    analogWrite(LEFT_MOTOR, (int)left_cur_pwm);
    analogWrite(RIGHT_MOTOR, (int)right_cur_pwm); 
     
    // Counter for how many times loop is executed since entering DRIVE MODE
    drive_count++;

    if (drive_count == 3*1000/TIMESTEP){
      // Completely stop and go back to listen MODE after 3 seconds
      analogWrite(LEFT_MOTOR, 0);
      analogWrite(RIGHT_MOTOR, 0); 
      delay(1000); // 1 seconds buffer 
      re_pointer = 0;
      TIMER_MODE = MODE_LISTEN;
      setTimer(MODE_LISTEN);
    }
    
    do_loop = 0;    
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



// Helper function to set new target velocities and change to DRIVE MODE
void wheel_drive(float left_vel, float right_vel){
  // Reset variables
  drive_count = 0;
  left_position = 0;
  right_position = 0;
  TIMER_MODE = MODE_DRIVE;
  left_target_speed = left_vel;
  right_target_speed = right_vel;

  // Enter DRIVE MODE
  setTimer(MODE_DRIVE);

  // Send starting pulse to start motor movement
  left_cur_pwm = INITIAL_PWM;
  right_cur_pwm = INITIAL_PWM;
  if (left_vel > 0) analogWrite(LEFT_MOTOR, left_cur_pwm);
  if (right_vel > 0) analogWrite(RIGHT_MOTOR, right_cur_pwm); 

  // Trigger encoder readings
  right_history = 0;
  right_num_ticks = 0;
  left_history = 0;
  left_num_ticks = 0;
  P1IFG |= BIT2;
  P2IFG |= BIT5;
  delay(200);
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
  if (P2IFG & BIT5){
    templ = millis();
    left_history += templ - left_last_time;
    left_last_time = templ;
    left_position += 1; //cm
    left_num_ticks += 1;
    P2IFG &= ~BIT5; // P2.5 IFG cleared
  }
}

// Port 1 ISR for right encoder
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  if (P1IFG & BIT2){
    tempr = millis();
    right_history += tempr - right_last_time;
    right_last_time = tempr;
    right_position += 1; //cm
    right_num_ticks += 1;
    P1IFG &= ~BIT2; 
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(boolean mode){
  if (mode == MODE_LISTEN){
    // Set the timer based on 25MHz clock
    TA2CCR0 = (unsigned int)(25000*TIMER_MS);  
    TA2CCTL0 = CCIE;             
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  } 
  else if (mode == MODE_DRIVE){
    // Set the timer based on 32kHz clock
    TA2CCR0 = (unsigned int)(32.768*TIMESTEP);
    TA2CCTL0 = CCIE;             // enable interrupts for Timer A
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  TIMER_MODE = mode;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR( void )
{
  if (TIMER_MODE == MODE_LISTEN){
    if(re_pointer < SIZE){
      re_pointer += 1;
    }
  } 
  else if (TIMER_MODE == MODE_DRIVE){
    do_loop = 1;
  }
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
