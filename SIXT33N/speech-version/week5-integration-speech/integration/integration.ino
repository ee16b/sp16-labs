/* integration.ino
** Final sketch for SIXT33N Speech version
** 
** EE16B Spring 2016
** Emily Naviasky & Nathaniel Mailoa
**
*/

// Define pins
// Note that if you change the encoder pins
// you also need to change the setup function,
// the end of the loop function and the ISR to 
// enable appropriate pin interrupts - ask your 
// GSI if you want to do this
#define LEFT_MOTOR P2_0
#define LEFT_ENCODER P2_5
#define RIGHT_MOTOR P1_5
#define RIGHT_ENCODER P1_2
#define MIC_ADC P6_0

// Define constants
#define TIMESTEP 200
#define HIGH_PWM 150
#define LOW_PWM 80
#define SIZE 2752
#define SIZE_AFTER_FILTER 172
#define INITIAL_PWM 130 // you might need to change this


/*---------------*/
/* CODE BLOCK A1 */
/*---------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE // YOUR CODE HERE 
#define PRELENGTH // YOUR CODE HERE 
#define THRESHOLD // YOUR CODE HERE 

#define KMEANS_THRESHOLD // YOUR CODE HERE 
#define LOUDNESS_THRESHOLD // YOUR CODE HERE 

/*----------------------*/
/* END OF CODE BLOCK A1 */
/*----------------------*/


// Operation modes
#define MODE_LISTEN 0
#define MODE_DRIVE 1

boolean TIMER_MODE = MODE_LISTEN;
int i=0;


// Timer period
#define TIMER_MS 0.35


/*---------------*/
/* CODE BLOCK A2 */
/*---------------*/

//define arrays
float pca_vec1[SNIPPET_SIZE] = {…}; // YOUR CODE HERE
float pca_vec2[SNIPPET_SIZE] = {…}; // YOUR CODE HERE
float mean_vec[SNIPPET_SIZE] = {…}; // YOUR CODE HERE
float centroid1[2] = {…}; // YOUR CODE HERE
float centroid2[2] = {…}; // YOUR CODE HERE
float centroid3[2] = {…}; // YOUR CODE HERE
float centroid4[2] = {…}; // YOUR CODE HERE

/*----------------------*/
/* END OF CODE BLOCK A2 */
/*----------------------*/


float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float dist[4] = {0};


// Data array and index pointer
int i=0;
int re[SIZE]={0};
volatile int16_t re_pointer = 0;

//Control variables
boolean do_loop = 0;
float left_target_speed = 0.04;
float left_current_speed = 0; // in ms
long left_position = 0;
long left_last_time = 0;
int32_t left_history = 0;
int left_num_ticks = 0;

float right_target_speed = 0.04;
float right_current_speed = 0; // in ms
long right_position = 0;
long right_last_time = 0;
int32_t right_history = 0;
int right_num_ticks = 0;

long tempr, templ = 0;
float left_cur_pwm = (HIGH_PWM + LOW_PWM)/2;
float right_cur_pwm = (HIGH_PWM + LOW_PWM)/2;


/*---------------*/
/* CODE BLOCK A3 */
/*---------------*/

// Controller gain vector
float F1_left =  // YOUR CODE HERE 
float F2_left =  // YOUR CODE HERE 
float F1_right =  // YOUR CODE HERE 
float F2_right =  // YOUR CODE HERE 

/*----------------------*/
/* END OF CODE BLOCK A3 */
/*----------------------*/


// drive_counter for how many times timestep since reset
int drive_count = 0;

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int*, float*);

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid){
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid){
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup()
{  
  // Left wheel control and encoder
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  
  // Right wheel control and encoder
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  //microphone ADC
  pinMode(MIC_ADC, INPUT);
  
  pinMode(RED_LED, OUTPUT);

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
  
  // Set timer for timestep
  setTimer(MODE_LISTEN);
  __enable_interrupt();

  re_pointer = 0;
}



void loop()
{  
  if(TIMER_MODE == MODE_LISTEN && re_pointer == SIZE){  
    // Stop motor
    analogWrite(LEFT_MOTOR, 0);
    analogWrite(RIGHT_MOTOR, 0); 
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if(envelope(re, result)){

      /*--------------*/
      /* CODE BLOCK B */
      /*--------------*/

      // Perform principal component projection
      // YOUR CODE HERE 

      // Classification - Hint: use the function l2_norm defined below
      // Call the function wheel_drive with appropriate arguments for going
      // straight at different speeds and turning
      // YOUR CODE HERE 
      
      /*---------------------*/
      /* END OF CODE BLOCK B */
      /*---------------------*/

    }

    delay(2000);
    re_pointer = 0;
  }
 
  else if(TIMER_MODE == MODE_DRIVE && do_loop){
    
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

    if (drive_count == 4*1000/TIMESTEP){
      // Completely stop and go back to LISTEN MODE after 4 seconds
      re_pointer = 0;
      analogWrite(LEFT_MOTOR, 0);
      analogWrite(RIGHT_MOTOR, 0); 
      delay(1000); // 1 second buffer for wheels to stop
      TIMER_MODE = MODE_LISTEN;
      setTimer(MODE_LISTEN);
    }
             
    do_loop = 0;  
  }
  
  // Encoder reading for wheel not moving
  // Needs to be modified if encoder pins changed
  long temp1 = millis();
  if (temp1 - right_last_time > 1000) {
    P1IFG |= BIT2;
  }
  if (temp1 - left_last_time > 1000) {
    P2IFG |= BIT5;
  }

}


// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out){
  int32_t avg = 0;
  float maximum = 0;
  int thres_index = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++){
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }
  
  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) return false;

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) 
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  for (int i = 0; i < SNIPPET_SIZE; i++){
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out 
  for (int i = 0; i < SNIPPET_SIZE; i++)
    data_out[i] = data_out[i] / total;

  return true;
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
  } else if (mode == MODE_DRIVE){
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
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_ADC) >> 4) - 128;
      re_pointer += 1;
    }
  } else if (TIMER_MODE == MODE_DRIVE){
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
