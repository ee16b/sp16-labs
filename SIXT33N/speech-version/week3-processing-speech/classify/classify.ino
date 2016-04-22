/* integration.ino
** Processing Integration sketch for SIXT33N Speech version
** 
** EE16B Spring 2016
** Emily Naviasky & Nathaniel Mailoa
**
*/

//define constants
#define MIC_INPUT P6_0
#define SIZE 2752
#define SIZE_AFTER_FILTER 172


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

//data array and index pointer
int i=0;
int re[SIZE]={0};
volatile int re_pointer = 0;

//timer period
float timer_ms = 0.35;

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
  //microphone ADC
  pinMode(MIC_INPUT, INPUT);
  pinMode(P6_1, INPUT);
  pinMode(P6_5, INPUT);
  //recording light
  pinMode(RED_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
  
  Serial.begin(38400); 
}

void loop()
{  
  if(re_pointer == SIZE){
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function and get snippet with speech. Do classification only if loud enough
    if(envelope(re, result)){

      /*--------------*/
      /* CODE BLOCK B */
      /*--------------*/

      // Perform principal component projection
      // YOUR CODE HERE 

      // Classification - Hint: use the function l2_norm defined above
      // YOUR CODE HERE 
      
      /*---------------------*/
      /* END OF CODE BLOCK B */
      /*---------------------*/

    }
    
    delay(2000);
    re_pointer = 0;
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



// Timer ISR for ADC sampling
#pragma vector=TIMER0_A0_VECTOR    // Timer A ISR
__interrupt void Timer1_A0_ISR( void )
{
  if(re_pointer < SIZE){
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for ADC
void setTimer(){
  // set the timer based on 25MHz clock
  TA0CCR0 = (unsigned int)(25000*timer_ms);       
  // enable interrupts for Timer A
  TA0CCTL0 = CCIE;             
  __bis_SR_register(GIE);
  TA0CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
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

