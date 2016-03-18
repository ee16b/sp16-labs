/* collect-data-envelope.ino
** Data collection sketch for SIXT33N Speech version
** 
** EE16B Spring 2016
** Emily Naviasky & Nathaniel Mailoa
**
*/

//define vars
#define MIC_INPUT P6_0
#define SIZE 2752
#define SIZE_AFTER_FILTER 172
int i=0;

//data array and index pointer
int16_t re[SIZE]={0};
volatile int re_pointer = 0;

//timer period
float timer_ms = 0.35;

// Enveloping function
void envelope(int16_t* data, int16_t* data_out){
  int32_t avg = 0;
  int block;
  for (block = 0; block < SIZE_AFTER_FILTER; block++){
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data_out[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data_out[block] += abs(data[i+block*16] - avg);
    }
  }   
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
  setTimer();
  reset_blinker();
  
  Serial.begin(38400); 
}

void loop()
{  
  if(re_pointer == SIZE){
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function
    envelope(re, re);

    // Print out recorded data
    Serial.println("Start");
    for (int i = 0; i < SIZE_AFTER_FILTER; i++)
      Serial.println(re[i]);

    delay(2000);
    re_pointer = 0;
  }
}


#pragma vector=TIMER0_A0_VECTOR    // Timer A ISR
__interrupt void Timer1_A0_ISR( void )
{
  if(re_pointer < SIZE){
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

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
