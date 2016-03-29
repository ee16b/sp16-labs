volatile int res = 0;
volatile int flag = 0;
int timer_ms = 3;
int samples[3330] = {0};
int idx = 0;

void setup()
{
  // put your setup code here, to run once:
  pinMode(A0,INPUT);
  pinMode(P3_5, OUTPUT);
  res = 0;
  flag = 0;
  Serial.begin(9600);
  setTimer();
  reset_blinker();
  idx = 0;
}

void loop()
{
  // put your main code here, to run repeatedly:
  //Serial.println((int)analogRead(A0));
  //delay(10);
  if(flag){
    digitalWrite(P3_5, LOW);
    Serial.println("Start");
    for (int i = 0; i < 3330; i++){
      Serial.println(samples[i]);
    }
    flag = 0;
    idx = 0;
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

void setTimer(){
  TA0CCR0 = (unsigned int)(32.768*timer_ms);       // set the timer based on 32kHz clock
  TA0CCTL0 = CCIE;             // enable interrupts for Timer A
  __bis_SR_register(GIE);
  TA0CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
}

#pragma vector=TIMER0_A0_VECTOR    // Timer A ISR
__interrupt void Timer1_A0_ISR( void )
{
  //__bic_SR_register(GIE);
  if (flag == 0){
    digitalWrite(P3_5, HIGH);
    samples[idx++] = (int)analogRead(A0);
    if(idx == 3330) flag = 1;
  }
}
