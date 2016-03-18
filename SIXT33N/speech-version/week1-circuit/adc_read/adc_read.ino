int res;

void setup()
{
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  Serial.begin(9600);
  reset_blinker();
}

void loop()
{
  // put your main code here, to run repeatedly:
  delay(1);
  res = analogRead(A0);
  //split into 5 bit chunks
  Serial.write((byte) ((res >> 6) & 63) | 64);
  Serial.write((byte) (res & 63));
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
