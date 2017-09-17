#define STATE_BIT0_PIN D0
#define STATE_BIT1_PIN D1
#define STATE_BIT2_PIN D2
#define STATE_BIT3_PIN D3
#define INT_LINE D4

void setup() {
  //start serial communication
  Serial.begin(9600);

  //set state pins as output
  pinMode(STATE_BIT0_PIN, OUTPUT);
  pinMode(STATE_BIT1_PIN, OUTPUT);
  pinMode(STATE_BIT2_PIN, OUTPUT);
  pinMode(STATE_BIT3_PIN, OUTPUT);

  //set flag to read state lines as output)
  pinMode(INT_LINE, OUTPUT);

}

void loop() {
  delay(100);
  digitalWrite(STATE_BIT3_PIN, HIGH);
  digitalWrite(STATE_BIT2_PIN, HIGH);
  digitalWrite(STATE_BIT1_PIN, HIGH);
  digitalWrite(STATE_BIT0_PIN, HIGH);
  delay(200);
  digitalWrite(INT_LINE, LOW);
  delay(200);
  digitalWrite(INT_LINE, HIGH);
  delay(3000);
  
  delay(100);
  digitalWrite(STATE_BIT3_PIN, HIGH);
  digitalWrite(STATE_BIT2_PIN, LOW);
  digitalWrite(STATE_BIT1_PIN, HIGH);
  digitalWrite(STATE_BIT0_PIN, HIGH);
  delay(200);
  digitalWrite(INT_LINE, LOW);
  delay(200);
  digitalWrite(INT_LINE, HIGH);
  delay(3000);

  delay(100);
  digitalWrite(STATE_BIT3_PIN, HIGH);
  digitalWrite(STATE_BIT2_PIN, LOW);
  digitalWrite(STATE_BIT1_PIN, HIGH);
  digitalWrite(STATE_BIT0_PIN, LOW);
  delay(200);
  digitalWrite(INT_LINE, HIGH);
  delay(200);
  digitalWrite(INT_LINE, HIGH);
  delay(3000);

  delay(100);
  digitalWrite(STATE_BIT3_PIN, LOW);
  digitalWrite(STATE_BIT2_PIN, HIGH);
  digitalWrite(STATE_BIT1_PIN, LOW);
  digitalWrite(STATE_BIT0_PIN, LOW);
  delay(200);
  digitalWrite(INT_LINE, LOW);
  delay(200);
  digitalWrite(INT_LINE, HIGH);
  delay(3000);

  delay(100);
  digitalWrite(STATE_BIT3_PIN, HIGH);
  digitalWrite(STATE_BIT2_PIN, HIGH);
  digitalWrite(STATE_BIT1_PIN, HIGH);
  digitalWrite(STATE_BIT0_PIN, LOW);
  delay(200);
  digitalWrite(INT_LINE, LOW);
  delay(200);
  digitalWrite(INT_LINE, HIGH);
  delay(3000);
  
}
