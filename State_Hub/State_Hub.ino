//define pins
	int in_0 = A0;
	int in_1 = A1;
	int in_2 = A2;
	int in_3 = A3;
	int Gs = A4;	//group select line indicates priority inputs present

	int out_0 = 2;
	int out_1 = 3;
	int out_2 = 4;
	int out_3 = 5;
	int start_send = 0;

//define input pin variables
	boolean val_in_0;
	boolean val_in_1;
	boolean val_in_2;
	boolean val_in_3;
	boolean val_Gs;

void setup() 
{

//set on-board LED pin as output, and turn off
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
  
//set encoder receipt pins as inputs
	pinMode(in_0, INPUT);
	pinMode(in_1, INPUT);
	pinMode(in_2, INPUT);
	pinMode(in_3, INPUT);
	pinMode(Gs, INPUT);
	
//set state update pins as outputs
	pinMode(out_0, OUTPUT);
	pinMode(out_1, OUTPUT);
	pinMode(out_2, OUTPUT);
	pinMode(out_3, OUTPUT);
	pinMode(start_send, OUTPUT);
  
}

void loop() 
{
	val_Gs = digitalRead(Gs);
	
	if(val_Gs == HIGH)
	{
		val_in_0 = digitalRead(in_0);
		val_in_1 = digitalRead(in_1);
		val_in_2 = digitalRead(in_2);
		val_in_3 = digitalRead(in_3);	
		
		send_data(val_in_0, val_in_1, val_in_2, val_in_3);
	}
}


void send_data(boolean val_out_0, boolean val_out_1, boolean val_out_2, boolean val_out_3)
{
	digitalWrite(out_0, val_out_0);
	digitalWrite(out_1, val_out_1);
	digitalWrite(out_2, val_out_2);
	digitalWrite(out_3, val_out_3);
	digitalWrite(start_send, HIGH);
	
	delay(2500);
	
	digitalWrite(out_0, LOW);
	digitalWrite(out_1, LOW);
	digitalWrite(out_2, LOW);
	digitalWrite(out_3, LOW);
	digitalWrite(start_send, LOW);
	
	return;
}
