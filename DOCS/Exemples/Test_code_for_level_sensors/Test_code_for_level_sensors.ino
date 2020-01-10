const byte sensor1 = 54; // A0;       
const byte sensor2 = 55; // A1;       
const byte sensor3 = 56; // A2;       
const byte sensor4 = 57; // A3;     
const byte sensor5 = 58; // A4; 
const byte sensor6 = 59; // A5;   

void setup()
{
  Serial.begin(9600);
  while(!Serial)
  {
    ; // Wait for serial.
  }
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  pinMode(sensor6, INPUT);

}

void loop()
{
  int Value1 = analogRead(sensor1);
  int Value2 = analogRead(sensor2);
  int Value3 = analogRead(sensor3);
  int Value4 = analogRead(sensor4);
  int Value5 = analogRead(sensor5);
  int Value6 = analogRead(sensor6);

  Serial.print("Sensor 1: ");
  Serial.println(Value1);
  Serial.println();
  Serial.print("Sensor 2: ");
  Serial.println(Value2);
  Serial.println();
  Serial.print("Sensor 3: ");
  Serial.println(Value3);
  Serial.println();
  Serial.print("Sensor 4: ");
  Serial.println(Value4);
  Serial.println();
  Serial.print("Sensor 5: ");
  Serial.println(Value5);
  Serial.println();
  Serial.print("Sensor 6: ");
  Serial.println(Value6);
  Serial.println();
  Serial.println("*******************************");
  Serial.println();
  delay(5000);
}

