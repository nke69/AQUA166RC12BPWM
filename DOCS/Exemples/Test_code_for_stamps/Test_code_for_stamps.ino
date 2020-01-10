const int multiplexadorS0Pin = 16; // S0
const int multiplexadorS1Pin = 17; // S1

String inputstring = "";                                                       
String sensorstring = "";                                                     
boolean input_stringcomplete = false;                                       
boolean sensor_stringcomplete = false;
int index = 0;
char inData[2];

void setup()
{                                                       
  Serial.begin(38400);                                             
  Serial3.begin(38400);                                               
  inputstring.reserve(5);                                               
  sensorstring.reserve(30);                                             
  pinMode(multiplexadorS0Pin, OUTPUT);
  pinMode(multiplexadorS1Pin, OUTPUT);

  Serial.println("To communication with the stamp of PH of the tank enter: y0");

  Serial.println("To communication with the stamp of PH of the reactor enter: y1");

  Serial.println("To communication with the stamp of ORP enter: y2");

  Serial.println("To communication with the stamp of EC enter: y3");

  Serial.println("To clear the channel opened enter: 00");
}

void serialEvent()
{                                                       
  char inchar = (char)Serial.read();                           
  inputstring += inchar;                                     
  inData[index] = inchar;
  index++; 

  if((inData[0] == 'y') && (inData[1] == '0'))
  {
    digitalWrite(multiplexadorS0Pin, LOW);
    digitalWrite(multiplexadorS1Pin, LOW);
    Serial.println("Channel for PH of the tank is opened.");
    inData[0] = '9';
    inData[1] = '9';
  }
  if((inData[0] == 'y') && (inData[1] == '1'))
  {
    digitalWrite(multiplexadorS0Pin, HIGH);
    digitalWrite(multiplexadorS1Pin, LOW);
    Serial.println("Channel for PH of the reactor is opened.");
    inData[0] = '9';
    inData[1] = '9';
  }
  if((inData[0] == 'y') && (inData[1] == '2'))
  {
    digitalWrite(multiplexadorS0Pin, LOW);
    digitalWrite(multiplexadorS1Pin, HIGH);
    Serial.println("Channel for ORP is opened.");
    inData[0] = '9';
    inData[1] = '9';
  }
  if((inData[0] == 'y') && (inData[1] == '3'))
  {
    digitalWrite(multiplexadorS0Pin, HIGH);
    digitalWrite(multiplexadorS1Pin, HIGH);
    Serial.println("Channel for EC is opened.");
    inData[0] = '9';
    inData[1] = '9';
  }
  if((inData[0] == '0') && (inData[1] == '0'))
  {
    Serial3.flush();
    Serial.println("Clean.");
    inData[0] = '9';
    inData[1] = '9';
  }
  if(inchar == '\r')
  {
    input_stringcomplete = true;
  }               
} 

void loop()
{                                                                   
  if (input_stringcomplete)
  {                                                   
    Serial3.print(inputstring);                                           
    inputstring = "";                                                       
    input_stringcomplete = false;                                                                         
    index = 0;
  }

  while (Serial3.available())
  {                                             
    char inchar = (char)Serial3.read();                                 
    sensorstring += inchar;                                             
    if (inchar == '\r')
    {
      sensor_stringcomplete = true;
    }                   
  }

  if (sensor_stringcomplete)
  {                                                 
    Serial.print(sensorstring);                                             
    sensorstring = "";   
    sensor_stringcomplete = false;                                         
    Serial.println("");
  }
}
