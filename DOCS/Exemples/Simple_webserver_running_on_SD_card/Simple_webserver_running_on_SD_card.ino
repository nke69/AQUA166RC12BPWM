#include <SPI.h>
#include <Ethernet.h>
#include <SdFat.h>

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 177); // IP address, may need to change depending on network
EthernetServer server(80);

const byte SelectSlave_ETH = 53;
//const byte SelectSlave_SD = 4; // For ethernet shield
const byte SelectSlave_SD = 5;   // For Ferduino Mega 2560
const byte SelectSlave_RFM = 69;   // For Ferduino Mega 2560

SdFat SD;
SdFile file;

void setup()
{
	pinMode(SelectSlave_RFM, OUTPUT);
	digitalWrite(SelectSlave_RFM, HIGH);
	
	Serial.begin(9600);
	while (!Serial)
	{
		; // wait for serial port to connect. Needed for Leonardo only
	}
	Ethernet.begin(mac, ip, SelectSlave_ETH);
	server.begin();
	Serial.print("server is at ");
	Serial.println(Ethernet.localIP());

	Serial.println("Initializing SD card...");

	if (!SD.begin(SelectSlave_SD, SPI_HALF_SPEED))
	{
		Serial.println("ERROR - SD card initialization failed!");
		return;
	}

	Serial.println("SUCCESS - SD card initialized.");

	if (!file.open("index.htm", O_READ))
	{
		Serial.println("ERROR - Can't open index.htm file!");
		return;
	}
	file.close();
	Serial.println("SUCCESS - Found index.htm file.");
}

void loop()
{
	EthernetClient client = server.available();

	if (client)
	{
		boolean currentLineIsBlank = true;
		while (client.connected())
		{
			if (client.available())
			{
				char c = client.read();

				if (c == '\n' && currentLineIsBlank)
				{
					client.println("HTTP/1.1 200 OK");
					client.println("Content-Type: text/html");
					client.println("Connection: close");
					client.println();

					if (file.open("index.htm", O_READ))
					{
						while(file.available())
						{
							client.write(file.read());
						}
						file.close();
					}
					break;
				}
				if (c == '\n')
				{
					currentLineIsBlank = true;
				}
				else if (c != '\r')
				{
					currentLineIsBlank = false;
				}
			}
		}
		delay(1);
		client.stop();
	}
}

