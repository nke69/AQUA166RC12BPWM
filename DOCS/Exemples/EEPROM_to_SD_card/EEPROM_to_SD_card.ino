#include <SdFat.h>
#include <EEPROM.h>

const byte SelectSlave_SD = 5; // For Ferduino Mega
//const byte SelectSlave_SD = 4; // For ethernet shield
const byte SelectSlave_ETH = 53;
const byte SelectSlave_RFM = 69;

//#define RESTORE // Uncomment this line to copy values from .TXT to EEPROM.
//#define BACKUP  // Uncomment this line to copy values from EEPROM to .TXT

SdFat SD;
SdFile file;

void setup()
{
	int16_t n;
	byte buff[3];
	int i = 0;
	byte k = 0;
	byte value;
	
	pinMode(SelectSlave_ETH, OUTPUT);
	pinMode(SelectSlave_RFM, OUTPUT);
	digitalWrite(SelectSlave_RFM, HIGH);
	digitalWrite(SelectSlave_ETH, HIGH);

	Serial.begin(9600);

    SD.begin(SelectSlave_SD, SPI_HALF_SPEED);

	while(!Serial)
	{
		; // wait for serial.
	}

#ifdef BACKUP
	if(file.open("BACKUP.TXT", O_WRITE))
	{
		Serial.println("Old file deleted.");
		Serial.println();
		file.remove();
	}

	if(file.open("BACKUP.TXT", O_CREAT | O_APPEND | O_WRITE))
	{
		Serial.println("Reading and writing...");
		Serial.println();
		for (i = 0; i < 4096; i++)
		{
			n = EEPROM.read(i);
			file.print(n, DEC);
			file.print(',');
		}
		file.close();
		Serial.print("Finished.");
	}
	else
	{
		Serial.println("Can't open!");
	}
#endif

#ifdef RESTORE
	if(file.open("BACKUP.TXT", O_READ))
	{
		Serial.println("Reading and writing...");
		Serial.println();

		while ((n = file.read()) > 0)
		{

			if ((n != 44) && (k <= 2))
			{
				n -= '0';
				buff[k] = n;
				k++;
			}
			else
			{
				value = ((buff[0] * 100) + (buff[1] * 10) + buff[2]);

				if (k == 2)
				{
					value /= 10;
				}
				else if (k == 1)
				{
					value /= 100;
				}
				EEPROM.write(i, value);
				i++;
				k = 0;
				buff[0] = 0;
				buff[1] = 0;
				buff[2] = 0;
			}
		}
		file.close();
		Serial.print("Finished.");
	}
	else
	{
		Serial.print("Can't open!");
	}
#endif
}
void loop()
{
}
