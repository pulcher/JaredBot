
#include <Arduino.h>

void setup()
{
	Serial.begin(115200);
	delay(250);

	Serial.println();
	Serial.println("Hello, world! (PlatformIO serial_relay)");
}

void loop()
{
	Serial.print("Hello, world! uptime_ms=");
	Serial.println(millis());
	delay(2000);
}
