
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <U8g2lib.h>

#include <Preferences.h>
#include <WebServer.h>

// Debug/feature switch: disable WebSockets without removing code.
// Set to 1 to enable WebSockets server compile + runtime.
#ifndef ENABLE_WEBSOCKETS
#define ENABLE_WEBSOCKETS 0
#endif

#if ENABLE_WEBSOCKETS
#include <WebSocketsServer.h>
#endif

static constexpr uint16_t kHttpPort = 80;
static constexpr uint16_t kWsPort = 81;
static constexpr uint32_t kHeartbeatIntervalMs = 15000;
static constexpr uint32_t kStaConnectTimeoutMs = 60000;

static constexpr uint32_t kOledRefreshIntervalMs = 500;

// When enabled, the device will show the OLED calibration screen and then
// stop (no WiFi/server/etc). This ensures the calibration screen can't be
// overwritten by later rendering.
#ifndef OLED_CALIBRATION_MODE
#define OLED_CALIBRATION_MODE 0
#endif

// OLED display width and height (0.42" OLEDs on ESP32-C3 SuperMini are commonly 72x40)
#define SCREEN_WIDTH 72
#define SCREEN_HEIGHT 40

// I2C pins for ESP32-C3 0.42" OLED boards (check your board's datasheet)
#define OLED_SDA 5
#define OLED_SCL 6
#define OLED_RESET U8X8_PIN_NONE  // Some boards don't have a reset pin

// Offsets (in pixels) for panels where the active area is shifted.
// If text is cut off or not visible, tweak these (e.g. try 2..10).
static constexpr int OLED_X_OFFSET = 0;
static constexpr int OLED_Y_OFFSET = 0;

// Create display object (HW I2C uses the global Wire instance)
U8G2_SSD1306_72X40_ER_F_HW_I2C display(U8G2_R0, OLED_RESET);

static constexpr uint8_t kOledI2cAddress = 0x3C;
static constexpr uint16_t kOledCalibAnimMs = 150;

// Debug switch: disable the UNO UART (Serial2) without removing code.
// Set to 1 to enable Serial2 init/read/write.
#ifndef ENABLE_UNO_UART
#define ENABLE_UNO_UART 0
#endif

// UART link to Arduino (newline-delimited text, logged to USB Serial).
// Update these pins to match your wiring.
static constexpr uint32_t kUnoUartBaud = 9600;
static constexpr int kUnoUartRxPin = 16; // ESP32 RX (connect to Arduino TX via level shift)
static constexpr int kUnoUartTxPin = 17; // ESP32 TX (connect to Arduino RX)
static constexpr size_t kUnoLineMax = 192;

static constexpr const char *kPrefsNamespace = "serial_relay";
static constexpr const char *kPrefsStaSsidKey = "sta_ssid";
static constexpr const char *kPrefsStaPassKey = "sta_pass";

static WebServer server(kHttpPort);
static Preferences prefs;

#if ENABLE_WEBSOCKETS
static WebSocketsServer wsServer(kWsPort);
#endif

static bool oledOk = false;

static HardwareSerial unoSerial(2);

static String apSsid;
static bool wifiStaConnected = false;
static IPAddress wifiStaIp;
static uint32_t uart_rx_bytes = 0;
static uint32_t uart_tx_bytes = 0;

static String storedStaSsid;

static void i2cScan()
{
	Serial.println("I2C scan:");
	uint8_t found = 0;
	for (uint8_t address = 1; address < 127; address++)
	{
		Wire.beginTransmission(address);
		uint8_t err = Wire.endTransmission();
		if (err == 0)
		{
			Serial.print("  - 0x");
			if (address < 16)
				Serial.print('0');
			Serial.println(address, HEX);
			found++;
		}
	}
	if (found == 0)
		Serial.println("  (none)");
}

static void serialPrintEscaped(const uint8_t *data, size_t length, size_t maxLen)
{
	static constexpr char kHex[] = "0123456789ABCDEF";

	size_t n = length;
	if (n > maxLen)
		n = maxLen;

	for (size_t i = 0; i < n; i++)
	{
		uint8_t b = data[i];
		switch (b)
		{
		case '\\':
			Serial.print("\\\\");
			break;
		case '\r':
			Serial.print("\\r");
			break;
		case '\n':
			Serial.print("\\n");
			break;
		case '\t':
			Serial.print("\\t");
			break;
		default:
			if (b >= 0x20 && b <= 0x7E)
			{
				Serial.write((char)b);
			}
			else
			{
				char out[4] = {'\\', 'x', kHex[b >> 4], kHex[b & 0x0F]};
				Serial.write((const uint8_t *)out, sizeof(out));
			}
			break;
		}
	}

	if (length > maxLen)
	{
		Serial.print("... (truncated, total=");
		Serial.print(length);
		Serial.print(" bytes)");
	}
}

static void unoSerialInit()
{
	unoSerial.begin(kUnoUartBaud, SERIAL_8N1, kUnoUartRxPin, kUnoUartTxPin);
	Serial.print("UNO UART: Serial2 baud=");
	Serial.print(kUnoUartBaud);
	Serial.print(" RX=");
	Serial.print(kUnoUartRxPin);
	Serial.print(" TX=");
	Serial.println(kUnoUartTxPin);
}

#if ENABLE_WEBSOCKETS
static void wsOnEvent(uint8_t clientId, WStype_t type, uint8_t *payload, size_t length)
{
	switch (type)
	{
	case WStype_CONNECTED:
		Serial.print("WS client connected id=");
		Serial.println(clientId);
		break;
	case WStype_DISCONNECTED:
		Serial.print("WS client disconnected id=");
		Serial.println(clientId);
		break;
	case WStype_TEXT:
	{
		Serial.print("WS TEXT id=");
		Serial.print(clientId);
		Serial.print(" len=");
		Serial.print(length);
		Serial.print(" > ");
		if (payload && length)
			serialPrintEscaped(payload, length, 256);
		Serial.println();

		// Treat incoming text as a single line to forward to the Uno.
		// Forward the payload exactly as received (no trimming), and ensure it is newline-terminated.
		if (length == 0)
			break;
		if (!payload)
			break;
		if (length >= kUnoLineMax)
		{
			Serial.print("WS> (dropped: line too long len=");
			Serial.print(length);
			Serial.println(')');
			break;
		}

		#if ENABLE_UNO_UART
		Serial.print("WS> forwarding ");
		Serial.print(length);
		Serial.println(" bytes to UNO UART");
		size_t written = unoSerial.write(payload, length);
		if (payload[length - 1] != '\n')
			written += unoSerial.write((uint8_t)'\n');
		uart_tx_bytes += (uint32_t)written;
		#else
		Serial.println("WS> UNO UART disabled; not forwarding");
		#endif
		break;
	}
	case WStype_BIN:
		Serial.print("WS BIN id=");
		Serial.print(clientId);
		Serial.print(" len=");
		Serial.print(length);
		Serial.print(" > ");
		if (payload && length)
			serialPrintEscaped(payload, length, 128);
		Serial.println();
		break;
	default:
		break;
	}
}
#endif

static void oledRenderStatus()
{
	if (!oledOk)
		return;

	const int16_t originX = OLED_X_OFFSET;
	const int16_t originY = OLED_Y_OFFSET;

	IPAddress ip = wifiStaConnected ? WiFi.localIP() : WiFi.softAPIP();

	display.clearBuffer();
	display.setDrawColor(1);
	display.setFont(u8g2_font_6x10_tf);
	display.setFontPosTop();

	// Top-aligned Y coordinates for 6x10 font.
	const uint8_t line1Y = 0;
	const uint8_t line2Y = 12;
	const uint8_t line3Y = 24;

	display.setCursor(originX + 0, originY + line1Y);
	display.print(wifiStaConnected ? "STA" : "AP");

	// Fit IP string to the screen width (72px at 6px/char => 12 chars).
	String ipStr = ip.toString();
	const size_t maxChars = (size_t)(SCREEN_WIDTH / 6);
	while (ipStr.length() > maxChars)
	{
		int dot = ipStr.indexOf('.');
		if (dot < 0)
			break;
		ipStr = ipStr.substring((size_t)dot + 1);
	}
	if (ipStr.length() > maxChars)
		ipStr = ipStr.substring(ipStr.length() - (int)maxChars);

	display.setCursor(originX + 0, originY + line2Y);
	display.print(ipStr);

	// Compact counters on the last line.
	// oled.setCursor(originX + 0, originY + line3Y);
	// oled.print("R");
	// oled.print(uart_rx_bytes);
	// oled.print(" T");
	// oled.print(uart_tx_bytes);

	display.sendBuffer();
}

static void oledInit()
{
	// Initialize I2C with custom pins
	Wire.begin(OLED_SDA, OLED_SCL);
	i2cScan();

	display.setI2CAddress((uint8_t)(kOledI2cAddress << 1));
	oledOk = display.begin();
	if (!oledOk)
	{
		Serial.println("OLED init failed (U8g2)");
		return;
	}

	display.setContrast(255);

	// Unmistakable boot flash: full white -> full black
	display.clearBuffer();
	display.setDrawColor(1);
	display.drawBox(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	display.sendBuffer();
	delay(200);
	display.clearBuffer();
	display.sendBuffer();
	delay(200);

	// Obvious calibration visuals: edge markers + offset/geometry readout.
	// Thick left/right bars and top/bottom lines make it easy to see cropping/offset.
	const int16_t originX = OLED_X_OFFSET;
	const int16_t originY = OLED_Y_OFFSET;

	// Primary border (offset-aware)
	display.clearBuffer();
	display.setDrawColor(1);
	display.drawBox(originX + 0, originY + 0, 2, SCREEN_HEIGHT);
	display.drawBox(originX + SCREEN_WIDTH - 2, originY + 0, 2, SCREEN_HEIGHT);
	display.drawHLine(originX + 0, originY + 0, SCREEN_WIDTH);
	display.drawHLine(originX + 0, originY + SCREEN_HEIGHT - 1, SCREEN_WIDTH);

	display.setFont(u8g2_font_6x10_tf);
	display.setFontPosTop();

	// Text origin inside the border (avoid drawing under the thick border).
	const int16_t textOriginX = originX + 2;
	const int16_t textOriginY = originY + 1;

	// Corner/edge labels (help identify which edges are off-screen)
	display.setCursor(textOriginX + 0, textOriginY + 0);
	display.print('L');
	display.setCursor(originX + SCREEN_WIDTH - 8, textOriginY + 0);
	display.print('R');
	display.setCursor(originX + (SCREEN_WIDTH / 2) - 3, originY + 0);
	display.print('T');
	display.setCursor(originX + (SCREEN_WIDTH / 2) - 3, originY + SCREEN_HEIGHT - 10);
	display.print('B');

	// Origin crosshair at (0,0) in our logical buffer
	display.drawVLine(originX + 0, originY + 0, 12);
	display.drawHLine(originX + 0, originY + 0, 12);

	// Ruler ticks every 6px horizontally (character cell width)
	for (int16_t x = 0; x < SCREEN_WIDTH; x += 6)
	{
		display.drawVLine(originX + x, originY + 0, (x % 12 == 0) ? 6 : 3);
	}

	// Redundant markers every 16px (helps when we don't know the true visible origin)
	for (int16_t x = 0; x < SCREEN_WIDTH; x += 16)
		display.drawVLine(originX + x, originY + 0, SCREEN_HEIGHT);
	for (int16_t y = 0; y < SCREEN_HEIGHT; y += 16)
		display.drawHLine(originX + 0, originY + y, SCREEN_WIDTH);

	// Geometry/offset readout (kept away from animation and borders).
	display.setCursor(textOriginX + 12, textOriginY + 10);
	display.print(SCREEN_WIDTH);
	display.print('x');
	display.print(SCREEN_HEIGHT);
	display.print(" X");
	display.print(OLED_X_OFFSET);
	display.print(" Y");
	display.print(OLED_Y_OFFSET);

	// Fill the screen with a repeating 0-9 pattern.
	// Start below the geometry line so we can always see some known text.
	const int16_t cols = (SCREEN_WIDTH - 2) / 6;
	const int16_t rows = SCREEN_HEIGHT / 10;
	const int16_t digitsStartY = textOriginY + 20;
	for (int16_t r = 0; r < rows; r++)
	{
		int16_t y = digitsStartY + r * 10;
		if (y > (originY + SCREEN_HEIGHT - 10))
			break;

		display.setCursor(textOriginX, y);

		// First character: row number (wraps 0..9)
		display.print((char)('0' + (r % 10)));

		// Remaining: 0..9 repeating to the end of the line
		for (int16_t c = 1; c < cols; c++)
			display.print((char)('0' + ((c - 1) % 10)));
	}

	display.sendBuffer();

	// NOTE: Intentionally no animation here. The calibration screen is meant to
	// be static and predictable so text can't be erased by later drawing.
}

static bool loadStaCredentials(String &ssidOut, String &passOut)
{
	prefs.begin(kPrefsNamespace, true);
	ssidOut = prefs.getString(kPrefsStaSsidKey, "");
	passOut = prefs.getString(kPrefsStaPassKey, "");
	prefs.end();

	ssidOut.trim();
	// Password may be empty for open networks.
	return ssidOut.length() > 0;
}

static void saveStaCredentials(const String &ssid, const String &pass)
{
	prefs.begin(kPrefsNamespace, false);
	prefs.putString(kPrefsStaSsidKey, ssid);
	prefs.putString(kPrefsStaPassKey, pass);
	prefs.end();
}

static String htmlEscape(const String &input)
{
	String out;
	out.reserve(input.length());
	for (size_t i = 0; i < input.length(); i++)
	{
		switch (input[i])
		{
		case '&': out += "&amp;"; break;
		case '<': out += "&lt;"; break;
		case '>': out += "&gt;"; break;
		case '"': out += "&quot;"; break;
		case '\'': out += "&#39;"; break;
		default: out += input[i]; break;
		}
	}
	return out;
}

static void handleRoot()
{
	IPAddress apIp = WiFi.softAPIP();

	String body;
	body.reserve(512);
	body += "<!doctype html><html><head><meta charset='utf-8'>";
	body += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
	body += "<title>serial_relay status</title></head><body>";
	body += "<h1>serial_relay status</h1>";
	body += "<p><a href='/config'>WiFi config</a></p>";
	body += "<ul>";
	body += "<li>WiFi mode: <code>" + String(wifiStaConnected ? "STA" : "AP") + "</code></li>";
	if (wifiStaConnected)
	{
		body += "<li>STA SSID: <code>" + htmlEscape(String(WiFi.SSID())) + "</code></li>";
		body += "<li>STA IP: <code>" + wifiStaIp.toString() + "</code></li>";
	}
	else
	{
		body += "<li>AP SSID: <code>" + htmlEscape(apSsid) + "</code></li>";
		body += "<li>AP IP: <code>" + apIp.toString() + "</code></li>";
	}
	body += "<li>Uptime (ms): <code>" + String(millis()) + "</code></li>";
	body += "<li>UART RX bytes: <code>" + String(uart_rx_bytes) + "</code></li>";
	body += "<li>UART TX bytes: <code>" + String(uart_tx_bytes) + "</code></li>";
	body += "</ul>";
	body += "</body></html>";

	server.send(200, "text/html; charset=utf-8", body);
}

static void handleNotFound()
{
	server.send(404, "text/plain; charset=utf-8", "Not found\n");
}

static void handleConfigGet()
{
	String savedSsid;
	String savedPass;
	bool hasSaved = loadStaCredentials(savedSsid, savedPass);

	String body;
	body.reserve(800);
	body += "<!doctype html><html><head><meta charset='utf-8'>";
	body += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
	body += "<title>serial_relay WiFi config</title></head><body>";
	body += "<h1>WiFi config</h1>";
	body += "<p>Configure STA (connect to existing WiFi). On save, device will reboot.</p>";
	body += "<form method='POST' action='/config'>";
	body += "<label>SSID<br><input name='ssid' value='" + htmlEscape(hasSaved ? savedSsid : String("")) + "' maxlength='64' style='width: 100%; max-width: 360px;'></label><br><br>";
	body += "<label>Password<br><input name='pass' type='password' value='' maxlength='64' style='width: 100%; max-width: 360px;'></label><br><br>";
	body += "<button type='submit'>Save</button>";
	body += "</form>";
	body += "<p><a href='/'>Back to status</a></p>";
	body += "</body></html>";

	server.send(200, "text/html; charset=utf-8", body);
}

static void handleConfigPost()
{
	String ssid = server.arg("ssid");
	String pass = server.arg("pass");
	ssid.trim();

	if (ssid.length() == 0)
	{
		server.send(400, "text/plain; charset=utf-8", "SSID is required\n");
		return;
	}

	saveStaCredentials(ssid, pass);

	server.send(200, "text/plain; charset=utf-8", "Saved. Rebooting...\n");
	Serial.println("WiFi credentials saved; rebooting");
	delay(500);
	ESP.restart();
}

void setup()
{
	Serial.begin(115200);
	delay(5000);

	Serial.println();
	Serial.println("serial_relay boot");
	Serial.println("FW: WALKING_CHAR_CAL_2026-03-25a");

	oledInit();

	#if OLED_CALIBRATION_MODE
	Serial.println("OLED calibration mode enabled; halting after oledInit()");
	while (true)
	{
		delay(1000);
	}
	#endif
	#if ENABLE_UNO_UART
	unoSerialInit();
	#else
	Serial.println("UNO UART disabled (ENABLE_UNO_UART=0)");
	#endif

	// M4: Try STA first, fall back to AP if STA fails.
	wifiStaConnected = false;
	storedStaSsid = "";

	String staSsid;
	String staPass;
	bool hasPrefsCreds = loadStaCredentials(staSsid, staPass);

	if (hasPrefsCreds)
	{
		WiFi.mode(WIFI_STA);
		WiFi.begin(staSsid.c_str(), staPass.c_str());
		Serial.print("STA connecting to SSID: ");
		Serial.println(staSsid);
		storedStaSsid = staSsid;

		uint32_t startMs = millis();
		while (WiFi.status() != WL_CONNECTED && (uint32_t)(millis() - startMs) < kStaConnectTimeoutMs)
		{
			delay(250);
			Serial.print('.');
		}
		Serial.println();

		wifiStaConnected = (WiFi.status() == WL_CONNECTED);
		if (wifiStaConnected)
		{
			wifiStaIp = WiFi.localIP();
			Serial.println("STA connected");
			Serial.print("STA IP: ");
			Serial.println(wifiStaIp);
		}
		else
		{
			Serial.println("STA connect failed; falling back to AP");
			WiFi.disconnect(true, true);
		}
	}
	else
	{
		Serial.println("No stored STA credentials; starting AP");
	}

	if (!wifiStaConnected)
	{
		WiFi.mode(WIFI_AP);
		uint64_t mac = ESP.getEfuseMac();
		apSsid = "SerialRelay-" + String((uint32_t)(mac & 0xFFFFFF), HEX);
		apSsid.toUpperCase();

		// Open AP by default (no password). Add a password later if desired.
		bool apOk = WiFi.softAP(apSsid.c_str());
		IPAddress apIp = WiFi.softAPIP();

		Serial.print("AP start: ");
		Serial.println(apOk ? "OK" : "FAILED");
		Serial.print("AP SSID: ");
		Serial.println(apSsid);
		Serial.print("AP IP: ");
		Serial.println(apIp);
	}

	server.on("/config", HTTP_GET, handleConfigGet);
	server.on("/config", HTTP_POST, handleConfigPost);
	server.on("/", HTTP_GET, handleRoot);
	server.onNotFound(handleNotFound);
	server.begin();

	#if ENABLE_WEBSOCKETS
	wsServer.begin();
	wsServer.onEvent(wsOnEvent);
	#endif

	Serial.print("HTTP server: http://");
	Serial.print(wifiStaConnected ? wifiStaIp : WiFi.softAPIP());
	Serial.println("/");
	#if ENABLE_WEBSOCKETS
	Serial.print("WebSocket server: ws://");
	Serial.print(wifiStaConnected ? wifiStaIp : WiFi.softAPIP());
	Serial.print(":");
	Serial.print(kWsPort);
	Serial.println("/");
	#else
	Serial.println("WebSocket server disabled (ENABLE_WEBSOCKETS=0)");
	#endif

	oledRenderStatus();
}

void loop()
{
	server.handleClient();
	#if ENABLE_WEBSOCKETS
	wsServer.loop();
	#endif

	#if ENABLE_UNO_UART
	// Read from Arduino UART and print complete lines to USB Serial.
	static char unoLineBuf[kUnoLineMax];
	static size_t unoLineLen = 0;
	while (unoSerial.available() > 0)
	{
		int c = unoSerial.read();
		if (c < 0)
			break;

		uart_rx_bytes++;
		char ch = (char)c;

		if (ch == '\r')
			continue;

		if (ch == '\n')
		{
			unoLineBuf[unoLineLen] = '\0';
			if (unoLineLen > 0)
			{
				// Serial.print("UNO> ");
				// Serial.println(unoLineBuf);
				#if ENABLE_WEBSOCKETS
				wsServer.broadcastTXT((uint8_t *)unoLineBuf, unoLineLen);
				#endif
			}
			unoLineLen = 0;
			continue;
		}

		if (unoLineLen < (kUnoLineMax - 1))
		{
			unoLineBuf[unoLineLen++] = ch;
		}
		else
		{
			unoLineLen = 0;
			Serial.println("UNO> (line dropped: too long)");
		}
	}
	#endif

	static uint32_t lastOledMs = 0;
	uint32_t now = millis();
	if ((uint32_t)(now - lastOledMs) >= kOledRefreshIntervalMs)
	{
		lastOledMs = now;
		oledRenderStatus();
	}

	static uint32_t lastHelloMs = 0;
	if ((uint32_t)(now - lastHelloMs) >= kHeartbeatIntervalMs)
	{
		lastHelloMs = now;
		Serial.print("Heartbeat uptime_ms=");
		Serial.print(now);
		Serial.print(" wifi=");
		Serial.print(wifiStaConnected ? "STA" : "AP");
		Serial.print(" ip=");
		Serial.print(wifiStaConnected ? WiFi.localIP() : WiFi.softAPIP());
		Serial.print(" rx=");
		Serial.print(uart_rx_bytes);
		Serial.print(" tx=");
		Serial.print(uart_tx_bytes);
		Serial.print(" display connected= ");
		Serial.print(oledOk ? "YES" : "NO");
		Serial.println();
	}
}
