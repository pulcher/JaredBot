
#include <Arduino.h>

#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

#include <WebSocketsServer.h>

static constexpr uint16_t kHttpPort = 80;
static constexpr uint16_t kWsPort = 81;
static constexpr uint32_t kHeartbeatIntervalMs = 15000;
static constexpr uint32_t kStaConnectTimeoutMs = 60000;

static constexpr uint32_t kOledRefreshIntervalMs = 500;

static constexpr uint8_t kOledWidth = 128;
static constexpr uint8_t kOledHeight = 32;
static constexpr uint8_t kOledI2cAddress = 0x3C;

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

static WebSocketsServer wsServer(kWsPort);

static Adafruit_SSD1306 oled(kOledWidth, kOledHeight, &Wire, -1);
static bool oledOk = false;

static HardwareSerial unoSerial(2);

static String apSsid;
static bool wifiStaConnected = false;
static IPAddress wifiStaIp;
static uint32_t uart_rx_bytes = 0;
static uint32_t uart_tx_bytes = 0;

static String storedStaSsid;

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

		Serial.print("WS> forwarding ");
		Serial.print(length);
		Serial.println(" bytes to UNO UART");
		size_t written = unoSerial.write(payload, length);
		if (payload[length - 1] != '\n')
			written += unoSerial.write((uint8_t)'\n');
		uart_tx_bytes += (uint32_t)written;
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

static void oledRenderStatus()
{
	if (!oledOk)
		return;

	IPAddress ip = wifiStaConnected ? WiFi.localIP() : WiFi.softAPIP();

	oled.clearDisplay();
	oled.setTextSize(1);
	oled.setTextColor(SSD1306_WHITE);

	oled.setCursor(0, 0);
	oled.print(wifiStaConnected ? "STA " : "AP  ");
	oled.print(ip);

	oled.setCursor(0, 12);
	oled.print("RX ");
	oled.print(uart_rx_bytes);

	oled.setCursor(0, 22);
	oled.print("TX ");
	oled.print(uart_tx_bytes);

	oled.display();
}

static void oledInit()
{
	Wire.begin();

	if (!oled.begin(SSD1306_SWITCHCAPVCC, kOledI2cAddress))
	{
		oledOk = false;
		Serial.println("OLED init failed (SSD1306)");
		return;
	}

	oledOk = true;
	oled.clearDisplay();
	oled.setTextSize(1);
	oled.setTextColor(SSD1306_WHITE);
	oled.setCursor(0, 0);
	oled.println("serial_relay");
	oled.println("Booting...");
	oled.display();
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
	delay(250);

	Serial.println();
	Serial.println("serial_relay boot");

	oledInit();
	unoSerialInit();

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

	wsServer.begin();
	wsServer.onEvent(wsOnEvent);

	Serial.print("HTTP server: http://");
	Serial.print(wifiStaConnected ? wifiStaIp : WiFi.softAPIP());
	Serial.println("/");
	Serial.print("WebSocket server: ws://");
	Serial.print(wifiStaConnected ? wifiStaIp : WiFi.softAPIP());
	Serial.print(":");
	Serial.print(kWsPort);
	Serial.println("/");

	oledRenderStatus();
}

void loop()
{
	server.handleClient();
	wsServer.loop();

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
				wsServer.broadcastTXT((uint8_t *)unoLineBuf, unoLineLen);
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
		Serial.println(uart_tx_bytes);
	}
}
