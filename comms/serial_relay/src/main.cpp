
#include <Arduino.h>

#include <WiFi.h>
#include <WebServer.h>

static constexpr uint16_t kHttpPort = 80;
static constexpr uint32_t kHelloIntervalMs = 2000;
static constexpr uint32_t kStaConnectTimeoutMs = 60000;

// M4/M6 stepping stone: STA credentials are compile-time for now.
// These will be replaced by persisted config in M5.
static constexpr const char *kStaSsid = "";
static constexpr const char *kStaPassword = "";

static WebServer server(kHttpPort);

static String apSsid;
static bool wifiStaConnected = false;
static IPAddress wifiStaIp;
static uint32_t uart_rx_bytes = 0;
static uint32_t uart_tx_bytes = 0;

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

void setup()
{
	Serial.begin(115200);
	delay(250);

	Serial.println();
	Serial.println("serial_relay boot");

	// M4 (tweaked): Try STA first, fall back to AP if STA fails.
	wifiStaConnected = false;

	if (strlen(kStaSsid) > 0)
	{
		WiFi.mode(WIFI_STA);
		WiFi.begin(kStaSsid, kStaPassword);
		Serial.print("STA connecting to SSID: ");
		Serial.println(kStaSsid);

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
		// If the ESP32 has previously stored STA credentials in flash, WiFi.begin()
		// (with no args) will attempt to reconnect to them.
		WiFi.mode(WIFI_STA);
		WiFi.begin();
		Serial.println("STA SSID not set; attempting saved STA credentials");

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
			Serial.println("STA connected (saved credentials)");
			Serial.print("STA IP: ");
			Serial.println(wifiStaIp);
		}
		else
		{
			Serial.println("No saved STA connection; starting AP");
			WiFi.disconnect(true, true);
		}
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

	server.on("/", HTTP_GET, handleRoot);
	server.onNotFound(handleNotFound);
	server.begin();

	Serial.print("HTTP server: http://");
	Serial.print(wifiStaConnected ? wifiStaIp : WiFi.softAPIP());
	Serial.println("/");
}

void loop()
{
	server.handleClient();

	static uint32_t lastHelloMs = 0;
	uint32_t now = millis();
	if ((uint32_t)(now - lastHelloMs) >= kHelloIntervalMs)
	{
		lastHelloMs = now;
		Serial.print("Hello, world! uptime_ms=");
		Serial.println(now);
	}
}
