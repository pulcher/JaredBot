
#include <Arduino.h>

#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

static constexpr uint16_t kHttpPort = 80;
static constexpr uint32_t kHelloIntervalMs = 2000;
static constexpr uint32_t kStaConnectTimeoutMs = 60000;

static constexpr const char *kPrefsNamespace = "serial_relay";
static constexpr const char *kPrefsStaSsidKey = "sta_ssid";
static constexpr const char *kPrefsStaPassKey = "sta_pass";

static WebServer server(kHttpPort);
static Preferences prefs;

static String apSsid;
static bool wifiStaConnected = false;
static IPAddress wifiStaIp;
static uint32_t uart_rx_bytes = 0;
static uint32_t uart_tx_bytes = 0;

static String storedStaSsid;

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
