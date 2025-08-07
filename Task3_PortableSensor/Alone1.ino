#include <stdlib.h>
#include <vector>
#include <deque>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <esp_pthread.h>
#include <WiFi.h>
//	#include <DNSServer.h>
#include <PsychicHttp.h>
#include <PsychicHttpServer.h>
#include <PsychicHttpsServer.h>
#include <PsychicStreamResponse.h>
#include <RTClib.h>
#include <Adafruit_SSD1306.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT4x.h>

#include <Fonts/FreeSans9pt7b.h>
#define FONT_0 FreeSans9pt7b
#define FONT_0_OFFSET 16
#include <Fonts/FreeSerif9pt7b.h>
#define FONT_1 FreeSerif9pt7b
#define FONT_1_OFFSET 16

#include "config.h"

static String show_time(DateTime const *);
static void redraw_display(bool = false);

/* *************************************************************************** / ************************************ */

static std::mutex mutex_hardware;
#define DISPLAY_LOCK(lock) std::lock_guard<std::mutex> lock(mutex_hardware)
#define DEVICE_LOCK(lock) std::lock_guard<std::mutex> lock(mutex_hardware)
#define SDCARD_LOCK(lock)
static std::mutex mutex_data;
#define DATA_LOCK(lock) std::lock_guard<std::mutex> lock(mutex_data)

static std::mutex wait_measure_mutex;
static std::condition_variable wait_measure_condition;
static bool use_assigned_device_time = false;
static DateTime assigned_device_time;

static bool clock_synchronized = false;
static bool need_save = false;
static bool need_reboot = false;

static Adafruit_SSD1306 Monitor(128, 64);

static char const report_path[] = "report";
static char const upload_data_path[] = "upload/data";
static char const upload_position_path[] = "upload/position";

/* *************************************************************************** / ************************************ */
/* Data */

struct Field {
	char const *name;
	char const *unit;
};

struct Data {
	DateTime time;
	DateTime device_time;
	float temperature;
	float humidity;
};

static Field const data_fields[] = {
	{"time", nullptr},
	{"device time", nullptr},
	{"clock synchronized", nullptr},
	{"temperature", "\u2103"},
	{"humidity", "%"}
};

static size_t const data_meta = 3;

static String CSV_Data(struct Data const *const data) {
	return show_time(&data->time)
		+ ',' + show_time(&data->device_time)
		+ ',' + clock_synchronized
		+ ',' + data->temperature
		+ ',' + data->humidity;
}

struct GPS {
	DateTime time;
	DateTime browser_time;
	DateTime position_time;
	double latitude;
	double longitude;
	double altitude;
};

static Field const gps_fields[] = {
	{"time", nullptr},
	{"browser time", nullptr},
	{"position time", nullptr},
	{"latitude", "\u00B0"},
	{"longitude", "\u00B0"},
	{"altitude", "m"}
};

static size_t gps_meta = 3;

static String CSV_GPS(struct GPS const *const data) {
	return show_time(&data->time)
		+ ',' + show_time(&data->browser_time)
		+ ',' + show_time(&data->position_time)
		+ ',' + String(data->latitude,  7)
		+ ',' + String(data->longitude, 7)
		+ ',' + String(data->altitude,  7);
}

/* *************************************************************************** / ************************************ */
/* SD card */

static char const setting_filename[] = "/setting.txt";
static char const data_filename[] = "/data.csv";
static char const gps_filename[] = "/gps.csv";
static String data_header;
static String gps_header;

//	static SPIClass SPI_1(HSPI);
static bool SD_card_exist;

static void save_settings(void) {
	if (!SD_card_exist) return;
	SDCARD_LOCK(sdcard_lock);
	File file = SD.open(setting_filename, "w", true);
	if (!file) {
		Serial.println("ERROR: failed to open setting file");
		return;
	}
	file.println(campaign_name);
	file.println(organisation_name);
	file.println(device_name);
	file.println(measure_interval / 1000);
	file.println(int(use_AP_mode));
	file.println(AP_SSID);
	file.println(AP_PASS);
	file.println(STA_SSID);
	file.println(STA_PASS);
	file.println(monitor_URL);
	file.println(upload_password);
	file.close();
}

static bool load_settings(void) {
	char *e;
	String s;
	unsigned long int u;

	if (digitalRead(reset_pin) == HIGH) {
		Serial.println("Setting is not loaded because of hardware switch");
		return false;
	}
	SDCARD_LOCK(sdcard_lock);
	File file = SD.open(setting_filename, "r", false);
	if (!file) {
		Serial.println("Failed to open setting file");
		return false;
	}

	campaign_name = file.readStringUntil('\n');
	campaign_name.trim();
	organisation_name = file.readStringUntil('\n');
	organisation_name.trim();
	device_name = file.readStringUntil('\n');
	device_name.trim();
	s = file.readStringUntil('\n');
	s.trim();
	u = strtoul(s.c_str(), &e, 10);
	if (!*e && u >= 15 && u <= 900) measure_interval = u * 1000;
	s = file.readStringUntil('\n');
	s.trim();
	u = strtoul(s.c_str(), &e, 10);
	if (!*e) use_AP_mode = bool(u);
	AP_SSID = file.readStringUntil('\n');
	AP_SSID.trim();
	AP_PASS = file.readStringUntil('\n');
	AP_PASS.trim();
	STA_SSID = file.readStringUntil('\n');
	STA_SSID.trim();
	STA_PASS = file.readStringUntil('\n');
	STA_PASS.trim();
	monitor_URL = file.readStringUntil('\n');
	monitor_URL.trim();
	upload_password = file.readStringUntil('\n');
	upload_password.trim();

	file.close();
	return true;
}

/* *************************************************************************** / ************************************ */
/* Real-time clock */

static RTC_Millis internal_clock;
static bool internal_clock_available = false;
static RTC_DS3231 external_clock;
static bool external_clock_available = false;

static bool clock_available(void) {
	return external_clock_available || internal_clock_available;
}

static void set_time(DateTime const *const datetime) {
	if (external_clock_available) {
		DEVICE_LOCK(device_lock);
		external_clock.adjust(*datetime);
	}
	else {
		internal_clock.adjust(*datetime);
		internal_clock_available = true;
	}
	clock_synchronized = true;
}

static DateTime get_time(void) {
	if (external_clock_available) {
		DEVICE_LOCK(device_lock);
		return external_clock.now();
	}
	else
		return internal_clock.now();
}

static String show_time(DateTime const *const datetime) {
	if (datetime->isValid())
		return datetime->timestamp();
	else
		return String("?");
}

static DateTime round_up_time(DateTime const *const datetime, unsigned long int const interval = measure_interval) {
	uint64_t shifted = (uint64_t)datetime->unixtime() * 1000 + (interval >> 1);
	return DateTime((shifted - shifted % interval) / 1000);
}

/* *************************************************************************** / ************************************ */
/* Measurement */

static Adafruit_SHT4x SHT4x = Adafruit_SHT4x();;

static size_t const records_max_size = 60;
static std::deque<Data> data_records;
static std::deque<GPS> gps_records;

static DateTime measure(void) {
	Data data;
	if (clock_available())
		data.device_time = get_time();
	else
		data.device_time = DateTime(0, 0, 0) + TimeSpan(millis() / 1000);
	if (use_assigned_device_time) {
		data.time = assigned_device_time;
		use_assigned_device_time = false;
	}
	else
		data.time = round_up_time(&data.device_time);
	{
		DEVICE_LOCK(device_lock);
		sensors_event_t temperature_event, humidity_event;
		SHT4x.getEvent(&humidity_event, &temperature_event);
		data.temperature = temperature_event.temperature;
		data.humidity = humidity_event.relative_humidity;
	}
	String const data_string = CSV_Data(&data);
	Serial.print("INFO: Measure ");
	Serial.println(data_string);

	if (data_records.size() >= records_max_size)
		data_records.pop_front();
	data_records.push_back(data);

	if (SD_card_exist) {
		SDCARD_LOCK(sdcard_lock);
		File file = SD.open(data_filename, "a", true);
		try {
			if (!file.position())
				file.println(data_header);
			file.println(data_string);
		}
		catch (...) {
			Serial.println("ERROR: failed to write weather data into SD card");
		}
		file.close();
	}

	redraw_display(true);
	return data.device_time;
}

static void wait_to_measure(DateTime const now) {
	unsigned int const t1 = measure_interval - (uint64_t)now.unixtime() * 1000 % measure_interval;
	unsigned int const t2 = t1 < measure_interval >> 1 ? t1 + measure_interval : t1;
	std::unique_lock<std::mutex> wait_lock(wait_measure_mutex);
	wait_measure_condition.wait_for(wait_lock, std::chrono::duration<unsigned int, std::milli>(t2));
}

static void measure_thread(void) {
	wait_to_measure(get_time());
	for (;;)
		try {
			wait_to_measure(measure());
		}
		catch (...) {
			Serial.println("ERROR: exception in measurement");
		}
}

/* *************************************************************************** / ************************************ */
/* WiFi */

namespace WIFI {
	//	static DNSServer DNSd;

	static void handle_event(WiFiEvent_t const event) {
		switch (event) {
		case ARDUINO_EVENT_WIFI_READY:
			Serial.println("WiFi interface ready");
			break;
		case ARDUINO_EVENT_WIFI_SCAN_DONE:
			Serial.println("Completed scan for access points");
			break;
		case ARDUINO_EVENT_WIFI_STA_START:
			Serial.println("WiFi client started");
			break;
		case ARDUINO_EVENT_WIFI_STA_STOP:
			Serial.println("WiFi clients stopped");
			break;
		case ARDUINO_EVENT_WIFI_STA_CONNECTED:
			Serial.println("Connected to access point");
			break;
		case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
			Serial.println("Disconnected from WiFi access point");
			break;
		case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
			Serial.println("Authentication mode of access point has changed");
			break;
		case ARDUINO_EVENT_WIFI_STA_GOT_IP:
			Serial.print("Obtained IP address: ");
			Serial.println(WiFi.localIP());
			break;
		case ARDUINO_EVENT_WIFI_STA_LOST_IP:
			Serial.println("Lost IP address and IP address is reset to 0");
			break;
		case ARDUINO_EVENT_WPS_ER_SUCCESS:
			Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
			break;
		case ARDUINO_EVENT_WPS_ER_FAILED:
			Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
			break;
		case ARDUINO_EVENT_WPS_ER_TIMEOUT:
			Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
			break;
		case ARDUINO_EVENT_WPS_ER_PIN:
			Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
			break;
		case ARDUINO_EVENT_WIFI_AP_START:
			Serial.println("WiFi access point started");
			break;
		case ARDUINO_EVENT_WIFI_AP_STOP:
			Serial.println("WiFi access point  stopped");
			break;
		case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
			Serial.println("Client connected");
			break;
		case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
			Serial.println("Client disconnected");
			break;
		case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
			Serial.println("Assigned IP address to client");
			break;
		case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
			Serial.println("Received probe request");
			break;
		case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
			Serial.println("AP IPv6 is preferred");
			break;
		case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
			Serial.println("STA IPv6 is preferred");
			break;
		case ARDUINO_EVENT_ETH_GOT_IP6:
			Serial.println("Ethernet IPv6 is preferred");
			break;
		case ARDUINO_EVENT_ETH_START:
			Serial.println("Ethernet started");
			break;
		case ARDUINO_EVENT_ETH_STOP:
			Serial.println("Ethernet stopped");
			break;
		case ARDUINO_EVENT_ETH_CONNECTED:
			Serial.println("Ethernet connected");
			break;
		case ARDUINO_EVENT_ETH_DISCONNECTED:
			Serial.println("Ethernet disconnected");
			break;
		case ARDUINO_EVENT_ETH_GOT_IP:
			Serial.println("Obtained IP address");
			break;
		default:
			Serial.print("Unknown WiFi event ");
			Serial.println(event);
			break;
		}
	}

	static char const *status_message(signed int const WiFi_status) {
		switch (WiFi_status) {
		case WL_NO_SHIELD:
			return "WiFi no shield";
		case WL_IDLE_STATUS:
			return "WiFi idle";
		case WL_NO_SSID_AVAIL:
			return "WiFi no SSID";
		case WL_SCAN_COMPLETED:
			return "WiFi scan completed";
		case WL_CONNECTED:
			return "WiFi connected";
		case WL_CONNECT_FAILED:
			return "WiFi connect failed";
		case WL_CONNECTION_LOST:
			return "WiFi connection lost";
		case WL_DISCONNECTED:
			return "WiFi disconnected";
		default:
			return "WiFi Status: " + WiFi_status;
		}
	}

	static signed int WiFi_check_status(void) {
		static signed int last_status = WL_NO_SHIELD;
		signed int status = WiFi.status();
		if (status != last_status) {
			last_status = status;
			Serial.println(status_message(status));
			if (status == WL_CONNECTED) {
				String const SSID = WiFi.SSID();
				Serial.print("WiFi SSID: ");
				Serial.println(WiFi.SSID());
				Serial.print("IP address: ");
				WiFi.localIP().printTo(Serial);
				Serial.println();
			}
		}
		return status;
	}

	static void thread(void) {
		for (;;)
			try {
				delay(WiFi_check_interval);
				WiFi_check_status();
			}
			catch (...) {
				Serial.println("ERROR: exception in WiFi checking");
			}
	}

	static void setup(void) {
		WiFi.disconnect();
		WiFi.onEvent(handle_event);
		WiFi.setHostname("WeatherStation");

		if (use_AP_mode) {
			/* WiFi access-point */
			WiFi.mode(WIFI_AP);
			//	IPAddress my_IP_address = IPAddress(8, 8, 8, 8);
			//	WiFi.softAPConfig(my_IP_address, my_IP_address, IPAddress(255, 255, 255, 0));
			while (!WiFi.softAP(AP_SSID.c_str(), AP_PASS, 1, 0, 4)) {
				Serial.println("ERROR: failed to create soft AP");
				Monitor.println("ERROR: WiFi AP");
				Monitor.display();
				delay(reinitialize_interval);
			}
			Serial.print("WiFi SSID: ");
			Serial.println(WiFi.softAPSSID());
			Serial.print("IP address: ");
			WiFi.softAPIP().printTo(Serial);
			Serial.println();
			Monitor.print("AP:");
			Monitor.println(WiFi.softAPSSID());
			WiFi.softAPIP().printTo(Monitor);
			Monitor.println();
			Monitor.display();

			/* DNS server */
			//	static uint16_t const DNS_port = 53;
			//	static String const DNS_domain("*");
			//	while (!DNSd.start(DNS_port, DNS_domain, my_IP_address)) {
			//		Serial.println("ERROR: failed to create DNS server");
			//		Monitor.println("ERROR: DNS server");
			//		delay(reinitialize_interval);
			//	}
		}
		else {
			/* WiFi stationary */
			WiFi.mode(WIFI_STA);
			WiFi.begin(STA_SSID, STA_PASS);
			while (WiFi.status() == WL_NO_SHIELD) {
				Serial.println("ERROR: no WiFi shield");
				Monitor.println("No WiFi shield");
				Monitor.display();
				delay(reinitialize_interval);
			}
			/* Spawn WiFi thread */
			set_pthread_stack_size(4096);
			std::thread(thread).detach();
		}
	}
}

/* *************************************************************************** / ************************************ */
/* Web server */

namespace WEB {
	static PsychicHttpServer HTTPd;
	static PsychicHttpsServer HTTPSd;

	static PROGMEM char const tls_key[] =
R"(-----BEGIN PRIVATE KEY-----
MIICdwIBADANBgkqhkiG9w0BAQEFAASCAmEwggJdAgEAAoGBAMNgxUb2U45aziNv
J+VWYP0CUOvjVwxe9pW8lrSevsX+thjYHiU3OOridQ/Q/GocJaZgCQDFAnL54FYN
pSmSIXtIuwjWEk+nYK7chDSwnkZ191dIQGTqI7BWglckqHc4y2auWD7JZCGIEFjK
yEyraHv2S2cAd0pwlWcUCICgte0PAgMBAAECgYEAicaI92SnQYCpUvWEtcX2+RQU
CnQzo2aoDqmBwPcc4rSepuBoSagqfACbuj6OcSlOJ4gbcS58bqXk2+odaTZCYtlC
+3ME9A+WNQCVjHm8qXugLyuw6LHHQkKZ0D5T4uiTCQCF4eGkfPwke3o8/H/0Dzqe
l49zwfT/xxHL16I4KgECQQDk1Fe0jVLjhVYbmODB3Zp81oktxo283oaTLnCMO09M
l9sM1WzZXS7ZiJe5jq6LY8ysiQMUMxl1TAxbGT+WZSUBAkEA2pOdXG78FhYuojpz
hwF8eMoQ3p+BfZ+4R5K+y2jeANoui4JrQ2XC23UJ9I51nzFO+7BlyKynvZXf0Ai4
tL7CDwJAAvbhP/yIs1vZ1revSbOmObHJyycEVQsI8UUrvhVSnKpm8w6cv2AeqEDF
vmijyDh9wUpxGMTksolOq6tzEG61AQJBAKRUhuKProcMdlMRjvnZbDOD99roIPrJ
skpdUYSsevw5DPVmQC6Tu0QzYiCzWkstTyx7GosdA5/Npk9Jv1RkdpECQCS/fBnl
nY7gZGKfoTbwiTBjAvRt+zfX7Ur0CUCYrZpTNTKtHNFh/8bImR+nnhGMqWV5DCUH
2yH9XHBPj+9AXdA=
-----END PRIVATE KEY-----
)";

	static PROGMEM char const tls_cert[] =
R"(-----BEGIN CERTIFICATE-----
MIICAjCCAWsCFDEQA8xXkoJCKRZu7sn1fd0h9AsgMA0GCSqGSIb3DQEBCwUAMEAx
CzAJBgNVBAYTAkhLMQwwCgYDVQQKDANIS1UxDTALBgNVBAsMBEdlb2cxFDASBgNV
BAMMCzE5Mi4xNjguNC4xMB4XDTI0MDgwODA4MzEwM1oXDTI0MDkwNzA4MzEwM1ow
QDELMAkGA1UEBhMCSEsxDDAKBgNVBAoMA0hLVTENMAsGA1UECwwER2VvZzEUMBIG
A1UEAwwLMTkyLjE2OC40LjEwgZ8wDQYJKoZIhvcNAQEBBQADgY0AMIGJAoGBAMNg
xUb2U45aziNvJ+VWYP0CUOvjVwxe9pW8lrSevsX+thjYHiU3OOridQ/Q/GocJaZg
CQDFAnL54FYNpSmSIXtIuwjWEk+nYK7chDSwnkZ191dIQGTqI7BWglckqHc4y2au
WD7JZCGIEFjKyEyraHv2S2cAd0pwlWcUCICgte0PAgMBAAEwDQYJKoZIhvcNAQEL
BQADgYEAaruQki3Ot6X3wuu26HdA9V0S+ZwjrjOAaRPO1liO1qkkNyXKenL6yRpC
i2lVLW/CxmGgT6nT64MFUh4wIlZmbNuK+yTNdCaJI7I713YEjDAuomzS/myPyLDm
fPQsAPOfXW3x3SDYwVp+V8rcl/xegEyo1BQaKbTloCEKFmNfEAI=
-----END CERTIFICATE-----
)";

	static String const XHTML_content_type = "application/xhtml+xml; charset=UTF-8";

	static String javascript_escape(String const &string) {
		String result;
		for (char const c: string)
			switch (c) {
			case '\"':
				result.concat("\\\"");
				break;
			case '\'':
				result.concat("\\\'");
				break;
			default:
				result.concat(c);
			}
		return result;
	}

	static PROGMEM char const home_html_1[] =
R"HTML(<html xmlns='http://www.w3.org/1999/xhtml'>
<head>
<meta content-type='application/xhtml+xml; charset=UTF-8' />
<meta charset='UTF-8' />
<meta name='viewport' content='width=device-width, initial-scale=1' />
<title>Weather data</title>
<link rel='stylesheet' type='text/css' href='style.css' />
<link
	rel='stylesheet'
	href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'
	integrity='sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY='
	crossorigin='' />
</head>
<body>
<noscript>Javascript is required for this web page.</noscript>
<script
	type='text/javascript'
	src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'
	integrity='sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo='
	crossorigin=''
></script>
<script type='text/javascript'>
	(function(p){document.readyState!=="loading"?p():document.addEventListener("DOMContentLoaded",p)})(function(p){
		window.Alone = {
			operator: location.pathname === "/operator",

)HTML";

	static PROGMEM char const home_html_2[] =
R"HTML(
		};
		return import("./script.js").then(function(){}, p);
	}
	(function(SD_load_error){
		"use strict";
		console.log("Failed to load script from SD card:", SD_load_error);
		var GPS_watch = false;
		function $T(string) {
			return document.createTextNode(string);
		}
		function $E(name) {
			return document.createElementNS(document.documentElement.namespaceURI, name);
		}
		function c_(parent, child) {
			return parent.appendChild(child);
		}
		function s_(element, name, value) {
			return element.style[name] = value;
		}
		function a_(element, name, value) {
			return element.setAttribute(name, value);
		}
		function string_from_Date(value, seperator = " ") {
			var date = new Date(value);
			return (
				date.getFullYear().toString()
					+ "-"
					+ (date.getMonth() + 1).toString().padStart(2, "0")
					+ "-"
					+ date.getDate().toString().padStart(2, "0")
					+ seperator
					+ date.getHours().toString().padStart(2, "0")
					+ ":"
					+ date.getMinutes().toString().padStart(2, "0")
					+ ":"
					+ date.getSeconds().toString().padStart(2, "0")
			);
		}
		var MILLISECONDS_FROM_1970_TO_2000 = 946684800000; /* = Date.UTC(2000, 0, 1, 0, 0, 0, 0) */
		document.body.textContent = "";
		void function () {
			var $p;
			$p = $E("p");
			c_($p, $T("Campaign: "));
			c_($p, $T(Alone.campaign));
			c_($p, $T(" | Organisation: "));
			c_($p, $T(Alone.organisation));
			c_($p, $T(" | Device: "));
			c_($p, $T(Alone.device));
			c_(document.body, $p);
		}
		void function () {
			var $p, $a;
			function style_$a() {
				s_($a, "margin", "1ex");
				s_($a, "border", "solid thin gray");
				s_($a, "padding", "1ex");
			}
			$p = $E("p");
			s_($p, "display", "flex");
			s_($p, "flex-flow", "row wrap");
			s_($p, "text-align", "center");
			if (Alone.operator) {
				$a = $E("a");
				style_$a();
				a_($a, "href", "setting.html");
				c_($a, $T("Settings"));
				c_($p, $a);
			}
			$a = $E("a");
			style_$a();
			a_($a, "href", "data/recent.csv");
			a_($a, "download", "data_recent.csv");
			c_($a, $T("Recent weather data"));
			c_($p, $a);
			$a = $E("a");
			style_$a();
			a_($a, "href", Alone.data_file);
			a_($a, "download", "");
			c_($a, $T("All weather data"));
			c_($p, $a);
			$a = $E("a");
			style_$a();
			a_($a, "href", "gps/recent.csv");
			a_($a, "download", "gps_recent.csv");
			c_($a, $T("Recent GPS data"));
			c_($p, $a);
			$a = $E("a");
			style_$a();
			a_($a, "href", Alone.gps_file);
			a_($a, "download", "");
			c_($a, $T("All GPS data"));
			c_($p, $a);
			c_(document.body, $p);
		}();
		var $refresh, $refresh_auto;
		void function () {
			var $form, $button, $label, $input;
			$refresh = $form = $E("form");
			s_($form, "display", "inline-block");
			s_($form, "margin", "1ex");
			s_($form, "border", "solid thin gray");
			s_($form, "padding", "1ex");
			$label = $E("label");
			s_($label, "margin-right", "1ex");
			s_($label, "padding", "1ex");
			$refresh_auto = $input = $E("input");
			a_($input, "type", "checkbox");
			c_($label, $input);
			c_($label, $T("Auto refresh"));
			c_($form, $label);
			$button = $E("button");
			a_($button, "type", "submit");
			s_($button, "margin-left", "1ex");
			c_($button, $T("Refresh now"));
			c_($form, $button);
			c_(document.body, $form);
		}();
		if (Alone.operator) {
			var $report_auto;
			var $upload;
			void function () {
				var $div, $input, $button;
				$div = $E("div");
				s_($div, "display", "inline-block");
				s_($div, "margin-top", "1ex");
				s_($div, "margin-bottom", "1ex");
				s_($div, "margin-left", "1ex");
				s_($div, "margin-right", "2ex");
				s_($div, "border", "solid thin gray");
				s_($div, "padding", "1ex");
				$report_auto = $input = $E("input");
				a_($input, "type", "checkbox");
				c_($div, $input);
				c_($div, $T("Report position"));
				c_(document.body, $div);
				$div = $E("div");
				s_($div, "display", "inline-block");
				s_($div, "margin-top", "1ex");
				s_($div, "margin-bottom", "1ex");
				s_($div, "margin-left", "1ex");
				s_($div, "margin-right", "2ex");
				s_($div, "border", "solid thin gray");
				s_($div, "padding", "1ex");
				$upload = $button = $E("button");
				a_($button, "type", "button");
				c_($button, $T("Upload data"));
				c_($div, $button);
				c_(document.body, $div);
			}();
		}
		var $data_list;
		var data_latest = null;
		void function () {
			var $table, $caption, $thead, $tr, $th, $tbody;
			$table = $E("table");
			s_($table, "margin-bottom", "3ex");
			s_($table, "border-collapse", "collapse");
			s_($table, "width", "100%");
			$caption = $E("caption");
			c_($caption, $T("Sensor data"));
			c_($table, $caption);
			$thead = $E("thead");
			s_($thead, "border-bottom-style", "solid");
			$tr = $E("tr");
			Alone.data_fields.forEach(
				function (field) {
					var text = field.name[0].toUpperCase() + field.name.substring(1);
					if (field.unit)
						text = text + " (" + field.unit + ")";
					$th = $E("th");
					c_($th, $T(text));
					c_($tr, $th);
				}
			);
			c_($thead, $tr);
			c_($table, $thead);
			$data_list = $tbody = $E("tbody");
			c_($table, $tbody);
			c_(document.body, $table);
		}();
		var $data_loading = $E("p");
		$data_loading.hidden = true;
		c_($data_loading, $T("Loading..."));
		c_(document.body, $data_loading);
		var $GPS_list;
		void function () {
			var $table, $caption, $thead, $tr, $th, $tbody;
			$table = $E("table");
			s_($table, "margin-bottom", "3ex");
			s_($table, "border-collapse", "collapse");
			s_($table, "width", "100%");
			$caption = $E("caption");
			c_($caption, $T("GPS data"));
			c_($table, $caption);
			$thead = $E("thead");
			s_($thead, "border-bottom-style", "solid");
			$tr = $E("tr");
			Alone.gps_fields.forEach(
				function (field) {
					var text = field.name[0].toUpperCase() + field.name.substring(1);
					if (field.unit)
						text = text + " (" + field.unit + ")";
					$th = $E("th");
					c_($th, $T(text));
					c_($tr, $th);
				}
			);
			c_($thead, $tr);
			c_($table, $thead);
			$GPS_list = $tbody = $E("tbody");
			c_($table, $tbody);
			c_(document.body, $table);
		}();
		var $GPS_loading = $E("p");
		$GPS_loading.hidden = true;
		c_($GPS_loading, $T("Loading..."));
		c_(document.body, $GPS_loading);
		function data_load() {
			return new Promise(
				function (resolve, reject) {
					$data_list.textContent = null;
					$data_loading.hidden = false;
					var xhr = new XMLHttpRequest();
					xhr.onloadend = function (event) {
						$data_loading.hidden = true;
						var text = xhr.responseText;
						if (text == null || xhr.status !== 200) {
							alert("Failed to load data");
							return reject(xhr);
						}
						var fields = null;
						var lines = text.split("\r\n");
						if (!lines || !(lines.length > 0)) return;
						for (var i = 1; lines.length > i; ++i) {
							var line = lines[lines.length - i].trim();
							if (!line || typeof line !== "string") continue;
							fields = line.split(",");
							var $tr = $E("tr");
							data_latest = new Object;
							for (var j = 0; fields.length > j; ++j) {
								var $td = $E("td");
								s_($td, "border-style", "solid");
								s_($td, "border-width", "thin");
								s_($td, "text-align", "center");
								var v = j === 0 ? string_from_Date(fields[j]) : fields[j];
								c_($td, $T(v));
								c_($tr, $td);
								if (Alone.data_meta > j)
									data_latest[Alone.data_fields[j].name] = v;
							}
							c_($data_list, $tr);
						}
						return resolve();
					};
					xhr.open("GET", "data/recent.csv", true);
					xhr.send(null);
				}
			);
		}
		function GPS_load() {
			return new Promise(
				function (resolve, reject) {
					$GPS_list.textContent = null;
					$GPS_loading.hidden = false;
					var xhr = new XMLHttpRequest();
					xhr.onloadend = function (event) {
						$GPS_loading.hidden = true;
						var text = xhr.responseText;
						if (text == null || xhr.status !== 200) {
							alert("Failed to load GPS records");
							return reject(xhr);
						}
						var lines = text.split("\r\n");
						if (!lines || !(lines.length > 0)) return;
						for (var i = 1; lines.length > i; ++i) {
							var line = lines[lines.length - i].trim();
							if (!line || typeof line !== "string") continue;
							var fields = line.split(",");
							var $tr = $E("tr");
							for (var j = 0; fields.length > j; ++j) {
								var $td = $E("td");
								s_($td, "border-style", "solid");
								s_($td, "border-width", "thin");
								s_($td, "text-align", "center");
								if (j === 0) c_($td, $T(string_from_Date(fields[j])));
								else c_($td, $T(fields[j]));
								c_($tr, $td);
							}
							c_($GPS_list, $tr);
						}
						return resolve();
					};
					xhr.open("GET", "gps/recent.csv", true);
					xhr.send(null);
				}
			);
		}
		function load_all() {
			return (
				data_load()
					.catch(function () {})
					.then(function () {return GPS_load();})
					.catch(function () {})
			);
		}
		$refresh.addEventListener(
			"submit",
			function (event) {
				event.preventDefault();
				return load_all();
			}
		);
		var refresh_timer = null;
		$refresh_auto.addEventListener(
			"change",
			function (event) {
				if ($refresh_auto.checked) {
					if (refresh_timer !== null) return;
					refresh_timer = setInterval(load_all, Alone.measure_interval);
				}
				else {
					if (refresh_timer === null) return;
					clearInterval(refresh_timer);
					refresh_timer = null;
				}
			}
		);
		setTimeout(load_all, 3000);
		if (Alone.operator)
			void function () {
				if ("geolocation" in window.navigator) if (window.isSecureContext) {
					function GPS_upload(planned_time, browser_time, position_time, coords) {
						var body = new URLSearchParams;
						body.append("campaign",      Alone.campaign);
						body.append("organisation",  Alone.organisation);
						body.append("device",        Alone.device);
						body.append("time",          planned_time);
						body.append("browser_time",  browser_time);
						body.append("position_time", position_time);
						body.append("latitude",      coords.latitude);
						body.append("longitude",     coords.longitude);
						body.append("altitude",      coords.altitude);
						var xhr = new XMLHttpRequest();
						xhr.open("POST", "/gps/upload.exe", true);
						xhr.send(body);
					}
					function GPS_report(timestamp, coords) {
						var body = new URLSearchParams;
						body.append("campaign",     Alone.campaign);
						body.append("organisation", Alone.organisation);
						body.append("device",       Alone.device);
						body.append("time",         timestamp);
						body.append("latitude",     coords.latitude);
						body.append("longitude",    coords.longitude);
						body.append("altitude",     coords.altitude);
						for (var field in data_latest)
							body.append(field, data_latest[field]);
						var xhr = new XMLHttpRequest();
						xhr.open("POST", Alone.report_URL, true);
						xhr.send(body);
					}
					function GPS_record(planned_time, spacetime) {
						if (spacetime === null || typeof spacetime === "undefined") return;
						var browser_time = string_from_Date(Date.now(), "T");
						var position_time = string_from_Date(spacetime.timestamp, "T");
						var coords = spacetime.coords;
						if (Alone.operator) {
							GPS_upload(planned_time, browser_time, position_time, coords);
							if ($report_auto.checked)
								GPS_report(position_time, coords);
						}
					}
					function GPS_error(error) {
						console.error("GeoLocationError: ", error.message);
					}
					var GPS_options = {
						timeout: Alone.measure_interval / 4,
						enableHighAccuracy: true
					};
					function GPS_request() {
						var now_plus_half =
							Date.now()
								- MILLISECONDS_FROM_1970_TO_2000
								+ Alone.measure_interval / 2;
						var planned_time =
							string_from_Date(
								now_plus_half
									- now_plus_half % Alone.measure_interval
									+ MILLISECONDS_FROM_1970_TO_2000,
								"T"
							);
						navigator.geolocation.getCurrentPosition(GPS_record.bind(this, planned_time), GPS_error, GPS_options)
					}
					function GPS_start() {
						setInterval(GPS_request, Alone.measure_interval);
					}
					setTimeout(GPS_start, (Date.now() - MILLISECONDS_FROM_1970_TO_2000) % Alone.measure_interval);
					setTimeout(GPS_request, 0);
					function GPS_callback(spacetime) {
						GPS_record(string_from_Date(spacetime.timestamp), spacetime);
					}
					if (GPS_watch) navigator.geolocation.watchPosition(GPS_callback, GPS_error, GPS_options);
				}
				void function () {
					$upload.addEventListener(
						"click",
						function (event) {
							event.preventDefault();
							var identity =
								Alone.campaign + "\r\n" +
								Alone.organisation + "\r\n" +
								Alone.device + "\r\n" +
								Alone.password + "\r\n";
							new Promise(
								function (resolve, reject) {
									var xhr = new XMLHttpRequest();
									xhr.onloadend = function () {
										var text = xhr.responseText;
										if (xhr.status !== 200 || text == null)
											return reject();
										return resolve(text);
									};
									xhr.open("GET", Alone.data_file, true);
									xhr.send(null);
								}
							).then(
								function (text) {
									return new Promise(
										function (resolve, reject) {
											var xhr = new XMLHttpRequest();
											xhr.onloadend = function () {
												if (xhr.status !== 200)
													return reject();
												return resolve();
											};
											xhr.open("POST", Alone.upload_data_URL, true);
											xhr.send(identity + text);
										}
									);
								}
							).then(
								function (text) {
									return new Promise(
										function (resolve, reject) {
											var xhr = new XMLHttpRequest();
											xhr.onloadend = function () {
												var text = xhr.responseText;
												if (xhr.status !== 200 || text == null)
													return reject();
												return resolve(text);
											};
											xhr.open("GET", Alone.gps_file, true);
											xhr.send(null);
										}
									);
								}
							).then(
								function (text) {
									return new Promise(
										function (resolve, reject) {
											var xhr = new XMLHttpRequest();
											xhr.onloadend = function () {
												if (xhr.status !== 200)
													return reject();
												return resolve();
											};
											xhr.open("POST", Alone.upload_position_URL, true);
											xhr.send(identity + text);
										}
									);
								}
							).catch(
								function (e) {
									console.error(e);
									alert("Failed to upload data");
								}
							);
						}
					);
				}();
				void function () {
					/* set device time */
					var xhr = new XMLHttpRequest();
					var body = new URLSearchParams();
					body.append("time", string_from_Date(new Date(), "T"));
					xhr.open("POST", "setting.exe", true);
					xhr.send(body);
				}();
			}();
	}));
</script>
</body>
</html>
)HTML";

	static esp_err_t home_handle(PsychicRequest *const request) {
		PsychicStreamResponse response(request, XHTML_content_type);
		response.addHeader("CONTENT-SECURITY-POLICY", "connect-src *");
		response.beginSend();
		response.write(reinterpret_cast<uint8_t const *>(home_html_1), sizeof home_html_1 - 1);
		response.print("\t\tcampaign: \"");
		response.print(javascript_escape(campaign_name));
		response.print("\",\r\n\t\t\t\torganisation: \"");
		response.print(javascript_escape(organisation_name));
		response.print("\",\r\n\t\t\tdevice: \"");
		response.print(javascript_escape(device_name));
		response.print("\",\r\n\t\t\tmeasure_interval: \"");
		response.print(measure_interval);
		response.print("\",\r\n\t\t\tdata_file: \"");
		response.print(javascript_escape(data_filename));
		response.print("\",\r\n\t\t\tgps_file: \"");
		response.print(javascript_escape(gps_filename));
		response.print("\",\r\n\t\t\tmonitor_URL: \"");
		response.print(javascript_escape(monitor_URL));
		response.print("\",\r\n\t\t\treport_URL: \"");
		response.print(javascript_escape(monitor_URL + report_path));
		response.print("\",\r\n\t\t\tupload_data_URL: \"");
		response.print(javascript_escape(monitor_URL + upload_data_path));
		response.print("\",\r\n\t\t\tupload_position_URL: \"");
		response.print(javascript_escape(monitor_URL + upload_position_path));
		response.print("\",\r\n\t\t\tpassword: \"");
		response.print(javascript_escape(upload_password));
		response.print("\",\r\n\t\t\tdata_fields: [");
		bool first = true;
		for (Field const field: data_fields) {
			if (first)
				first = false;
			else
				response.print(", ");
			response.print("{name:\"");
			response.print(javascript_escape(field.name));
			response.print("\",unit:");
			if (field.unit == nullptr)
				response.print("null");
			else {
				response.print('"');
				response.print(javascript_escape(field.unit));
				response.print('"');
			}
			response.print("}");
		}
		response.print("],\r\n\t\t\tdata_meta: ");
		response.print(data_meta);
		response.print(",\r\n\t\t\tgps_fields: [");
		first = true;
		for (Field const field: gps_fields) {
			if (first)
				first = false;
			else
				response.print(", ");
			response.print("{name:\"");
			response.print(javascript_escape(field.name));
			response.print("\",unit:");
			if (field.unit == nullptr)
				response.print("null");
			else {
				response.print('"');
				response.print(javascript_escape(field.unit));
				response.print('"');
			}
			response.print("}");
		}
		response.print("],\r\n\t\t\tgps_meta: ");
		response.print(gps_meta);
		response.print("\r\n");
		response.write(reinterpret_cast<uint8_t const *>(home_html_2), sizeof home_html_2 - 1);
		return response.endSend();
	}

	static PROGMEM char const icon_data[] = {
		/* PNG signature */
		0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A,
		/* data length */
		0x00, 0x00, 0x00, 0x0D,
		/* "IHDR" as ASCII */
		0x49, 0x48, 0x44, 0x52,
		/* width */
		0x00, 0x00, 0x00, 0x01,
		/* height */
		0x00, 0x00, 0x00, 0x01,
		/* bit depth */
		0x01,
		/* colour type */
		0x00,
		/* compression method */
		0x00,
		/* filter method */
		0x00,
		/* interlace method */
		0x00,
		/* header checksum */
		0x37, 0x6E, 0xF9, 0x24,
		/* data length */
		0x00, 0x00, 0x00, 0x0A,
		/* chunk type "IDAT" */
		0x49, 0x44, 0x41, 0x54,
		/* zlib header */
		0x78, 0x01,
		/* compressed DEFLATE block */
		0x63, 0x60, 0x00, 0x00,
		/* zlib checksum */
		0x00, 0x02, 0x00, 0x01,
		/* chunk checksum */
		0x73, 0x75, 0x01, 0x18
	};

	static esp_err_t icon_handle(PsychicRequest *const request) {
		return request->reply(200, "image/png", icon_data);
	}

	static esp_err_t data_recent_handle(PsychicRequest *const request) {
		PsychicStreamResponse response(request, "text/csv");
		response.addHeader("CONTENT-SECURITY-POLICY", "connect-src *");
		response.beginSend();
		response.println(data_header);
		for (Data const &record: data_records)
			response.println(CSV_Data(&record));
		return response.endSend();
	}

	static esp_err_t data_latest_handle(PsychicRequest *const request) {
		PsychicStreamResponse response(request, "text/csv");
		response.addHeader("CONTENT-SECURITY-POLICY", "connect-src *");
		response.beginSend();
		response.println(data_header);
		if (!data_records.empty())
			response.println(CSV_Data(&data_records.back()));
		return response.endSend();
	}

	static esp_err_t gps_recent_handle(PsychicRequest *const request) {
		PsychicStreamResponse response(request, "text/csv");
		response.addHeader("CONTENT-SECURITY-POLICY", "connect-src *");
		response.beginSend();
		response.println(gps_header);
		for (GPS const &record: gps_records)
			response.println(CSV_GPS(&record));
		return response.endSend();
	}

	static esp_err_t gps_latest_handle(PsychicRequest *const request) {
		PsychicStreamResponse response(request, "text/csv");
		response.addHeader("CONTENT-SECURITY-POLICY", "connect-src *");
		response.beginSend();
		response.println(gps_header);
		if (!data_records.empty())
			response.println(CSV_Data(&data_records.back()));
		return response.endSend();
	}

	static esp_err_t gps_upload_handle(PsychicRequest *const request) {
		GPS gps = {.time = (uint32_t)0, .latitude = NAN, .longitude = NAN, .altitude = NAN};
		PsychicWebParameter *parameter;
		parameter = request->getParam("time");
		if (parameter != nullptr) {
			String const &value = parameter->value();
			DateTime const datetime(value.c_str());
			if (datetime.isValid())
				gps.time = datetime;
			else {
				Serial.print("WARN: incorrect GPS time = ");
				Serial.println(value);
			}
		}
		parameter = request->getParam("browser_time");
		if (parameter != nullptr) {
			String const &value = parameter->value();
			DateTime const datetime(value.c_str());
			if (datetime.isValid())
				gps.browser_time = datetime;
			else {
				Serial.print("WARN: incorrect GPS browser_time = ");
				Serial.println(value);
			}
		}
		parameter = request->getParam("position_time");
		if (parameter != nullptr) {
			String const &value = parameter->value();
			DateTime const datetime(value.c_str());
			if (datetime.isValid())
				gps.position_time = datetime;
			else {
				Serial.print("WARN: incorrect GPS position_time = ");
				Serial.println(value);
			}
		}
		parameter = request->getParam("latitude");
		if (parameter != nullptr) {
			String const &value = parameter->value();
			if (value != "null") {
				char *end;
				double const x = strtod(value.c_str(), &end);
				if (!*end)
					gps.latitude = x;
				else {
					Serial.print("WARN: incorrect GPS latitude = ");
					Serial.println(value);
				}
			}
		}
		parameter = request->getParam("longitude");
		if (parameter != nullptr) {
			String const &value = parameter->value();
			if (value != "null") {
				char *end;
				double const x = strtod(value.c_str(), &end);
				if (!*end)
					gps.longitude = x;
				else {
					Serial.print("WARN: incorrect GPS longitude = ");
					Serial.println(value);
				}
			}
		}
		parameter = request->getParam("altitude");
		if (parameter != nullptr) {
			String const &value = parameter->value();
			if (value != "null") {
				char *end;
				double const x = strtod(value.c_str(), &end);
				if (!*end)
					gps.altitude = x;
				else {
					Serial.print("WARN: incorrect GPS altitude = ");
					Serial.println(value);
				}
			}
		}
		String const GPS_string = CSV_GPS(&gps);
		Serial.print("INFO: GPS ");
		Serial.println(GPS_string);

		if (gps_records.size() >= records_max_size)
			gps_records.pop_front();
		gps_records.push_back(gps);

		if (SD_card_exist) {
			SDCARD_LOCK(sdcard_lock);
			File file = SD.open(gps_filename, "a", true);
			try {
				if (!file.position())
					file.println(gps_header);
				file.println(GPS_string);
			}
			catch (...) {
				Serial.println("ERROR: failed to write GPS data into SD card");
			}
			file.close();
		}

		return request->reply(204, "text/plain", "");
	}

	static PROGMEM char const setting_html_1[] =
R"HTML(<html xmlns='http://www.w3.org/1999/xhtml'>
<head>
<meta content-type='application/xhtml+xml; charset=UTF-8' />
<meta charset='UTF-8' />
<meta name='viewport' content='width=device-width, initial-scale=1' />
<title>Settings</title>
<link rel='icon' type='image/png' href='favicon.ico' />
<link rel='stylesheet' type='text/css' href='style.css' />
</head>
<body>
<p><a href='operator'>&#x2190; Back</a></p>
)HTML";

	static PROGMEM char const setting_html_2[] =
R"HTML(
</body>
</html>
)HTML";

	static PROGMEM char const setting_form_1[] = "<form\r\n\tid='";

	static PROGMEM char const setting_form_2[] =
R"HTML('
	class='setting-form'
	action='setting.exe'
	method='POST'
	style='margin: 1ex; border: solid thin; padding: 1ex'
>
)HTML";

	static String XML_escape(String const &string) {
		String result;
		for (char const c: string)
			switch (c) {
			case '&':
				result.concat("&amp;");
				break;
			case '<':
				result.concat("&lt;");
				break;
			case '>':
				result.concat("&gt;");
				break;
			case '"':
				result.concat("&quot;");
				break;
			case '\'':
				result.concat("&apos;");
				break;
			default:
				result.concat(c);
			}
		return result;
	}

	static void setting_form(PsychicStreamResponse *const response, char const *const id) {
		response->write(reinterpret_cast<uint8_t const *>(setting_form_1), sizeof setting_form_1 - 1);
		response->print(XML_escape(id));
		response->write(reinterpret_cast<uint8_t const *>(setting_form_2), sizeof setting_form_2 - 1);
	}

	static esp_err_t setting_handle(PsychicRequest *const request) {
		PsychicStreamResponse response(request, XHTML_content_type);
		response.addHeader("CONTENT-SECURITY-POLICY", "connect-src *");
		response.beginSend();

		response.write(reinterpret_cast<uint8_t const *>(setting_html_1), sizeof setting_html_1 - 1);

		setting_form(&response, "set_time");
		response.print(
			"\t<label>\r\n"
			"\t\tCurrent time \r\n"
			"\t\t<input type='datetime-local' name='time' required='' />\r\n"
			"\t</label>\r\n"
			"\t<button type='submit'>Set</button>\r\n"
			"</form>\r\n"
		);

		setting_form(&response, "set_campaign");
		response.print(
			"\t<label>\r\n"
			"\t\tCampaign\r\n"
			"\t\t<input type='text' name='campaign' required='' value='"
		);
		response.print(XML_escape(campaign_name));
		response.print(
			"' />\r\n"
			"\t</label>\r\n"
			"\t<button type='submit'>Set</button>\r\n"
			"</form>\r\n"
		);

		setting_form(&response, "set_organisation");
		response.print(
			"\t<label>\r\n"
			"\t\tOrganisation\r\n"
			"\t\t<input type='text' name='organisation' required='' value='"
		);
		response.print(XML_escape(organisation_name));
		response.print(
			"' />\r\n"
			"\t</label>\r\n"
			"\t<button type='submit'>Set</button>\r\n"
			"</form>\r\n"
		);

		setting_form(&response, "set_device");
		response.print(
			"\t<label>\r\n"
			"\t\tDevice ID\r\n"
			"\t\t<input type='text' name='device' required='' value='"
		);
		response.print(XML_escape(device_name));
		response.print(
			"' />\r\n"
			"\t</label>\r\n"
			"\t<button type='submit'>Set</button>\r\n"
			"</form>\r\n"
		);

		setting_form(&response, "set_interval");
		response.print(
			"\t<label>\r\n"
			"\t\tMeasure interval / seconds\r\n"
			"\t\t<input type='number' name='interval' min='10' max='900' required='' value='"
		);
		response.print(String(measure_interval / 1000));
		response.print(
			"' />"
			"\t</label>\r\n"
			"\t<button type='submit'>Set</button>\r\n"
			"</form>\r\n"
		);

		setting_form(&response, "set_wifi");
		response.print(
			"\t<label style='display: block'>\r\n"
			"\t\tProvide WiFi\r\n"
			"\t\t<select name='WiFi'>\r\n"
			"\t\t\t<option value='AP'"
		);
		if (use_AP_mode) response.print(" selected=''");
		response.print(
			">\r\n"
			"\t\t\t\tAccess point\r\n"
			"\t\t\t</option>\r\n"
			"\t\t\t<option value='STA'"
		);
		if (!use_AP_mode) response.print(" selected=''");
		response.print(
			">\r\n"
			"\t\t\t\tStation\r\n"
			"\t\t\t</option>\r\n"
			"\t\t</select>\r\n"
			"\t</label>\r\n"
			"\t<label style='display: block'>\r\n"
			"\t\tAP SSID\r\n"
			"\t\t<input name='APSSID' value='"
		);
		response.print(XML_escape(AP_SSID));
		response.print(
			"' />\r\n"
			"\t</label>\r\n"
			"\t<label style='display: block'>\r\n"
			"\t\tAP PASS\r\n"
			"\t\t<input name='APPASS' value='"
		);
		response.print(XML_escape(AP_PASS));
		response.print(
			"' />\r\n"
			"\t</label>\r\n"
			"\t<label style='display: block'>\r\n"
			"\t\tSTA SSID\r\n"
			"\t\t<input name='STASSID' value='"
		);
		response.print(XML_escape(STA_SSID));
		response.print(
			"' />\r\n"
			"\t</label>\r\n"
			"\t<label style='display: block'>\r\n"
			"\t\tSTA PASS\r\n"
			"\t\t<input name='STAPASS' value='"
		);
		response.print(XML_escape(STA_PASS));
		response.print(
			"' />\r\n"
			"\t</label>\r\n"
			"\t<button type='submit'>Set</button>\r\n\r\n"
			"</form>\r\n"
		);

		setting_form(&response, "set_monitor");
		response.print(
			"\t<label>\r\n"
			"\t\tMonitor server URL\r\n"
			"\t\t<input type='text' name='monitor' required='' value='"
		);
		response.print(XML_escape(monitor_URL));
		response.print(
			"' />\r\n"
			"\t</label>\r\n"
			"\t<button type='submit'>Set</button>\r\n"
			"</form>\r\n"
		);

		setting_form(&response, "set_password");
		response.print(
			"\t<label>\r\n"
			"\t\tUpload password\r\n"
			"\t\t<input type='text' name='password' required='' value='"
		);
		response.print(XML_escape(upload_password));
		response.print(
			"' />\r\n"
			"\t</label>\r\n"
			"\t<button type='submit'>Set</button>\r\n"
			"</form>\r\n"
		);

		setting_form(&response, "do_measure");
		response.print(
			"\t<label style='display: block'>\r\n"
			"\t\tConfirm\r\n"
			"\t\t<input type='checkbox' name='measure' />\r\n"
			"\t</label>\r\n"
			"\t<button type='submit'>Measure now</button>\r\n"
			"</form>\r\n"
		);

		setting_form(&response, "do_delete");
		response.print(
			"\t<label style='display: block'>\r\n"
			"\t\tConfirm\r\n"
			"\t\t<input type='checkbox' name='delete' />\r\n"
			"\t</label>\r\n"
			"\t<button type='submit'>Delete all data</button>\r\n"
			"</form>\r\n"
		);

		setting_form(&response, "do_reboot");
		response.print(
			"\t<label style='display: block'>Confirm \r\n"
			"\t\t<input type='checkbox' name='reboot' />\r\n"
			"\t</label>\r\n"
			"\t<button type='submit' name='reboot'>Reboot</button>\r\n"
			"</form>\r\n"
		);

		response.write(reinterpret_cast<uint8_t const *>(setting_html_2), sizeof setting_html_2 - 1);
		return response.endSend();
	}

	static PROGMEM char const command_html[] =
R"HTML(<html xmlns='http://www.w3.org/1999/xhtml'>
<head>
<meta content-type='application/xhtml+xml; charset=UTF-8' />
<meta charset='UTF-8' />
<meta name='viewport' content='width=device-width, initial-scale=1' />
<title>Command redirection</title>
<link rel='stylesheet' type='text/css' href='style.css' />
</head>
<body>
<p>Command received. Redirect to <a href='./setting.html'>homepage.</a></p>
</body>
</html>
)HTML";

	static esp_err_t command_handle(PsychicRequest *const request) {
		PsychicWebParameter *parameter;
		parameter = request->getParam("time");
		if (parameter != nullptr) {
			Serial.print("INFO: command time = ");
			Serial.println(parameter->value());
			DateTime const datetime(parameter->value().c_str());
			if (datetime.isValid())
				set_time(&datetime);
			else {
				Serial.print("WARN: incorrect command time = ");
				Serial.println(parameter->value());
			}
		}
		parameter = request->getParam("campaign");
		if (parameter != nullptr) {
			Serial.print("INFO: command campaign = ");
			Serial.println(parameter->value());
			campaign_name = parameter->value();
			need_save = true;
		}
		parameter = request->getParam("organisation");
		if (parameter != nullptr) {
			Serial.print("INFO: command organisation = ");
			Serial.println(parameter->value());
			organisation_name = parameter->value();
			need_save = true;
		}
		parameter = request->getParam("device");
		if (parameter != nullptr) {
			Serial.print("INFO: command device = ");
			Serial.println(parameter->value());
			device_name = parameter->value();
			need_save = true;
		}
		parameter = request->getParam("interval");
		if (parameter != nullptr) {
			char const *const value = parameter->value().c_str();
			Serial.print("INFO: command interval = ");
			Serial.println(value);
			char *end;
			unsigned long int const x = strtoul(value, &end, 10);
			if (!*end && x >= 15 && x <= 900) {
				measure_interval = x * 1000;
				need_save = true;
			}
			else {
				Serial.print("WARN: incorrect command interval = \"");
				Serial.print(value);
				Serial.println('"');
			}
		}
		parameter = request->getParam("WiFi");
		if (parameter != nullptr) {
			String const &value = parameter->value();
			Serial.print("INFO: command WiFi = ");
			Serial.println(value);
			if (value == "AP") {
				use_AP_mode = true;
				need_save = true;
			}
			else if (value == "STA") {
				use_AP_mode = false;
				need_save = true;
			}
			else {
				Serial.print("WARN: incorrect command WiFi = \"");
				Serial.print(value);
				Serial.println('"');
			}
		}
		parameter = request->getParam("APSSID");
		if (parameter != nullptr) {
			Serial.print("INFO: command APSSID = ");
			Serial.println(parameter->value());
			AP_SSID = parameter->value();
			need_save = true;
		}
		parameter = request->getParam("APPASS");
		if (parameter != nullptr) {
			Serial.print("INFO: command APPASS = ");
			Serial.println(parameter->value());
			AP_PASS = parameter->value();
			need_save = true;
		}
		parameter = request->getParam("STASSID");
		if (parameter != nullptr) {
			Serial.print("INFO: command STASSID = ");
			Serial.println(parameter->value());
			STA_SSID = parameter->value();
			need_save = true;
		}
		parameter = request->getParam("STAPASS");
		if (parameter != nullptr) {
			Serial.print("INFO: command STAPASS = ");
			Serial.println(parameter->value());
			STA_PASS = parameter->value();
			need_save = true;
		}
		parameter = request->getParam("monitor");
		if (parameter != nullptr) {
			Serial.print("INFO: command monitor = ");
			Serial.println(parameter->value());
			monitor_URL = parameter->value();
			need_save = true;
		}
		parameter = request->getParam("password");
		if (parameter != nullptr) {
			Serial.print("INFO: command password = ");
			Serial.println(parameter->value());
			upload_password = parameter->value();
			need_save = true;
		}
		parameter = request->getParam("measure");
		if (parameter != nullptr) {
			Serial.print("INFO: command measure = ");
			Serial.println(parameter->value());
			if (parameter->value().length()) {
				use_assigned_device_time = true;
				assigned_device_time = DateTime(parameter->value().c_str());
			}
			wait_measure_condition.notify_all();
		}
		if (request->hasParam("delete")) {
			Serial.println("INFO: command delete");
			data_records.clear();
			gps_records.clear();
			SDCARD_LOCK(sdcard_lock);
			File data_file = SD.open(data_filename, "w", true);
			try {
				data_file.println(data_header);
			}
			catch (...) {
				Serial.println("ERROR: failed to write header into data file");
			}
			data_file.close();
			File gps_file = SD.open(gps_filename, "w", true);
			try {
				gps_file.println(gps_header);
			}
			catch (...) {
				Serial.println("ERROR: failed to write header into GPS file");
			}
			gps_file.close();
		}
		if (request->hasParam("reboot")) {
			Serial.println("INFO: command reboot");
			Serial.flush();
			need_reboot = true;
			need_save = false;
		}

		PsychicResponse response(request);
		response.setCode(303);
		response.setContentType("application/xhtml+xml; charset=UTF-8");
		response.addHeader("LOCATION", "/setting.html");
		response.setContent(reinterpret_cast<uint8_t const *>(command_html), sizeof command_html - 1);
		return response.send();
	}

	static void setup(void) {
		HTTPd.config.max_uri_handlers = 20;
		while (HTTPd.listen(HTTP_port) != ESP_OK) {
			Serial.println("ERROR: failed to start HTTP server");
			Monitor.println("Failed to start HTTPS server");
			Monitor.display();
			delay(reinitialize_interval);
		}
		Serial.println("HTTP server started");

		HTTPSd.config.max_uri_handlers = 20;
		while (HTTPSd.listen(HTTPS_port, tls_cert, tls_key) != ESP_OK) {
			Serial.println("ERROR: failed to start HTTPS server");
			Monitor.println("Failed to start HTTPS server");
			Monitor.display();
			delay(reinitialize_interval);
		}
		Serial.println("HTTPS server started");

		HTTPd .on("/",                HTTP_GET, home_handle);
		HTTPSd.on("/",                HTTP_GET, home_handle);
		HTTPd .on("/operator",        HTTP_GET, home_handle);
		HTTPSd.on("/operator",        HTTP_GET, home_handle);
		HTTPd .on("/favicon.ico",     HTTP_GET, icon_handle);
		HTTPSd.on("/favicon.ico",     HTTP_GET, icon_handle);
		HTTPd .on("/data/recent.csv", HTTP_GET, data_recent_handle);
		HTTPSd.on("/data/recent.csv", HTTP_GET, data_recent_handle);
		HTTPd .on("/data/latest.csv", HTTP_GET, data_latest_handle);
		HTTPSd.on("/data/latest.csv", HTTP_GET, data_latest_handle);
		HTTPd .on("/gps/recent.csv",  HTTP_GET, gps_recent_handle);
		HTTPSd.on("/gps/recent.csv",  HTTP_GET, gps_recent_handle);
		HTTPd .on("/gps/latest.csv",  HTTP_GET, gps_latest_handle);
		HTTPSd.on("/gps/latest.csv",  HTTP_GET, gps_latest_handle);
		HTTPd .on("/gps/upload.exe", HTTP_POST, gps_upload_handle);
		HTTPSd.on("/gps/upload.exe", HTTP_POST, gps_upload_handle);
		HTTPd .on("/setting.html",    HTTP_GET, setting_handle);
		HTTPSd.on("/setting.html",    HTTP_GET, setting_handle);
		HTTPd .on("/setting.exe",    HTTP_POST, command_handle);
		HTTPSd.on("/setting.exe",    HTTP_POST, command_handle);
		if (SD_card_exist) {
			HTTPd .serveStatic("/", SD, "/", "max-age=604800");
			HTTPSd.serveStatic("/", SD, "/", "max-age=604800");
		}
		else {
			HTTPd .on(data_filename, HTTP_GET, data_recent_handle);
			HTTPSd.on(data_filename, HTTP_GET, data_recent_handle);
			HTTPd .on(gps_filename,  HTTP_GET, gps_recent_handle);
			HTTPSd.on(gps_filename,  HTTP_GET, gps_recent_handle);
		}
	}
}

/* *************************************************************************** / ************************************ */
/* Main procedures */

static void redraw_display(bool const start_over) {
	static unsigned short int section = 0;
	if (start_over) section = 0;
	DISPLAY_LOCK(display_lock);
	Monitor.clearDisplay();
	if (section == 0 && data_records.size()) {
		Data const *const data = &data_records.back();
		char year[6], date[7], time[6];
		String fulltime = show_time(&data->time);
		if (fulltime.length() == 19) {
			memcpy(year, fulltime.c_str(), 5);
			year[5] = 0;
			memcpy(date, fulltime.c_str() + 5, 5);
			date[5] = 0;
			memcpy(time, fulltime.c_str() + 11, 5);
			time[5] = 0;
		}
		else {
			year[0] = date[0] = time[0] = '?';
			year[1] = date[1] = time[1] = 0;
		}
		Monitor.setRotation(3);
		Monitor.setFont(&FONT_0);
		Monitor.setCursor(0, FONT_0_OFFSET);
		Monitor.println(year);
		Monitor.println(date);
		Monitor.println(time);
		Monitor.println();
		Monitor.print(data->temperature, 1);
		Monitor.println("C");
		Monitor.print(data->humidity, 1);
		Monitor.println("%");
		Monitor.drawLine(0, 76, 63, 76, SSD1306_WHITE);
		++section;
	}
	else {
		Monitor.setRotation(0);
		Monitor.setFont(&FONT_1);
		Monitor.setCursor(0, FONT_1_OFFSET);
		if (SD_card_exist)
			Monitor.println("SD card found");
		else
			Monitor.println("No SD card");
		if (use_AP_mode) {
			WiFi.softAPIP().printTo(Monitor);
			Monitor.println();
			Monitor.print("AP:");
			Monitor.println(WiFi.softAPSSID());
		}
		else {
			signed int status = WiFi.status();
			Monitor.println(WIFI::status_message(status));
			if (WiFi.status() == WL_CONNECTED) {
				WiFi.localIP().printTo(Monitor);
				Monitor.println();
				Monitor.print("STA:");
				Monitor.println(WiFi.SSID());
			}
		}
		section = 0;
	}
	Monitor.display();
}

void loop(void) {
	static unsigned short int count = 0;

	delay(1000);

	if (need_save) {
		save_settings();
		need_save = false;
	}

	if (need_reboot) {
		static unsigned long int reboot_time_0 = 0;
		static unsigned long int reboot_time_1 = 0;
		unsigned long int now = millis();
		if (!reboot_time_1) {
			reboot_time_0 = now;
			reboot_time_1 = now + reboot_wait_time;
		}
		else if (
			reboot_time_0 < reboot_time_1 && (now < reboot_time_0 || reboot_time_1 < now) ||
			reboot_time_1 < reboot_time_0 && reboot_time_1 < now && now < reboot_time_0
		) {
			need_reboot = false;
			esp_restart();
		}
	}

	if (++count >= display_refresh_interval) {
		count = 0;
		redraw_display();
	}
}

static void set_pthread_stack_size(size_t const stack_size) {
	static esp_pthread_cfg_t esp_pthread_cfg = esp_pthread_get_default_config();
	esp_pthread_cfg.stack_size = stack_size;
	esp_pthread_set_cfg(&esp_pthread_cfg);
}

void setup(void) {
	/* Constants*/
	data_header = data_fields[0].name;
	if (data_fields[0].unit)
		data_header = data_header + " (" + data_fields[0].unit + ')';
	for (unsigned int i = 1; i < sizeof data_fields / sizeof *data_fields; ++i) {
		data_header = data_header + ',' + data_fields[i].name;
		if (data_fields[i].unit)
			data_header = data_header + " (" + data_fields[i].unit + ')';
	}
	gps_header = gps_fields[0].name;
	if (gps_fields[0].unit)
		gps_header = gps_header + " (" + gps_fields[0].unit + ')';
	for (unsigned int i = 1; i < sizeof gps_fields / sizeof *gps_fields; ++i) {
		gps_header = gps_header + ',' + gps_fields[i].name;
		if (gps_fields[i].unit)
			gps_header = gps_header + " (" + gps_fields[i].unit + ')';
	}

	/* Reset pin */
	pinMode(reset_pin, INPUT);

	/* Serial port */
	Serial.begin(serial_baudrate);

	/* OLED display */
	Monitor.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	Monitor.setRotation(3);
	Monitor.invertDisplay(false);
	Monitor.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
	Monitor.setCursor(0, 0);
	Monitor.clearDisplay();
	Monitor.display();

	/* Start-up delay */
	delay(start_wait_time);

	/* SD */
	pinMode(SD_MISO, INPUT_PULLUP);
	SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
	SD_card_exist = SD.begin(SD_CS, SPI);
	if (SD_card_exist) {
		Serial.println("SD card found");
		Monitor.println("OK SD card");
		load_settings();
	}
	else {
		Serial.println("SD card not found");
		Monitor.println("No SD card");
	}

	/* Clock */
	external_clock_available = external_clock.begin();
	if (external_clock_available) {
		Serial.println("Clock found");
		Monitor.println("OK clock");
	}
	else {
		Serial.println("Clock not found");
		Monitor.println("No clock");
	}

	/* Sensor */
	while (!SHT4x.begin()) {
		Serial.println("ERROR: SHT40 not found");
		Monitor.println("No SHT40");
		Monitor.display();
		delay(reinitialize_interval);
	}
	SHT4x.setPrecision(SHT4X_HIGH_PRECISION);
	SHT4x.setHeater(SHT4X_NO_HEATER);
	Serial.println("SHT40 found");
	Monitor.println("OK SHT40");
	Monitor.display();

	/* WiFi */
	WIFI::setup();

	/* Web server */
	set_pthread_stack_size(16384);
	WEB::setup();

	/* Spawn measurement thread */
	set_pthread_stack_size(4096);
	std::thread(measure_thread).detach();

	/* Main loop delay */
	delay(start_wait_time);
}

/* *************************************************************************** / ************************************ */
