

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_camera.h"


const char* ssid     = "";
const char* password = "";


#define BOT_TOKEN ""
#define CHAT_ID   ""

WiFiClientSecure secured_client;   
UniversalTelegramBot bot(BOT_TOKEN, secured_client);   


#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define SDA_PIN 41
#define SCL_PIN 42

#define TRIG_LEFT  2
#define ECHO_LEFT  1

#define TRIG_RIGHT 47
#define ECHO_RIGHT 48

// jarak (cm) di bawah ini dianggap BAHAYA
#define JARAK_BAHAYA_CM 60.0

//==============================================================
// BUZZER & BUTTON
//==============================================================
#define BUZZER_PIN 21
#define BUTTON_PIN 45

//==============================================================
// GPS
//==============================================================
#define GPS_RX 19 
#define GPS_TX 20

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;


#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13


float jarakKiri  = 0;
float jarakKanan = 0;

bool bahaya  = false;
bool gpsLock = false;
bool cameraOK = false;

unsigned long lastOLED   = 0;
unsigned long lastCameraRetry = 0;


bool gpsWasLocked = false;          
unsigned long lastSerialMonitor = 0; 


TaskHandle_t gpsTaskHandle = NULL;
portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED; 


TaskHandle_t sensorTaskHandle = NULL;
portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED; 


bool sendPhotoTelegram();
bool initCamera();
void printSerialMonitor();   
void gpsTask(void *param);
void sensorTask(void *param); 


float bacaJarak(int trig, int echo)
{
    digitalWrite(trig, LOW);
    delayMicroseconds(2);

    digitalWrite(trig, HIGH);
    delayMicroseconds(10);

    digitalWrite(trig, LOW);

    long durasi = pulseIn(echo, HIGH, 30000);

    if (durasi == 0)
        return -1;

    return durasi * 0.0343 / 2;
}


void sensorTask(void *param)
{
    for (;;)
    {
        float kiri  = bacaJarak(TRIG_LEFT, ECHO_LEFT);
        float kanan = bacaJarak(TRIG_RIGHT, ECHO_RIGHT);

        bool kiriBahaya  = (kiri  > 0 && kiri  < JARAK_BAHAYA_CM);
        bool kananBahaya = (kanan > 0 && kanan < JARAK_BAHAYA_CM);
        bool bahayaNow   = kiriBahaya || kananBahaya;

        portENTER_CRITICAL(&sensorMux);
        jarakKiri  = kiri;
        jarakKanan = kanan;
        bahaya     = bahayaNow;
        portEXIT_CRITICAL(&sensorMux);

        // Buzzer dinyalakan/dimatikan langsung di sini, tidak
        // menunggu giliran loop(), supaya reaksinya seketika.
        digitalWrite(BUZZER_PIN, bahayaNow ? HIGH : LOW);

        // delay kecil supaya task lain (termasuk gpsTask & loop())
        // tetap kebagian CPU. bacaJarak() sendiri sudah "memakan"
        // waktu s/d ~60ms (2x pulseIn timeout 30ms), jadi delay di
        // sini cukup singkat.
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void gpsTask(void *param)
{
    for (;;)
    {
        portENTER_CRITICAL(&gpsMux);
        while (GPSSerial.available() > 0)
        {
            gps.encode(GPSSerial.read());
        }
        // status LOCK harus benar-benar real-time, bukan "pernah
        // lock sekali lalu selamanya true".
        gpsLock = gps.location.isValid()
                  && gps.location.age() < 2000
                  && gps.satellites.isValid()
                  && gps.satellites.value() > 0;
        portEXIT_CRITICAL(&gpsMux);

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}


void updateOLED()
{
    
    portENTER_CRITICAL(&sensorMux);
    float kiri  = jarakKiri;
    float kanan = jarakKanan;
    bool  bhy   = bahaya;
    portEXIT_CRITICAL(&sensorMux);

    display.clearDisplay();

    display.setCursor(0, 0);
    display.print("Kiri : ");
    if (kiri < 0)
        display.print("--");
    else
    {
        display.print(kiri, 1);
        display.print(" cm");
    }

    display.setCursor(0, 10);
    display.print("Kanan: ");
    if (kanan < 0)
        display.print("--");
    else
    {
        display.print(kanan, 1);
        display.print(" cm");
    }

    display.setCursor(0, 20);
    display.print("GPS  : ");
    display.print(gpsLock ? "LOCK" : "SEARCH");

    display.setCursor(0, 29);
    display.print("SAT  : ");
    if (gps.satellites.isValid())
        display.print(gps.satellites.value());
    else
        display.print("--");

    display.setCursor(0, 38);
    display.print("CAM  : ");
    display.print(cameraOK ? "OK" : "GAGAL");

    display.setCursor(0, 47);
    display.print("STATUS: ");
    display.print(bhy ? "BAHAYA" : "AMAN");

    display.setCursor(0, 56);
    display.print("WiFi : ");
    display.print(WiFi.status() == WL_CONNECTED ? "OK" : "LOST");

    display.display();
}


void printSerialMonitor()
{
    portENTER_CRITICAL(&sensorMux);
    float kiri  = jarakKiri;
    float kanan = jarakKanan;
    bool  bhy   = bahaya;
    portEXIT_CRITICAL(&sensorMux);

    Serial.println("----------------------------------------");

    Serial.print("Sensor Kiri  : ");
    if (kiri < 0) Serial.println("--");
    else { Serial.print(kiri, 1); Serial.println(" cm"); }

    Serial.print("Sensor Kanan : ");
    if (kanan < 0) Serial.println("--");
    else { Serial.print(kanan, 1); Serial.println(" cm"); }

    Serial.print("Status Bahaya: ");
    Serial.println(bhy ? "BAHAYA" : "AMAN");

    Serial.print("GPS Status   : ");
    Serial.println(gpsLock ? "LOCK" : "SEARCHING...");

    Serial.print("Jumlah Satelit: ");
    if (gps.satellites.isValid()) Serial.println(gps.satellites.value());
    else Serial.println("--");

    if (gpsLock)
    {
        Serial.print("Latitude     : ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude    : ");
        Serial.println(gps.location.lng(), 6);
        Serial.print("Google Maps  : https://maps.google.com/?q=");
        Serial.print(gps.location.lat(), 6);
        Serial.print(",");
        Serial.println(gps.location.lng(), 6);
    }

    Serial.print("Kamera       : ");
    Serial.println(cameraOK ? "OK" : "GAGAL");

    Serial.print("WiFi         : ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    Serial.println("----------------------------------------");
}


void reconnectWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
        return;

    Serial.println("Reconnect WiFi...");

    WiFi.disconnect(true);
    delay(1000);
    WiFi.begin(ssid, password);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    if (WiFi.status() == WL_CONNECTED)
        Serial.println("WiFi Connected");
    else
        Serial.println("WiFi gagal connect");
}


void sendLocationTelegram()
{
    if (!gps.location.isValid())
    {
        bot.sendMessage(CHAT_ID, "Rompi Navigasi: kamera tidak aktif, GPS belum mendapatkan lokasi.", "");
        return;
    }

    String msg;
    msg += "ROMPI NAVIGASI (tanpa foto, kamera tidak aktif)\n\n";
    msg += "Latitude : "  + String(gps.location.lat(), 6) + "\n";
    msg += "Longitude: "  + String(gps.location.lng(), 6) + "\n\n";
    msg += "Satelit : ";
    msg += gps.satellites.isValid() ? String(gps.satellites.value()) : "--";
    msg += "\n\n";
    msg += "https://maps.google.com/?q=" + String(gps.location.lat(), 6) +
           "," + String(gps.location.lng(), 6);

    bot.sendMessage(CHAT_ID, msg, "");
}


bool sendPhotoTelegram()
{
    if (!cameraOK)
    {
        Serial.println("Kamera belum siap, coba init ulang...");
        cameraOK = initCamera();
    }

    if (!cameraOK)
    {
        Serial.println("Kamera tetap gagal, kirim lokasi saja");
        sendLocationTelegram();
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Gagal mengambil frame dari kamera");
        sendLocationTelegram();
        return false;
    }

    WiFiClientSecure client;
    client.setInsecure();

    if (!client.connect("api.telegram.org", 443))
    {
        Serial.println("Telegram gagal koneksi, fallback kirim lokasi saja");
        esp_camera_fb_return(fb);
        sendLocationTelegram();
        return false;
    }

    String boundary = "----ESP32CAM";

    String caption = "ROMPI NAVIGASI - EMERGENCY\n\n";
    if (gps.location.isValid())
    {
        caption += "Latitude : "  + String(gps.location.lat(), 6) + "\n";
        caption += "Longitude: "  + String(gps.location.lng(), 6) + "\n\n";
        caption += "https://maps.google.com/?q=" + String(gps.location.lat(), 6) +
                   "," + String(gps.location.lng(), 6);
    }
    else
    {
        caption += "GPS belum LOCK (foto tetap dikirim tanpa lokasi)";
    }

    String head;
    head += "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n";
    head += CHAT_ID;
    head += "\r\n";
    head += "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"caption\"\r\n\r\n";
    head += caption;
    head += "\r\n";
    head += "--" + boundary + "\r\n";
    head += "Content-Disposition: form-data; name=\"photo\"; filename=\"photo.jpg\"\r\n";
    head += "Content-Type: image/jpeg\r\n\r\n";

    String tail = "\r\n--" + boundary + "--\r\n";

    int totalLength = head.length() + fb->len + tail.length();

    client.println("POST /bot" + String(BOT_TOKEN) + "/sendPhoto HTTP/1.1");
    client.println("Host: api.telegram.org");
    client.println("Content-Type: multipart/form-data; boundary=" + boundary);
    client.println("Content-Length: " + String(totalLength));
    client.println();

    client.print(head);
    client.write(fb->buf, fb->len);
    client.print(tail);

    String statusLine = "";
    unsigned long timeout = millis();
    bool gotStatus = false;

    while (client.connected() && millis() - timeout < 10000)
    {
        while (client.available())
        {
            String line = client.readStringUntil('\n');
            if (!gotStatus)
            {
                statusLine = line;
                gotStatus = true;
            }
        }
    }

    client.stop();
    esp_camera_fb_return(fb);

    if (gotStatus)
        Serial.println("Response Telegram: " + statusLine);

    if (statusLine.indexOf("200") >= 0)
    {
        Serial.println("Foto berhasil terkirim");
        return true;
    }
    else
    {
        Serial.println("Kemungkinan foto GAGAL terkirim, cek koneksi/token/chat_id");
        return false;
    }
}


void cekButton()
{
    static bool lastState = HIGH;
    static unsigned long lastPress = 0;

    bool state = digitalRead(BUTTON_PIN);

    if (lastState == HIGH && state == LOW && millis() - lastPress > 1000)
    {
        lastPress = millis();

        Serial.println();
        Serial.println("=================================");
        Serial.println("TOMBOL EMERGENCY DITEKAN");
        Serial.println("=================================");

        reconnectWiFi();
        sendPhotoTelegram();   // sensorTask tetap jalan normal selama ini berlangsung
    }

    lastState = state;
}


bool initCamera()
{
    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk  = XCLK_GPIO_NUM;
    config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href  = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn  = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound())
    {
        Serial.println("PSRAM OK");
        config.frame_size   = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count     = 2;
        config.fb_location  = CAMERA_FB_IN_PSRAM;
        config.grab_mode    = CAMERA_GRAB_LATEST;
    }
    else
    {
        Serial.println("PSRAM TIDAK ditemukan! Cek Tools > PSRAM > OPI PSRAM di Arduino IDE.");
        config.frame_size   = FRAMESIZE_VGA;
        config.jpeg_quality = 15;
        config.fb_count     = 1;
        config.fb_location  = CAMERA_FB_IN_DRAM;
        config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init GAGAL, kode error: 0x%x\n", err);
        Serial.println("Kemungkinan penyebab:");
        Serial.println("- Wiring kamera longgar / salah pin");
        Serial.println("- Suplai daya kamera kurang stabil");
        Serial.println("- PSRAM belum diaktifkan di board settings");
        Serial.println("- XCLK frequency tidak cocok dengan modul OV2640 yang dipakai");
        return false;
    }

    Serial.println("Camera OK");
    return true;
}


void setup()
{
    Serial.begin(115200);
    delay(300);

    //================ GPS (paling awal) =================
    GPSSerial.setRxBufferSize(1024);
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

    xTaskCreatePinnedToCore(
        gpsTask,
        "GPS_Task",
        4096,
        NULL,
        3,                
        &gpsTaskHandle,
        1                
    );
    Serial.println("GPS Task dijalankan di Core 1, prioritas tinggi - START LEBIH AWAL");

    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    xTaskCreatePinnedToCore(
        sensorTask,
        "Sensor_Task",
        3072,
        NULL,
        2,                
        &sensorTaskHandle,
        1                 
    );
    Serial.println("Sensor Task dijalankan di Core 1, prioritas tinggi - obstacle detection tidak akan pernah berhenti");

    
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println("OLED tidak terdeteksi, cek wiring I2C");
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    
    WiFi.begin(ssid, password);
    Serial.print("Menghubungkan WiFi");

    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000)
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
    Serial.println(WiFi.status() == WL_CONNECTED ? "WiFi Connected" : "WiFi belum connect, lanjut...");

    secured_client.setInsecure();

    
    cameraOK = initCamera();
    Serial.println(cameraOK ? "Kamera siap" : "Kamera tidak aktif, akan dicoba lagi otomatis saat tombol ditekan / berkala");

    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("ROMPI NAVIGASI");
    display.setCursor(0, 20);
    display.println("Starting...");
    display.setCursor(0, 35);
    display.println(cameraOK ? "Camera: OK" : "Camera: GAGAL");
    display.display();
    delay(1500);

    gpsWasLocked = false;
}


void loop()
{
    portENTER_CRITICAL(&gpsMux);
    bool gpsLockNow = gpsLock;
    portEXIT_CRITICAL(&gpsMux);

    if (gpsLockNow && !gpsWasLocked)
    {
        Serial.println();
        Serial.println("=================================");
        Serial.println("GPS BARU SAJA LOCK -> kirim notifikasi otomatis");
        Serial.println("=================================");

        reconnectWiFi();
        sendPhotoTelegram();
    }
    gpsWasLocked = gpsLockNow;

    if (millis() - lastOLED > 500)
    {
        lastOLED = millis();
        updateOLED();
    }

    if (millis() - lastSerialMonitor > 1000)
    {
        lastSerialMonitor = millis();
        printSerialMonitor();
    }

    if (!cameraOK && millis() - lastCameraRetry > 10000)
    {
        lastCameraRetry = millis();
        Serial.println("Retry init kamera di background...");
        cameraOK = initCamera();
    }

    cekButton();
}
