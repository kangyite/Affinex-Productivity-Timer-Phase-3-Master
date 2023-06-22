#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include "at24c256.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include "time.h"
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <HardwareSerial.h>
// #include <WiFiClientSecure.h>

// #define DEBUG
#ifdef DEBUG
#define PRINT(x) Serial.print(x)
#define PRINTLN(x) Serial.println(x)
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINT(x)
#define PRINTLN(x)
#define PRINTF(...)
#endif

WiFiMulti wifiMulti;
hd44780_I2Cexp lcd;
at24c256 eeprom(0x50);
//===Pins===================================================
#define RUN_LED_pin 2
#define NEXT_SW_pin 23
#define G_LED_pin 14
#define Y_LED_pin 12
#define R_LED_pin 13
#define BUZZER_pin 19
#define MUTE_SW_pin 18
#define BARCODE_RX_pin 5
//==Wifi===================================
const char wifi_info[][2][30] = {
    {"Papaya_6", "steam123"},
    {"AFX_6", "8edef@16206"}
    // {"IP9tcN32kd", "H$u3e*X3-uByaL=BTXw_r45t&"}
};
const uint32_t connectTimeoutMs = 10000;

//=== Serial COM ================================================
enum
{
  COM_RX_NULL = 0,
  COM_RX_OK,
  COM_RX_NG
};
enum
{
  NOT_NUMBER = 0,
  VALUE_RANGE_OK,
  VALUE_RANGE_OUT
};
#define ASCII_CR 13
#define ASCII_LF 10
#define TERMINATION 0x0d // return

//=== Serial COM 0 ================================================
#define MAX_RX_BUFFER 80
char rx[MAX_RX_BUFFER];
byte rx_index = 0;
byte rx_status = 0;
char tx0[MAX_RX_BUFFER];

//=== Serial COM 2 ================================================
#define RXD2 16
#define TXD2 17
bool read_all_serial = true;
//=== Commands ====================================================
#define CMD_BUF 50
#define CMD_SIZE 4
char command[CMD_SIZE][CMD_BUF];
//===============================================================
#define MAX_SEQ_NUM 20
#define MAX_TIM_NUM MAX_SEQ_NUM
#define US MAX_TIM_NUM
enum
{
  T00 = 0,
  T01,
  T02,
  T03,
  T04,
  T05,
  T06,
  T07,
  T08,
  T09,
  T10,
  T11,
  T12,
  T13,
  T14,
  T15,
  T16,
  T17,
  T18,
  T19,
  U00,
  U01,
  U02,
  U03,
  U04,
  U05,
  U06,
  U07,
  U08,
  U09,
  U10,
  U11,
  U12,
  U13,
  U14,
  U15,
  U16,
  U17,
  U18,
  U19
};
enum
{
  TIMOUT = 0,
  TIMSET
};
unsigned long time_data[(MAX_TIM_NUM + US)], timeset[(MAX_TIM_NUM + US)];
byte t_status[(MAX_TIM_NUM + US)];
int seq[MAX_SEQ_NUM];
//===EEPROM====================================
#define EEP_TIMER 0x00          //-0x01
#define EEP_ACTUAL_COUNTER 0X02 //-0x03
#define EEP_PLAN 0X04           //-0x05
#define EEP_TARGET 0x06         //-0x07
#define EEP_OT 0x08
#define EEP_DEVICE_ID 0x09
#define EEP_BARCODE_COUNT 0x10 //-0x13
#define EEP_WORKING_TIME 0x20  //-0x84
#define EEP_OT_TIME 0x90       //-0xF4
bool eeprom_error = 0;

#define HOLD_TIME 3000

byte device_id;
uint16_t timer_set, timer_count_down, actual_counter, plan, target;
uint16_t _u_integer;

enum
{
  LCD_TIME = 0,
  LCD_PLAN,
  LCD_TARGET,
  LCD_ACTUAL
};
int LCD_DATA_LEN = 4;
bool times_up = false;
bool second_changed = false;
bool minute_changed = false;
bool mute_toggle = false;
//=== TIME===========================================
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 8 * 60 * 60;
const int daylightOffset_sec = 0;
struct tm timeinfo;
#define DEFAULT_WORKING_TIME "0830-1300,1400-1530,1540-1800"
#define DEFAULT_OT_TIME "1830-2030"
#define WORKING_TIME_LENGTH 100
char working_time[WORKING_TIME_LENGTH];
char ot_time[WORKING_TIME_LENGTH];

unsigned long total_working_minute = 0;
bool ot = 0;
bool set_up = 0;
//===Tasks==========================================
TaskHandle_t Task0;

//==Firebase==========================================
#define API_KEY "AIzaSyDce7WDB1VU_wXUZTRrNzAJlmmtMIBnGM0"
#define DATABASE_URL "https://affinex-productivity-timer-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define USER_EMAIL "kangyte@gmail.com"
#define USER_PASSWORD "123123"
FirebaseData fbdo;
FirebaseData stream;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;
bool force_update = false;

//===Software Serial for barcode====================================

#define MYPORT_RX BARCODE_RX_pin
HardwareSerial myPort(1);

char barcodeBuf[20];
int barcodeIdx = 0;
uint32_t barcodeCount = 0;

//===================FUNCTIONS DECLARATION=================================
void init_io();
void print_the_date_and_file_of_sketch();
void serialEvent();
void serialEvent2();
void seq0();
void seq1();
void seq2();
void seq3();
void seq4();
void seq5();
void seq6();
void seq7();
void seq10();
void seq11();
template <class T>
int EEPROM_read(int ee, T &value);
template <class T>
int EEPROM_write(int ee, const T &value);
bool is_working();
unsigned int count_working_minute();
unsigned int count_cur_working_minute();
void printLocalTime();
void print_help();
bool isEqual(char arr[], String s);
unsigned int toUInt(char cstr[]);
String four_digit(unsigned int count);
String str_format_spacing(String &s);
void slave_update(byte data);
byte rx_command(char rx[]);
byte timer_status(byte temp_tim_num);
byte set_timer(byte temp_tim_num, unsigned long temp_tim_set);
template <typename T>
byte check_decimal_range(char temp_x1000, char temp_x100, char temp_x10, char temp_x1, int temp_min, int temp_max, T &temp_out_value);
void rx_clear(void);
void timer_reset();
void lcd_update(byte lcd_data);
void Task0code(void *parameter);
unsigned long get_epoch_time();
String get_date();
void update_data();
void streamCallback(FirebaseStream data);
void streamTimeoutCallback(bool timeout);
void scan_i2c();

//===================================================================
void setup()
{
  byte i;
  for (i = 0; i < (MAX_TIM_NUM + US); i++)
  {
    t_status[i] = TIMOUT;
  }
  for (i = 0; i < MAX_SEQ_NUM; i++)
  {
    seq[i] = 0;
  }

  init_io();
  Serial.begin(115200);
  Serial.println();
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println(F("Program Begin..."));
  print_the_date_and_file_of_sketch();
  Wire.begin();
  Serial.println("\nI2C Scanner");
  scan_i2c();
  serialEvent();
  serialEvent2();
  eeprom.init();
  lcd.begin(20, 4);
  lcd.setCursor(0, 0);
  // 0123456789012345
  lcd.print("Productivity    ");
  lcd.setCursor(0, 1);
  lcd.print("Timer & Counter ");

  delay(2000);

  EEPROM_read(EEP_DEVICE_ID, device_id);

  EEPROM_read(EEP_TIMER, timer_set);
  if (timer_set > 10000)
  {
    timer_set = 300;
    EEPROM_write(EEP_TIMER, timer_set);
  }

  EEPROM_read(EEP_ACTUAL_COUNTER, actual_counter);
  if (actual_counter > 10000)
  {
    actual_counter = 0;
    EEPROM_write(EEP_ACTUAL_COUNTER, actual_counter);
  }

  EEPROM_read(EEP_PLAN, plan);
  if (plan > 10000)
  {
    plan = 500;
    EEPROM_write(EEP_PLAN, plan);
  }

  EEPROM_read(EEP_TARGET, target);
  if (target > 10000)
  {
    target = 0;
    EEPROM_write(EEP_TARGET, target);
  }

  EEPROM_read(EEP_OT, ot);

  EEPROM_read(EEP_WORKING_TIME, working_time);
  if (working_time[0] == 255)
    memcpy(working_time, DEFAULT_WORKING_TIME, sizeof(DEFAULT_WORKING_TIME));

  EEPROM_read(EEP_OT_TIME, ot_time);
  if (ot_time[0] == 255)
    memcpy(ot_time, DEFAULT_OT_TIME, sizeof(DEFAULT_OT_TIME));
  // Serial.println(working_time);
  Serial2.print("@spacing 0\r");
  slave_update(LCD_TARGET);
  slave_update(LCD_PLAN);
  slave_update(LCD_ACTUAL);

  EEPROM_read(EEP_BARCODE_COUNT, barcodeCount);

  xTaskCreatePinnedToCore(
      Task0code, /* Function to implement the task */
      "Task0",   /* Name of the task */
      10000,     /* Stack size in words */
      NULL,      /* Task input parameter */
      0,         /* Priority of the task */
      &Task0,    /* Task handle. */
      0);

  if (device_id >= 255)
  {
    Serial.println("This Device haven't set its id, please set it by using @write id x");
  }
  else
  {
    set_up = 1;
    Serial.print("Device ID: ");
    Serial.println(device_id);
  }
  timer_reset();

  myPort.begin(115200, SERIAL_8N1, 5);
}

void Task0code(void *parameter)
{
  while (set_up == 0)
  {
    delay(500);
  }
  WiFi.mode(WIFI_STA);
  for (int i = 0; i < (sizeof(wifi_info) / sizeof(wifi_info[0])); i++)
  {
    wifiMulti.addAP(wifi_info[i][0], wifi_info[i][1]);
  }
  Serial.println("Connecting Wifi...");
  if (wifiMulti.run() == WL_CONNECTED)
  {
    Serial.println("");
    Serial.print("WiFi connected to: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  Firebase.begin(&config, &auth);

  Firebase.reconnectWiFi(true);

  if (!Firebase.RTDB.beginStream(&stream, "devices/TIMER_" + String(device_id) + "/data/"))
    Serial.printf("stream begin error, %s\n\n", stream.errorReason().c_str());
  // Assign a calback function to run when it detects changes on the database
  Firebase.RTDB.setStreamCallback(&stream, streamCallback, streamTimeoutCallback);
  for (;;)
  {
    seq10();
    seq11();
  }
}

void loop()
{
  if (!set_up)
  {
    Serial.println("This Device haven't set its id, please set it by using @write id x");
    delay(1000);
    return;
  }

  seq0(); // Heart Beat
  seq1(); // Timer
  seq2(); // Buttons
  seq3(); // LCD
  seq4(); // LEDs
  seq5(); // Buzzer
  seq6(); // Barcode
  seq7(); // Target
}

void seq0(void)
{
  switch (seq[0])
  {
  case 0:
    if (timer_status(T00) == TIMOUT)
    {
      set_timer(T00, 500);
      digitalWrite(RUN_LED_pin, HIGH);
      seq[0] = 10;
    }
    break;
  case 10:
    if (timer_status(T00) == TIMOUT)
    {
      set_timer(T00, 500);
      digitalWrite(RUN_LED_pin, LOW);
      seq[0] = 0;
    }
    break;
  default:
    break;
  }
}
void seq1(void)
{
  switch (seq[1])
  {
  case 0:
    if (timer_status(T01) == TIMOUT)
    {
      set_timer(T01, 1000);

      if (timer_count_down <= 0)
        times_up = true;
      else
      {
        timer_count_down--;
        times_up = false;
        second_changed = true;
      }
    }
    break;
  case 10:
    break;
  case 20:
    break;
  default:
    break;
  }
}
void seq2(void)
{
  switch (seq[2])
  {
  case 0:
    if (timer_status(T02) == TIMOUT)
    {
      if (digitalRead(NEXT_SW_pin) == LOW)
      {
        set_timer(T02, 30);
        seq[2] = 100;
      }
      else if (digitalRead(MUTE_SW_pin) == LOW)
      {
        set_timer(T02, 30);
        seq[2] = 300;
      }
    }
    break;
  case 100:
    if (timer_status(T02) == TIMOUT)
    {
      set_timer(T03, HOLD_TIME);
      seq[2] = 110;
    }
    break;
  case 110:
    if (timer_status(T03) == TIMOUT)
    { // hold
      actual_counter = 0;
      EEPROM_write(EEP_ACTUAL_COUNTER, (uint16_t)0);
      timer_reset();
      force_update = true;
      seq[3] = 20;
      seq[2] = 120;
    }
    else if (digitalRead(NEXT_SW_pin) == HIGH)
    { // press
      seq[2] = 200;
    }
    break;
  case 120:
    if (digitalRead(NEXT_SW_pin) == HIGH)
    {
      set_timer(T02, 30);
      seq[2] = 0;
    }
    break;
  case 200:
    actual_counter++;
    mute_toggle = false;
    EEPROM_write(EEP_ACTUAL_COUNTER, actual_counter);
    timer_reset();
    lcd_update(LCD_ACTUAL);
    slave_update(LCD_ACTUAL);
    seq[2] = 120;
    break;
  case 300:
    if (timer_status(T02) == TIMOUT)
    {
      if (digitalRead(MUTE_SW_pin) == HIGH)
      {
        set_timer(T02, 30);
        mute_toggle = !mute_toggle;

        seq[2] = 0;
      }
    }

    break;
  default:
    break;
  }
}

void seq3(void)
{
  switch (seq[3])
  {
  case 0:
    lcd.setCursor(0, 0);
    lcd.print("Time left  :    ");
    lcd.setCursor(0, 1);
    lcd.print("Plan       :         ");
    lcd.setCursor(0, 2);
    lcd.print("Target     :");
    lcd.setCursor(0, 3);
    lcd.print("Actual     :");
    seq[3] = 20;
    break;
  case 10:
    if (second_changed)
      seq[3] = 30;
    break;
  case 20:
    lcd_update(LCD_TIME);
    lcd_update(LCD_PLAN);
    lcd_update(LCD_TARGET);
    lcd_update(LCD_ACTUAL);
    slave_update(LCD_TARGET);
    slave_update(LCD_PLAN);
    slave_update(LCD_ACTUAL);
    seq[3] = 10;
    break;
  case 30:
    lcd_update(LCD_TIME);
    second_changed = 0;
    seq[3] = 10;
    break;
  case 40:
    lcd_update(LCD_PLAN);
    seq[3] = 10;
    break;
  case 50:
    lcd_update(LCD_TARGET);
    seq[3] = 10;
    break;
  case 60:
    lcd_update(LCD_ACTUAL);
    seq[3] = 10;
    break;
  default:
    break;
  }
}

void seq4(void)
{
  switch (seq[4])
  {
  case 0:
    if (timer_status(T04) == TIMOUT)
    {
      if ((float)timer_count_down / (float)timer_set > 0.3)
        seq[4] = 100;
      else if ((float)timer_count_down / (float)timer_set > 0.1)
        seq[4] = 200;
      else
        seq[4] = 300;
    }
    break;
  case 100:
    digitalWrite(G_LED_pin, HIGH);
    digitalWrite(Y_LED_pin, LOW);
    digitalWrite(R_LED_pin, LOW);
    seq[4] = 0;
    break;
  case 200:
    digitalWrite(Y_LED_pin, HIGH);
    digitalWrite(G_LED_pin, LOW);
    digitalWrite(R_LED_pin, LOW);
    set_timer(T04, 500);
    seq[4] = 210;
    break;
  case 210:
    if (timer_status(T04) == TIMOUT)
    {
      digitalWrite(Y_LED_pin, LOW);
      set_timer(T04, 500);
      seq[4] = 0;
    }
    break;
  case 300:
    digitalWrite(R_LED_pin, HIGH);
    digitalWrite(G_LED_pin, LOW);
    digitalWrite(Y_LED_pin, LOW);
    set_timer(T04, 500);
    seq[4] = 310;
    break;
  case 310:
    if (timer_status(T04) == TIMOUT)
    {
      digitalWrite(R_LED_pin, LOW);
      set_timer(T04, 500);
      seq[4] = 0;
    }
    break;
  default:
    break;
  }
}

void seq5()
{
  switch (seq[5])
  {
  case 0:
    if (timer_status(T05) == TIMOUT)
    {
      if (times_up)
        seq[5] = 100;
    }
    break;
  case 100:
    if (timer_status(T05) == TIMOUT)
    {
      if (mute_toggle == false)
      {
        digitalWrite(BUZZER_pin, HIGH);
        set_timer(T05, 100);
        seq[5] = 110;
      }
      else
        seq[5] = 0;
    }
    break;
  case 110:
    if (timer_status(T05) == TIMOUT)
    {
      digitalWrite(BUZZER_pin, LOW);
      set_timer(T05, 100);
      seq[5] = 120;
    }
    break;
  case 120:
    if (timer_status(T05) == TIMOUT)
    {
      digitalWrite(BUZZER_pin, HIGH);
      set_timer(T05, 100);
      seq[5] = 130;
    }
    break;
  case 130:
    if (timer_status(T05) == TIMOUT)
    {
      digitalWrite(BUZZER_pin, LOW);
      set_timer(T05, 4700);
      seq[5] = 0;
    }
    break;
  default:
    break;
  }
}

void seq6()
{
  switch (seq[6])
  {
  case 0:
    if (myPort.available())
    {
      barcodeBuf[barcodeIdx] = (char)myPort.read();
      PRINTF("c: %d\n", barcodeBuf[barcodeIdx]);
      if (barcodeIdx >= 20)
      {
        seq[6] = 100;
      }
      else
      {
        seq[6] = 10;
      }
    }
    break;
  case 10:
    if (barcodeBuf[barcodeIdx] == '\n')
    {
      PRINTLN("HERE");
      unsigned long num = 0;
      bool flag = 1;
      for (int i = 0; i < barcodeIdx; i++)
      {
        if (barcodeBuf[i] < '0' || barcodeBuf[i] > '9')
        { // not valid
          PRINTLN("not valid");
          seq[6] = 100;
          flag = 0;
          break;
        }
      }
      if (flag == 0)
        break;
      barcodeBuf[barcodeIdx] = '\0';
      for (int i = 0; i < 5; i++)
      {
        PRINTF("%d: %d\n", i, barcodeBuf[i]);
      }
      long tempCount = atol(barcodeBuf);
      PRINTF("tempC: %u\n", tempCount);
      if (tempCount == barcodeCount + 1)
      {
        seq[2] = 200;
      }
      else if (tempCount > barcodeCount + 1)
      {
        actual_counter += tempCount - (barcodeCount + 1);
        seq[2] = 200;
      }
      else
      {
        actual_counter += tempCount - 1;
        seq[2] = 200;
      }
      barcodeCount = tempCount;
      EEPROM_write(EEP_BARCODE_COUNT, barcodeCount);
      seq[6] = 100;
    }
    else if (barcodeBuf[barcodeIdx] > '9' || barcodeBuf[barcodeIdx] < '0')
      seq[6] = 100;
    else
    {
      barcodeIdx++;
      seq[6] = 0;
    }
    break;

  case 100:
    PRINTLN("reseting barcode buffer");
    barcodeIdx = 0;
    while (myPort.available())
      myPort.read();
    seq[6] = 0;
    break;
  default:
    break;
  }
}

void seq7()
{
  switch (seq[7])
  {
    case 0:
      set_timer(T07, timer_set*1000);
      seq[7] = 100;
      break;
    case 100:
      if (timer_status(T07) == TIMOUT)
      {
        set_timer(T07, timer_set*1000);
        if(target<plan && is_working()) {
          target++;
          EEPROM_write(EEP_TARGET, target);
          slave_update(LCD_TARGET);
          lcd_update(LCD_TARGET);
        }
      }
      seq[7] = 100;
      break;
    default:
      break;
  }
}

void seq10(void)
{
  switch (seq[10])
  {
  case 0:
    if (timer_status(T10) == TIMOUT)
    {
      set_timer(T10, 20000);
      if (WiFi.status() != WL_CONNECTED)
      {
        wifiMulti.run();
      }
    }
    seq[10] = 0;
    break;
  case 10:
    break;
  case 20:
    break;
  default:
    break;
  }
}
void seq11(void)
{
  switch (seq[11])
  {
  case 0:
    if (timer_status(T11) == TIMOUT || force_update)
    {
      set_timer(T11, 1000);
      getLocalTime(&timeinfo);
      // printLocalTime();
      if (force_update)
      {
        Serial.println("Forcing Update");
        seq[11] = 100;
        force_update = false;
      }
      if (timeinfo.tm_sec == 0)
        seq[11] = 200;
      if (timeinfo.tm_min % 5 == 0 && timeinfo.tm_sec == 0)
        seq[11] = 100;
    }

    break;
  case 100:
  {
    if (Firebase.ready())
    {
      String parent_path = "/devices/TIMER_" + String(device_id) + "/data_logger/" + get_date() + "/" + String(get_epoch_time());
      json.clear();
      json.set("/epoch_time", get_epoch_time());
      json.set("/target", target);
      json.set("/actual", actual_counter);
      // Serial.println("Set json...\n");
      if (Firebase.RTDB.updateNodeAsync(&fbdo, parent_path.c_str(), &json))
      {
        Serial.println("Updated to firebase data logger");
      }
      else
      {
        Serial.println(fbdo.errorReason().c_str());
      }
      seq[11] = 200;
    }
    break;
  }
  case 200:
  {
    if (timer_status(T11) == TIMOUT && Firebase.ready())
    {
      // FirebaseJson updateData;
      json.clear();
      String parent_path = "/devices/TIMER_" + String(device_id) + "/data";
      json.set("/actual", actual_counter);
      json.set("/id", device_id);
      json.set("/plan", plan);
      json.set("/target", target);
      json.set("/timer_set", timer_set);
      json.set("/working_time", working_time);
      json.set("/ot_time", ot_time);
      json.set("/forceupdate", force_update);
      json.set("/enable_ot", ot);
      json.set("/online", true);
      json.set("/lastupdate", get_epoch_time());
      // Serial.println("Set json...\n");

      if (Firebase.RTDB.updateNodeAsync(&fbdo, parent_path.c_str(), &json))
      {
        Serial.println("Updated to firebase database");
      }
      else
      {
        Serial.println(fbdo.errorReason().c_str());
      }
      seq[11] = 0;
    }
    break;
  }
  default:
    break;
  }
}

//== Custom  Function ============================================================
unsigned long get_epoch_time()
{
  time_t now;
  struct tm _timeinfo;
  if (!getLocalTime(&_timeinfo))
  {
    return (0);
  }
  time(&now);
  return now;
}
String get_date()
{
  String res = "";
  res += String(timeinfo.tm_mday) + "-" + String(timeinfo.tm_mon + 1) + "-" + String(timeinfo.tm_year + 1900);
  return res;
}

bool check_working_time_format(char s[])
{
  int _sz = 100;
  for (int i = 0; i < _sz; i++)
  {
    if (s[i] == 0)
      return true;
    if ((i + 1) % 5 == 0)
    {
      if (s[i] == '-' || s[i] == ',')
        continue;
      else
        return false;
    }
    if (!('0' <= s[i] && s[i] <= '9'))
      return false;
  }
  return true;
}

bool is_working()
{
  String _s_working_time = working_time;
  String _s_ot_time = ot_time;
  String _min = (String)timeinfo.tm_min;
  String _hour = (String)timeinfo.tm_hour;
  String _now = _hour + (_min.length() == 1 ? "0" + _min : _min);
  for (int i = 0; _s_working_time.length(); i += 10)
  {
    if (_s_working_time[i] == 0 || _s_working_time[i] == 255)
      break;
    if (atoi(_s_working_time.substring(i, i + 4).c_str()) <= atoi(_now.c_str()) && atoi(_now.c_str()) < atoi(_s_working_time.substring(i + 5, i + 5 + 4).c_str()))
    {
      return true;
    }
  }
  if (!ot)
    return false;
  for (int i = 0; _s_ot_time.length(); i += 10)
  {
    if (_s_ot_time[i] == 0 || _s_ot_time[i] == 255)
      break;
    if (atoi(_s_ot_time.substring(i, i + 4).c_str()) <= atoi(_now.c_str()) && atoi(_now.c_str()) < atoi(_s_ot_time.substring(i + 5, i + 5 + 4).c_str()))
    {
      return true;
    }
  }
  return false;
}
unsigned int count_cur_working_minute()
{
  int _min = timeinfo.tm_min;   // 0830-1300,1400-1530,1540-1800
  int _hour = timeinfo.tm_hour; //
  int _time = _hour * 60 + _min;

  unsigned int cur_total_min = 0;
  for (int i = 0; i < strlen(working_time); i += 10)
  {
    if (working_time[i] == 0 || working_time[i] == 255)
      break;
    int from_time = ((working_time[i] - '0') * 10 + (working_time[i + 1] - '0')) * 60 + (working_time[i + 2] - '0') * 10 + (working_time[i + 3] - '0');
    int to_time = ((working_time[i + 5] - '0') * 10 + (working_time[i + 6] - '0')) * 60 + (working_time[i + 7] - '0') * 10 + (working_time[i + 8] - '0');
    // Serial.printf("from_time: %d to_time: %d", from_time, to_time);
    if (_time <= from_time)
      break;
    else if (_time <= to_time)
    {
      cur_total_min += (_time - from_time);
      break;
    }
    else
    {
      cur_total_min += (to_time - from_time);
    }
  }

  if (ot)
  {
    for (int i = 0; i < strlen(ot_time); i += 10) // 1200-1230
    {

      if (ot_time[i] == 0 || ot_time[i] == 255)
        return cur_total_min;
      int from_time = ((ot_time[i] - '0') * 10 + (ot_time[i + 1] - '0')) * 60 + (ot_time[i + 2] - '0') * 10 + (ot_time[i + 3] - '0');
      int to_time = ((ot_time[i + 5] - '0') * 10 + (ot_time[i + 6] - '0')) * 60 + (ot_time[i + 7] - '0') * 10 + (ot_time[i + 8] - '0');
      if (_time <= from_time)
        return cur_total_min;
      else if (_time <= to_time)
      {
        cur_total_min += (_time - from_time);
        return cur_total_min;
      }
      else
      {
        cur_total_min += (to_time - from_time);
      }
    }
  }
  return cur_total_min;
}
template <class T>
int EEPROM_write(int ee, const T &value)
{
  T reads;
  if(EEPROM_read(ee, reads) == -1){
    return -1;
  }
  if (value == reads)
    return -1;

  const byte *p = (const byte *)(const void *)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
  {
    eeprom.write(ee++, *p++);
  }

  return i;
}

template <class T>
int EEPROM_read(int ee, T &value)
{
  if(eeprom.read(ee) == -1) {
    Serial.println("EEPROM ERROR");
    eeprom_error = 1;
    return -1;
  }
  byte *p = (byte *)(void *)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
  {
    *p++ = eeprom.read(ee++);
  }
  return i;
}

unsigned int count_working_minute()
{ // 0800-1130,
  unsigned int res = 0;
  int _sz = sizeof(working_time) / sizeof(char);
  int _num_of_time = 0;
  for (int i = 0; i < _sz; i++)
  {
    if (working_time[i] == 0)
      break;
    if (working_time[i] == '-')
      _num_of_time++;
  }
  for (int i = 0; i < _num_of_time; i++)
  {
    res += (((working_time[i * 10 + 5] - '0') * 10 + (working_time[i * 10 + 6] - '0')) - ((working_time[i * 10] - '0') * 10 + (working_time[i * 10 + 1] - '0'))) * 60;
    res += ((working_time[i * 10 + 7] - '0') * 10 + (working_time[i * 10 + 8] - '0')) - ((working_time[i * 10 + 2] - '0') * 10 + (working_time[i * 10 + 3] - '0'));
  }
  if (ot == 0)
    return res;
  _sz = sizeof(ot_time) / sizeof(char);
  _num_of_time = 0;
  for (int i = 0; i < _sz; i++)
  {
    if (ot_time[i] == 0)
      break;
    if (ot_time[i] == '-')
      _num_of_time++;
  }
  for (int i = 0; i < _num_of_time; i++)
  {
    res += (((ot_time[i * 10 + 5] - '0') * 10 + (ot_time[i * 10 + 6] - '0')) - ((ot_time[i * 10] - '0') * 10 + (ot_time[i * 10 + 1] - '0'))) * 60;
    res += ((ot_time[i * 10 + 7] - '0') * 10 + (ot_time[i * 10 + 8] - '0')) - ((ot_time[i * 10 + 2] - '0') * 10 + (ot_time[i * 10 + 3] - '0'));
  }

  return res;
}
void printLocalTime()
{
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time 1");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S zone %Z %z ");
}
void print_help()
{

  Serial.println(F("=========================================================================================================================="));
  Serial.println(F("|Commands                            | Description                                                                        |"));
  Serial.println(F("|@help                               | Menu of commands                                                                   |"));
  Serial.println(F("|@read DATA                          | View DATA                                                                          |"));
  Serial.println(F("|                                    | DATA = id, view device id                                                          |"));
  Serial.println(F("|                                    | DATA = timer, view timer                                                           |"));
  Serial.println(F("|                                    | DATA = plan, view plan                                                             |"));
  Serial.println(F("|                                    | DATA = actual, view actual                                                         |"));
  Serial.println(F("|                                    | DATA = target, view target                                                         |"));
  Serial.println(F("|                                    | DATA = working_time, view working time                                             |"));
  Serial.println(F("|                                    | DATA = ot_time, view ot time                                                       |"));
  Serial.println(F("|                                    | DATA = ot, view state of ot                                                        |"));
  Serial.println(F("|@write DATA x                       | Overwrite DATA with x                                                              |"));
  Serial.println(F("|                                    | DATA = id, change id to x (0-255)                                                  |"));
  Serial.println(F("|                                    | DATA = timer, change timer to x (0-9999)                                                             |"));
  Serial.println(F("|                                    | DATA = plan, change plan to x (0-9999)                                             |"));
  Serial.println(F("|                                    | DATA = actual, change actual to x (0-9999)                                         |"));
  // Serial.println(F("|                                    | DATA = target, change target to x (0-9999)                                         |"));
  Serial.println(F("|                                    | DATA = working_time, change working time to x (e.g. 0830-1300,1400-1530,1540-1800) |"));
  Serial.println(F("|                                    | DATA = ot_time, change ot time to x (e.g. 1830-2030)                               |"));
  Serial.println(F("|                                    | DATA = ot, change the state of ot to x (0/1)                                       |"));
  Serial.println(F("|                                                                                                                         |"));
  // Serial.println(F("| FOR CUSTOMIZE DISPLAY : '|' = one space, '~' = two space                 |"));
  Serial.println(F("==========================================================================================================================="));
}

bool isEqual(char arr[], String s)
{
  return strcmp(arr, s.c_str()) == 0;
}

unsigned int toUInt(char cstr[])
{
  const int sz = strlen(cstr);
  long res = 0;
  if (sz >= 6)
    return -1;
  for (int i = 0; i < sz; i++)
  {
    if (cstr[i] < '0' || cstr[i] > '9')
      return -1;
    res += (cstr[i] - '0') * pow(10, sz - i - 1);
  }
  if (res > 65535 || res < 0)
    return -1;
  return res;
}
String four_digit(unsigned int count)
{
  const int len = String(count).length();
  String out_str = "";
  if (len >= 5)
    return "OVER";
  for (int i = 0; i < 4 - len; i++)
    out_str += ' ';
  return out_str + String(count);
}
String str_format_spacing(String &s)
{
  String res = "";
  bool put = 0;
  for (auto c : s)
  {

    if (c == '[')
    {
      res += c;
      res += '~';
      put = 1;
      continue;
    }
    else if (c == ']')
    {
      res += c;
      put = 0;
      break;
    }
    if (put)
    {
      if (c == ':')
      {
        res += '|';
        res += c;
        res += '|';
      }
      else
      {
        res += c;
      }
      res += "~";
    }
    else
      res += c;
  }
  return res;
}
void slave_update(byte data)
{
  String out_str = "";
  if (data == LCD_PLAN)
  {
    out_str = "@text 0 [PLAN:" + four_digit(plan) + "]";
  }
  else if (data == LCD_TARGET)
  {
    out_str = "@text 1 [TGT :" + four_digit(target) + "]";
  }
  else if (data == LCD_ACTUAL)
  {
    char sign;
    if(actual_counter == target) sign = '=';
    else if(actual_counter>target) sign = '+';
    else sign = '-';
    out_str = "@text 2 [ACT" + String(sign) + ":" + four_digit(actual_counter) + "]";
  }
  else
    return;
  Serial2.print(str_format_spacing(out_str) + "\r");
}

//==========
void print_the_date_and_file_of_sketch(void)
{
  // Print compile time and file name to serial monitor
  Serial.print("Compiled on ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.println(__TIME__);

  // Print sketch file name without directory to serial monitor
  Serial.print("Sketch file: ");
  String fileName = __FILE__;
  char *baseName = strrchr(fileName.c_str(), '\\');
  if (baseName != NULL)
  {
    baseName++;
    Serial.println(baseName);
  }
  else
  {
    Serial.println(fileName);
  }
}

//== Serial COMM 0  Function ============================================================
// UART0

byte rx_command(char rx[])
{
  for (int i = 0; i < CMD_SIZE; i++)
    command[i][0] = 0; // reset command array
  String in_str = rx;
  byte sz = in_str.length();
  bool in_msg = 0;
  byte start_idx = 0;
  byte cmd_idx = 0;
  // bool skip_next = 0;
  for (int i = 0; i < sz; i++)
  {
    // if(skip_next) {skip_next=0; continue;}
    if (in_str[i] == '[')
    {
      in_msg = 1;
    }
    if (in_str[i] == ']')
    {
      in_msg = 0;
      // in_str.substring(start_idx,i).toCharArray((command[cmd_idx]), CMD_BUF);
      // skip_next=1;
    }
    else if ((in_str[i] == ' ' && !in_msg) || i == sz - 1)
    {
      if (cmd_idx >= CMD_SIZE)
      {
        Serial.println("COMMAND TOO MUCH ARGUMENTS");
        return COM_RX_NG;
      }
      if (i - start_idx >= CMD_BUF - 1)
      {
        Serial.println("COMMAND TOO LONG");
        return COM_RX_NG;
      }
      in_str.substring(start_idx, i).toCharArray((command[cmd_idx]), CMD_BUF);
      cmd_idx++;
      start_idx = i + 1;
    }
  }
  rx_status = COM_RX_NG;
  if (isEqual(command[0], "@help"))
  {
    print_help();
    return COM_RX_OK;
  }
  else if (isEqual(command[0], "@read"))
  { // plan, timer, target , counter
    if (isEqual(command[1], "id"))
    {
      EEPROM_read(EEP_DEVICE_ID, _u_integer);
      Serial.print("DEVICE ID: ");
      Serial.println(_u_integer);
      return COM_RX_OK;
    }
    if (isEqual(command[1], "plan"))
    {
      EEPROM_read(EEP_PLAN, _u_integer);
      Serial.print("PLAN: ");
      Serial.println(_u_integer);
      return COM_RX_OK;
    }
    else if (isEqual(command[1], "timer"))
    {
      EEPROM_read(EEP_TIMER, _u_integer);
      Serial.print("TIMER: ");
      Serial.println(_u_integer);
      return COM_RX_OK;
    }
    else if (isEqual(command[1], "target"))
    {
      EEPROM_read(EEP_TARGET, _u_integer);
      Serial.print("TARGET: ");
      Serial.println(_u_integer);
      return COM_RX_OK;
    }
    else if (isEqual(command[1], "actual"))
    {
      EEPROM_read(EEP_ACTUAL_COUNTER, _u_integer);
      Serial.print("ACTUAL_COUNTER: ");
      Serial.println(_u_integer);
      return COM_RX_OK;
    }
    else if (isEqual(command[1], "working_time"))
    {
      EEPROM_read(EEP_WORKING_TIME, working_time);
      Serial.print("WORKING_TIME: ");
      Serial.println(working_time);
      return COM_RX_OK;
    }
    else if (isEqual(command[1], "ot_time"))
    {
      EEPROM_read(EEP_OT_TIME, ot_time);
      Serial.print("OT_TIME: ");
      Serial.println(ot_time);
      return COM_RX_OK;
    }
    else if (isEqual(command[1], "ot"))
    {
      EEPROM_read(EEP_OT, ot);
      Serial.print("OT: ");
      Serial.println(ot);
      return COM_RX_OK;
    }
  }
  else if (isEqual(command[0], "@write"))
  {
    if (isEqual(command[1], "id"))
    {
      if (toUInt(command[2]) == -1)
        return COM_RX_NG;
      _u_integer = toUInt(command[2]);
      if (_u_integer > 255)
        return COM_RX_NG;
      device_id = _u_integer;
      EEPROM_write(EEP_DEVICE_ID, _u_integer);
      Serial.print("Writen DEVICE ID: ");
      Serial.println(_u_integer);
      set_up = 1;
      return COM_RX_OK;
    }
    else if (isEqual(command[1], "plan"))
    {
      if (toUInt(command[2]) == -1)
        return COM_RX_NG;
      _u_integer = toUInt(command[2]);
      if (_u_integer > 9999)
        return COM_RX_NG;
      EEPROM_write(EEP_PLAN, _u_integer);
      Serial.print("Writen PLAN: ");
      Serial.println(_u_integer);
      plan = _u_integer;
      lcd_update(LCD_PLAN);
      slave_update(LCD_PLAN);
      return COM_RX_OK;
    }
    else if (isEqual(command[1], "timer"))
    {
      if (toUInt(command[2]) == -1)
        return COM_RX_NG;
      _u_integer = toUInt(command[2]);
      if (_u_integer > 9999)
        return COM_RX_NG;
      EEPROM_write(EEP_TIMER, _u_integer);
      timer_set = _u_integer;
      seq[7] = 0; // update target delay
      timer_reset();

      Serial.print("Writen TIMER: ");
      Serial.println(_u_integer);
      return COM_RX_OK;
    }
    // else if (isEqual(command[1], "target"))
    // {
    //     if (toUInt(command[2]) == -1)
    //         return COM_RX_NG;
    //     _u_integer = toUInt(command[2]);
    //     if (_u_integer > 9999)
    //         return COM_RX_NG;
    //     EEPROM_write(EEP_TARGET, _u_integer);
    //     Serial.print("Writen TARGET: ");
    //     Serial.println(_u_integer);
    //     target = _u_integer;
    //     lcd_update(LCD_TARGET);
    //     slave_update(LCD_TARGET);
    //     return COM_RX_OK;
    // }
    else if (isEqual(command[1], "actual"))
    {
      if (toUInt(command[2]) == -1)
        return COM_RX_NG;
      _u_integer = toUInt(command[2]);
      if (_u_integer > 9999)
        return COM_RX_NG;
      EEPROM_write(EEP_ACTUAL_COUNTER, _u_integer);
      Serial.print("Writen ACTUAL_COUNTER: ");
      Serial.println(_u_integer);
      actual_counter = _u_integer;
      timer_reset();
      lcd_update(LCD_ACTUAL);
      slave_update(LCD_ACTUAL);
      return COM_RX_OK;
    }
    else if (isEqual(command[1], "working_time"))
    {
      if (check_working_time_format(command[2]))
      {
        int _sz = 0;
        for (int c : command[2])
        {
          if (c == 0)
            break;
          _sz++;
        }
        memcpy(working_time, command[2], _sz + 1);
        Serial.print("Writen WORKING_TIME: ");
        Serial.println(working_time);
        EEPROM_write(EEP_WORKING_TIME, working_time);
        return COM_RX_OK;
      }
      else
      {
        Serial.println("ERROR FORMAT");
      }
    }
    else if (isEqual(command[1], "ot_time"))
    {
      if (check_working_time_format(command[2]))
      {
        int _sz = 0;
        for (int c : command[2])
        {
          if (c == 0)
            break;
          _sz++;
        }
        memcpy(ot_time, command[2], _sz + 1);
        Serial.print("Writen OT_TIME: ");
        Serial.println(ot_time);
        EEPROM_write(EEP_OT_TIME, ot_time);
        return COM_RX_OK;
      }
    }
    else if (isEqual(command[1], "ot"))
    {
      if (toUInt(command[2]) == -1)
        return COM_RX_NG;
      _u_integer = toUInt(command[2]);
      ot = _u_integer;
      EEPROM_write(EEP_OT, ot);
      Serial.print("Writen OT: ");
      Serial.println(ot);
      return COM_RX_OK;
    }
  }
  else if (isEqual(command[0], "@slave"))
  { // @slave @text 1 [adwa dlaos]
    for (int i = 1; i < CMD_SIZE; i++)
    {
      if (command[i][0] == 0)
        break;
      Serial2.print(command[i]);
      Serial2.print(' ');
    }
    Serial2.print('\r');
    rx_status = COM_RX_OK;
  }

  else if(isEqual(command[0], "@getslave")){
    slave_update(LCD_TARGET);
    slave_update(LCD_PLAN);
    slave_update(LCD_ACTUAL);
    rx_status = COM_RX_OK;
  }
  return rx_status;
}

void serialEvent2()
{
  char _inChar = 0;
  rx_index = 0;
  while (Serial2.available())
  {
    _inChar = (char)Serial2.read();
    // Serial.print(_inChar);
    rx[rx_index] = _inChar;
    if (rx_index <= MAX_RX_BUFFER)
      rx_index++;
    else
      rx_index = 0;
    if (_inChar == ASCII_CR)
    {
      if (read_all_serial)
        rx_status = rx_command(rx);
      if (rx_status == COM_RX_OK)
      {
        rx[0] = '*';
        Serial.println(rx);
      }
      else if (rx_status == COM_RX_NULL)
      {
      }
      else
        Serial.println("*CMD_ERROR2");
      rx_index = 0;
      rx_clear();
    }
  }
}
void serialEvent()
{
  char _inChar = 0;
  rx_index = 0;
  while (Serial.available())
  {
    _inChar = (char)Serial.read();
    // Serial.print(_inChar);
    rx[rx_index] = _inChar;
    if (rx_index <= MAX_RX_BUFFER)
      rx_index++;
    else
      rx_index = 0;
    if (_inChar == ASCII_CR)
    {
      rx_status = rx_command(rx);
      if (rx_status == COM_RX_OK)
      {
        rx[0] = '*';
        Serial.println(rx);
      }
      else if (rx_status == COM_RX_NULL)
      {
      }
      else
        Serial.println("*CMD_ERROR");
      rx_index = 0;
      rx_clear();
    }
  }
}

void rx_clear(void)
{
  for (int i = 0; i < MAX_RX_BUFFER; i++)
    rx[i] = 0;
}
void timer_reset()
{

  set_timer(T01, 1000);

  timer_count_down = timer_set;

  times_up = false;
  lcd_update(LCD_TIME);
}
void lcd_update(byte lcd_data)
{
  String str_out = "";
  if (timer_count_down >= 10000 || timer_count_down < 0)
    return;
  if (plan >= 10000 || plan < 0)
    return;
  if (actual_counter >= 10000 || actual_counter < 0)
    return;
  if (target >= 10000 || target < 0)
    return;
  if (lcd_data == LCD_TIME)
  {

    lcd.setCursor(14, 0);
    for (int _ = 0; _ < LCD_DATA_LEN - (int)(String(timer_count_down).length()); _++)
    {
      str_out += " ";
    }

    str_out += String(timer_count_down) + "s";
  }
  else if (lcd_data == LCD_PLAN)
  {
    lcd.setCursor(15, 1);
    for (int _ = 0; _ < LCD_DATA_LEN - (int)String(plan).length(); _++)
      str_out += " ";
    str_out += String(plan);
  }
  else if (lcd_data == LCD_TARGET)
  {
    lcd.setCursor(15, 2);
    for (int _ = 0; _ < LCD_DATA_LEN - (int)String(target).length(); _++)
      str_out += " ";
    str_out += String(target);
  }
  else if (lcd_data == LCD_ACTUAL)
  {
    lcd.setCursor(15, 3);
    for (int _ = 0; _ < LCD_DATA_LEN - (int)String(actual_counter).length(); _++)
      str_out += " ";
    str_out += String(actual_counter);
  }

  lcd.print(str_out);
}
void update_data(String _key, String _value)
{
  if (_key == "actual")
  {
    return;
    int val = _value.toInt();
    if (val != actual_counter)
    {
      actual_counter = val;
      EEPROM_write(EEP_ACTUAL_COUNTER, actual_counter);
      lcd_update(LCD_ACTUAL);
      slave_update(LCD_ACTUAL);
    }
  }
  else if (_key == "enable_ot")
  {
    bool val = _value == String("true");
    if (val != ot)
    {
      ot = val;
      EEPROM_write(EEP_OT, ot);
    }
  }
  else if (_key == "id")
  {
    return;
    int val = _value.toInt();
    if (val != device_id)
    {
      device_id = val;
      EEPROM_write(EEP_DEVICE_ID, device_id);
    }
  }
  else if (_key == "plan")
  {
    int val = _value.toInt();
    if (val != plan)
    {
      plan = val;
      EEPROM_write(EEP_PLAN, plan);
      lcd_update(LCD_PLAN);
      slave_update(LCD_PLAN);
    }
  }
  else if (_key == "timer_set")
  {
    int val = _value.toInt();
    if (val != timer_set)
    {
      timer_set = val;
      EEPROM_write(EEP_TIMER, val);
      seq[7] = 0; // update target delay
      timer_reset();
    }
  }
  else if (_key == "target")
  {
    return; //illegal to change target
    int val = _value.toInt();
    if (val != target)
    {
      target = val;
      EEPROM_write(EEP_TARGET, target);
      lcd_update(LCD_TARGET);
      slave_update(LCD_TARGET);
      seq[7] = 0; // update target delay
    }
  }
  else if (_key == "working_time")
  {
    // Serial.print("get this data: ");
    // Serial.println(_value);
    String real_value = _value;
    if (_value[0] == '"')
    {
      real_value = _value.substring(1, _value.length() - 1);
    }

    if (isEqual(working_time, real_value))
      return;
    real_value.toCharArray(working_time, WORKING_TIME_LENGTH);
    EEPROM_write(EEP_WORKING_TIME, working_time);
  }
  else if (_key == "ot_time")
  {
    String real_value = _value;
    if (_value[0] == '"')
    {
      real_value = _value.substring(1, _value.length() - 1);
    }
    if (isEqual(ot_time, real_value))
      return;
    real_value.toCharArray(ot_time, WORKING_TIME_LENGTH);
    EEPROM_write(EEP_OT_TIME, ot_time);
  }
  else if (_key == "forceupdate")
  {

    bool val = _value == String("true");

    if (val != force_update)
    {

      force_update = val;
    }
  }
  else
  {
    PRINTF("Key: %s no found\n", _key);
  }
}

void streamCallback(FirebaseStream data)
{

  // Print out all information

  PRINTLN("Stream Data...");
  PRINTLN(data.streamPath());
  PRINTLN(data.dataPath());
  PRINTLN(data.dataType());

  if (data.dataTypeEnum() == fb_esp_rtdb_data_type_json)
  {
    FirebaseJson json = data.to<FirebaseJson>();
    size_t count = json.iteratorBegin();
    for (size_t i = 0; i < count; i++)
    {
      FirebaseJson::IteratorValue value = json.valueAt(i);
      String key = value.key;
      String val = value.value;
      update_data(key, val);
    }
    json.iteratorEnd();
  }
  else
  {
    String key = data.dataPath().substring(1, data.dataPath().length());
    String value = data.to<String>();
    update_data(key, value);
  }
}

// Global function that notifies when stream connection lost
// The library will resume the stream connection automatically
void streamTimeoutCallback(bool timeout)
{
  if (timeout)
    Serial.println("stream timeout, resuming...\n");
  if (!stream.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", stream.httpCode(), stream.errorReason().c_str());
}

template <typename T>
byte check_decimal_range(char temp_x1000, char temp_x100, char temp_x10, char temp_x1, int temp_min, int temp_max, T &temp_out_value)
{ // effective range from 0000 to 9999
  int temp_value;
  byte temp_count = 0;
  if (temp_x1000 >= '0' && temp_x1000 <= '9')
    temp_count++;
  if (temp_x100 >= '0' && temp_x100 <= '9')
    temp_count++;
  if (temp_x10 >= '0' && temp_x10 <= '9')
    temp_count++;
  if (temp_x1 >= '0' && temp_x1 <= '9')
    temp_count++;
  if (temp_count != 4)
    return NOT_NUMBER; // if any of the character not number then exit here

  if (temp_x1000 >= '0' || temp_x1000 <= '9')
  {
    if (temp_x100 >= '0' || temp_x100 <= '9')
    {
      if (temp_x10 >= '0' || temp_x10 <= '9')
      {
        if (temp_x1 >= '0' || temp_x1 <= '9')
        {
          temp_value = int(temp_x1000 - '0') * 1000 + int(temp_x100 - '0') * 100 + int(temp_x10 - '0') * 10 + int(temp_x1 - '0');
          if (temp_value >= temp_min && temp_value <= temp_max)
          {
            temp_out_value = temp_value;
            return VALUE_RANGE_OK; // if range between min and max then exit here
          }
        }
      }
    }
  }
  return VALUE_RANGE_OUT;
}

//== Timer Function ===============================================================
byte set_timer(byte temp_tim_num, unsigned long temp_tim_set)
{
  if (temp_tim_num < MAX_TIM_NUM)
  {
    time_data[temp_tim_num] = millis();
    timeset[temp_tim_num] = temp_tim_set;
    t_status[temp_tim_num] = TIMSET;
    return 1;
  }
  else if (temp_tim_num >= MAX_TIM_NUM && temp_tim_num < (MAX_TIM_NUM + US))
  {
    time_data[temp_tim_num] = micros();
    timeset[temp_tim_num] = temp_tim_set;
    t_status[temp_tim_num] = TIMSET;
    return 1;
  }
  else
  {
    return 0;
  }
}

byte timer_status(byte temp_tim_num)
{
  byte temp_return = TIMOUT;
  if (temp_tim_num < MAX_TIM_NUM)
  {
    if (t_status[temp_tim_num] == TIMSET)
    {
      if ((millis() - time_data[temp_tim_num]) >= timeset[temp_tim_num])
        t_status[temp_tim_num] = TIMOUT;
      temp_return = t_status[temp_tim_num];
    }
  }
  else if (temp_tim_num >= MAX_TIM_NUM && temp_tim_num < (MAX_TIM_NUM + US))
  {
    if (t_status[temp_tim_num] == TIMSET)
    {
      if ((micros() - time_data[temp_tim_num]) >= timeset[temp_tim_num])
        t_status[temp_tim_num] = TIMOUT;
      temp_return = t_status[temp_tim_num];
    }
  }
  return temp_return;
}

//====== Initialize IO =====================================================
void init_io(void)
{
  pinMode(RUN_LED_pin, OUTPUT);
  digitalWrite(RUN_LED_pin, LOW); // Output, nomal LOW
  pinMode(R_LED_pin, OUTPUT);
  digitalWrite(R_LED_pin, LOW); // Output, nomal LOW
  pinMode(G_LED_pin, OUTPUT);
  digitalWrite(G_LED_pin, LOW); // Output, nomal LOW
  pinMode(Y_LED_pin, OUTPUT);
  digitalWrite(Y_LED_pin, LOW); // Output, nomal LOW
  pinMode(NEXT_SW_pin, INPUT_PULLUP);
  pinMode(MUTE_SW_pin, INPUT_PULLUP);
  pinMode(BUZZER_pin, OUTPUT);
  digitalWrite(BUZZER_pin, LOW);
}
void scan_i2c()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning..."); /*ESP32 starts scanning available I2C devices*/
  nDevices = 0;
  for (address = 1; address < 127; address++)
  { /*for loop to check number of devices on 127 address*/
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {                                                 /*if I2C device found*/
      Serial.print("I2C device found at address 0x"); /*print this line if I2C device found*/
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX); /*prints the HEX value of I2C address*/
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n"); /*If no I2C device attached print this message*/
  }
  else
  {
    Serial.println("done\n");
  }
}