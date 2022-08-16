#include <SPI.h>
#include <SD.h>
#include <iarduino_Position_BMX055.h>
#include <iarduino_Pressure_BMP.h>
#include <iarduino_I2C_SHT.h>
#include <iarduino_OLED_txt.h>
#include <iarduino_GPS_NMEA.h>
#include <iarduino_GPS_ATGM336.h>
#include <TroykaCurrent.h>
#include "menu_trolley.h"

// TIMER_CNT defined in milliseconds
#define AUTOSTOP true
#define TIMEOUT 4000
#define DEBUG_LOG

#define IND_PIN 2
#define AMP_PIN A0
#define VOLT_PIN A1
#define NEXT_PIN 5
#define SELECT_PIN 8

#define MAS_SIZE 60
const int PIN_CHIP_SELECT = 10; // 4 - on ethernet
const int wheel_size = 25;

#define VREF 5.102
#define V_COEF 1.045
#define DIV_R1 19680
#define DIV_R2 1176

#define DELAY_RD 65
//#define A4 S0
//#define A5 S1

// i2c: sda - a4, scl - a5
// spi: mosi - 11, miso - 12, sck - 13, ss - 10
// uart: d0, d1

// S1 S0:
//  0  0 - Wi-Fi
//  0  1 - Bluetooth
//  1  0 - GPS

volatile unsigned long counter;
volatile unsigned long counter_inc_time;
volatile unsigned long timer;
volatile unsigned long timer_begin;
volatile unsigned long timer_end;
volatile unsigned long timer_delta;
volatile unsigned long timer_last;
volatile unsigned long timeout_delta;
volatile unsigned long rpm;

float voltage;
float current;
float gps_speed;
double timer_d;
int i;
bool start;
bool start_writing;
bool file_was_written;
uint8_t sat_mas[30][7];
String current_time;
String year_str, month_str, day_str, hours_str, minutes_str;

bool timeout;

iarduino_Position_BMX055 sensor_a(BMA);
iarduino_Pressure_BMP sensor_p;
iarduino_I2C_SHT sht;
iarduino_GPS_NMEA gps;
iarduino_GPS_ATGM336 SettingsGPS;
iarduino_OLED_txt oled(0x3C);
ACS712 dataI(AMP_PIN);
File myFile;

struct sample
{
  unsigned long timer;
  int delta;
  int counter;
  float altitude;
  float temperature;
  float hum;
  float axis_x;
  float accel_x;
  float voltage;
  float current;
  float gps_speed;
};

extern uint8_t SmallFontRus[];
// sample sample_mas[MAS_SIZE];

void setup()
{
#ifdef DEBUG_LOG
  Serial.begin(9600);
  while (!Serial)
    ;
#endif

  oled.begin();
  oled.setFont(SmallFontRus);
  // print_check();
  // Serial.println("Initializing SD card...");
  pinMode(PIN_CHIP_SELECT, OUTPUT); // for SD card
  pinMode(IND_PIN, INPUT);
  pinMode(A4, OUTPUT); // S0
  pinMode(A5, OUTPUT); // S1

  counter = 0;
  timer = 0;
  timer_end = 0;
  timer_last = 0;
  voltage = 0;
  current = 0;
  i = 0;
  // btnTimerNext = 0;
  // btnTimerSelect = 0;
  cnt_main = 0;
  cnt_sample_mode = 0;
  cnt_freq_sample = 0;
  cnt_count_rot = 0;
  cnt_gps_en = 0;
  cnt_print_disp = 0;
  cnt_high_spd = 0;

  cnt_freq_sample = 0;
  h_spd_cnt = 10;

  FREQ = HZ1;
  ROTATION_BOUND = 1;
  GPS_SPEED = false;
  GPS_TIME = false;
  PRINT_PARAMS_EN = false;
  h_spd_mode = false;
  count_by_time = false;

  menu_active = true;
  main_menu_select = true;
  sample_mode_menu_select = false;
  freq_sample_menu_select = false;
  count_rot_select = false;
  gps_en_select = false;
  print_disp_select = false;
  high_spd_select = false;
  flagNext = true;
  flagSelect = true;
  file_was_written = false;
  start_writing = false;
  timeout = false;
  pinMode(2, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  oled.clrScr();
  if (!SD.begin(PIN_CHIP_SELECT))
  {
    oled.print("SD карта не обнаружена", 0, 0);
    while (true)
      ;
  }
  else
    oled.print("SD карта найдена", 0, 0);

  delay(1500);

  print_main_menu(oled, cnt_main);

  // Serial.println("Card OK");
  // gps_setup();

  // if(SD.exists(current_time))//"arduino.csv"
  //   SD.remove(current_time);//

  // current_time = "2021_09_07_20_20.csv";

  // prepare_to_start();
}

void loop()
{
  // Serial.println("1111");
  if (menu_active)
  {
    menu_actions(oled, start, NEXT_PIN, SELECT_PIN);
  }
  else
  {
    // if(start_writing)
    timeout = ((millis() - timer_end) > TIMEOUT) ? true : false;
    if (start_writing & timeout & !file_was_written)
    {
      myFile.close();
      file_was_written = true;
      print_finish(oled, file_was_written);
      start_writing = false;
      // Serial.println("2222");
    }
    else if (~start_writing & (counter > 3) & ~file_was_written)
    {
      // write first string in *.csv file
      start_writing = write_header();
      counter = 0;
      // Serial.println("Module was started");
      if (start_writing)
        print_started(oled);
      else
      {
        oled.clrScr();
        oled.print("Ошибка записи", 0, 0);
      }
      timer_end = millis();
      timeout = false;
      // Serial.println("3333");
    }
    else if (count_by_time & start_writing) // count by time
    {
      if (millis() - timer >= FREQ) // F
      {
        timer_begin = millis();
        read_sensors();
        write_sample();
        print_params();
        timer = millis();
        i++;
        counter = 0;
        // timer_end = millis();
      }
    }
    else if (!count_by_time & (counter >= ROTATION_BOUND) & start_writing) // count by rotations
    {
      timer_begin = millis();
      print_params();
      write_sample();
      i++;
      counter = 0;
      timer_end = millis();
    }
  }
  // Serial.println("end loop");
}

void read_sensors()
{
  timer_begin = timer; // millis();
  sensor_a.read(BMA_DEG);
  sensor_p.read(1);
  current = dataI.readCurrentDC();
  voltage = analogRead(VOLT_PIN) * VREF * V_COEF * ((DIV_R1 + DIV_R2) / DIV_R2) / 1024;

  if (GPS_SPEED)
  {
    gps.read();
    gps_speed = gps.speed;
  }
  /*
  if(gps.errCrs){
    //Serial.println("Velosity error");
    //delay(2000);
    //return;
  }
  */

  return;
}

bool write_header()
{
  // Serial.println("write header");
  if (myFile)
  {
    if (count_by_time)
    {
      myFile.println("count by time");
      myFile.print("every ");
      myFile.print(FREQ);
      myFile.println(" ms");
    }
    else
    {
      myFile.println("count by rotations");
      myFile.print("every ");
      myFile.print(ROTATION_BOUND);
      myFile.println(" rotation(s)");
    }
    if (GPS_TIME)
      myFile.println("GPS time is on");
    else
      myFile.println("GPS time is off");
    if (GPS_SPEED)
      myFile.println("GPS speed is on");
    else
      myFile.println("GPS speed is off");

    if (h_spd_mode)
    {
      myFile.println("High speed mode is on,");
      myFile.print("writing all data every ");
      myFile.print(h_spd_cnt);
      myFile.println(" rotations");
    }
    else
      myFile.println("High speed mode is off");

    myFile.print("timer");
    myFile.print(";");
    myFile.print("speed");
    myFile.print(";");
    myFile.print("counter");
    myFile.print(";");
    myFile.print("altitude");
    myFile.print(";");
    // pressure
    myFile.print("temp");
    myFile.print(";");
    myFile.print("humidity"); // sht.getTem()
    myFile.print(";");
    myFile.print("axis_x");
    myFile.print(";");
    myFile.print("accel_x");
    myFile.print(";");
    myFile.print("voltage");
    myFile.print(";");
    myFile.print("current");
    myFile.print(";");
    myFile.print("timer delta");
    myFile.print(";");
    myFile.println("rpm");
    return true;
  }
  else
    return false;
}

void write_sample()
{
  if (!h_spd_mode | (h_spd_mode & ((i % h_spd_cnt) == 0)))
  {
    read_sensors();
    myFile.print(timer_begin);
    myFile.print(";");
    myFile.print(gps_speed);
    myFile.print(";");
    myFile.print(counter);
    myFile.print(";");
    myFile.print(sensor_p.altitude);
    myFile.print(";");
    // pressure
    myFile.print(sensor_p.temperature);
    myFile.print(";");
    myFile.print(sht.getHum()); // sht.getTem()
    myFile.print(";");
    myFile.print(sensor_a.axisY); // axisX
    myFile.print(";");
    sensor_a.read(BMA_G);         // BMA_M_S
    myFile.print(sensor_a.axisY); // axisX
    myFile.print(";");
    myFile.print(voltage);
    myFile.print(";");
    myFile.print(current);
    myFile.print(";");
    timer = millis();
    myFile.print(timer - timer_last);
    myFile.print(";");
    // rpm = (counter * 60000) / (timer - timer_last);
    // myFile.println(rpm);
    myFile.println((counter * 60000) / (timer - timer_last));
  }
  else if (h_spd_mode) // & ((i % h_spd_cnt) == 0))
  {
    myFile.print(timer_begin);
    myFile.print("; ;");
    myFile.print(counter);
    myFile.print("; ; ; ; ; ; ; ;");
    timer = millis();
    myFile.print(timer - timer_last);
    myFile.print(";");
    // rpm = (counter * 60000) / (timer - timer_last);
    // myFile.println(rpm);
    myFile.println((counter * 60000) / (timer - timer_last));
  }
  timer_last = timer;
}

void gps_setup()
{
  byte sat_cnt = 0;
  bool datatime_err = true;
  digitalWrite(A5, HIGH); // select GPS
  digitalWrite(A4, LOW);

  oled.clrScr();
  oled.print("Ожидание сигнала", 0, 0);
  oled.print("от спутников", 0, 1);

  SettingsGPS.begin(Serial);
  gps.begin(Serial);
  //   Настраиваем работу модуля:
  SettingsGPS.baudrate(9600);
  SettingsGPS.system(GPS_GP, GPS_GL);
  SettingsGPS.composition(NMEA_GSA, NMEA_GSV, NMEA_ZDA, NMEA_VTG);
  // SettingsGPS.composition(NMEA_ZDA, NMEA_VTG);//speed
  // SettingsGPS.updaterate(10);
  SettingsGPS.model(GPS_VEHICLE);
  // SettingsGPS.updaterate(1);//update 1 time a second
  SettingsGPS.updaterate(10);
  while (sat_cnt < 4) // counting gps sattelites
  {
    gps.read(sat_mas, true);
    sat_cnt = 0;
    for (int i = 0; i < 30; i++)
    {
      if (sat_mas[i][0]) // sattelite id != 0
        sat_cnt++;
    }
    oled.clrScr();
    oled.print("Спутн.: ", 0, 0);
    oled.print(sat_cnt, 40, 0);
  }
  SettingsGPS.updaterate(10); // update 1 time a second

  while (datatime_err)
  {
    gps.read(); // reading time and date
    datatime_err = gps.errTim | gps.errDat;
  }

  current_time = "";
  year_str = String("20") + gps.year;

  if (gps.month < 10)
    month_str = String("0") + gps.month;
  else
    month_str = gps.month;

  if (gps.day < 10)
    day_str = String("0") + gps.day;
  else
    day_str = gps.day;

  if (gps.Hours < 10)
    hours_str = String("0") + gps.Hours;
  else
    hours_str = gps.Hours;

  if (gps.minutes < 10)
    minutes_str = String("0") + gps.minutes;
  else
    minutes_str = gps.minutes;

  // current_time = year_str + String("_") + month_str + String("_") + day_str
  //        + String("_") + hours_str + String("_") + minutes_str;

  current_time = month_str + day_str + hours_str + minutes_str + String(".csv");

  oled.clrScr();
  oled.print("GPS готов", 0, 0);
  oled.print(current_time, 0, 1);

  oled.print("Месяц: ", 0, 3);
  oled.print(gps.month, 50, 3);
  oled.print("День : ", 0, 4);
  oled.print(gps.day, 50, 4);
  oled.print("Час  : ", 0, 5);
  oled.print(gps.Hours, 50, 5);
  oled.print("Мин  : ", 0, 6);
  oled.print(gps.minutes, 50, 6);

  // settings for measuring speed
  // SettingsGPS.composition(NMEA_ZDA, NMEA_VTG);
  // SettingsGPS.updaterate(10);//update 10 time a second
  /*
  //gps debug
  //it's work fine!
  delay(10000);

  SettingsGPS.reset(GPS_HOT_START);
  delay(1000);
  SettingsGPS.begin(Serial);
  gps.begin(Serial);
  SettingsGPS.baudrate(9600);
  SettingsGPS.system(GPS_GP, GPS_GL);
  SettingsGPS.composition(NMEA_VTG);
  SettingsGPS.model(GPS_VEHICLE);
  SettingsGPS.updaterate(1);

  while(1){
    gps.read();
    if(gps.errCrs){                               //  Если данные не прочитаны (gps.errCrs=1) или скорость и курс недостоверны (gps.errCrs=2), то ...
       //Serial.println("Скорость недостоверна");    //  Выводим сообщение об ошибке.
       oled.clrScr();
       oled.print("Ошибка скорости", 0, 0);
       delay(2000);
    }
    oled.clrScr();
    oled.print("Скор.:", 0, 0);
    oled.print(gps.speed, 50, 0);
    oled.print("Курс : ", 0, 1);
    oled.print(gps.course, 50, 1);
    delay(1000);
  }
  */
  return;
}

void prepare_to_start()
{
  char filename[current_time.length() + 1];
  oled.clrScr();
  oled.print("Проверка датчиков", 0, 0);
  delay(1500);
  sensor_a.begin();
  sensor_a.setFastOffset();
  sensor_p.begin();
  sensor_p.measurement(4); // precision 0-4, max - 4
  sht.begin();

  if (GPS_TIME)
  {
    gps_setup();
    current_time.toCharArray(filename, current_time.length() + 1);
    myFile = SD.open(filename, FILE_WRITE);
  }
  else
    myFile = SD.open("tstrde.csv", FILE_WRITE);

  menu_active = false;

  // if(!myFile)
  // print error

  attachInterrupt(digitalPinToInterrupt(IND_PIN), inc_cnt, RISING);
  print_ready(oled);
  return;
}

void print_params()
{
  if (PRINT_PARAMS_EN)
  {
    oled.clrScr();
    // oled.setCoding(TXT_UTF8);
    oled.print("Время:", 0, 0);
    oled.print(timer / 1000, 40, 0);
    oled.print("Темп.:", 0, 1);
    oled.print(sensor_p.temperature, 40, 1);
    oled.print("Обор.:", 0, 2);
    oled.print(counter * 60, 40, 2);
    oled.print("Таг.: ", 0, 3);
    oled.print(sensor_a.axisY, 40, 3); // axisX
    oled.print("Выс.: ", 0, 4);
    oled.print(sensor_p.altitude, 40, 4);
    oled.print("Напр.:", 0, 5);
    oled.print(voltage, 40, 5);
    oled.print("Ток.:", 0, 6);
    oled.print(dataI.readCurrentAC(), 40, 6);
    oled.print("Скор.:", 0, 7);
    oled.print(gps_speed, 40, 7);
    return;
  }
}

void inc_cnt()
{
  counter++;
  timer_end = millis();

  return;
}
