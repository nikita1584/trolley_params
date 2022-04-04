#include <SPI.h>
#include <SD.h>
#include <iarduino_Position_BMX055.h>
#include <iarduino_Pressure_BMP.h>
#include <iarduino_I2C_SHT.h> 
#include <iarduino_OLED_txt.h>
#include <iarduino_GPS_NMEA.h>
#include <iarduino_GPS_ATGM336.h>
#include <TroykaCurrent.h>

//TIMER_CNT defined in milliseconds 
#define AUTOSTOP true
#define TIMEOUT 4000
#define DEBUG_LOG

#define IND_PIN   2
#define AMP_PIN   A0
#define VOLT_PIN  A1
#define NEXT_PIN   5
#define SELECT_PIN 8

#define MAS_SIZE  60
const int PIN_CHIP_SELECT = 10;//4 - on ethernet
const int wheel_size = 25;

#define VREF 5.102
#define V_COEF 1.045
#define DIV_R1 19680
#define DIV_R2 1176

#define DELAY_RD 65
#define HZ1 930
#define HZ2 430
#define HZ4 180
#define HZ10 30
//#define A4 S0
//#define A5 S1

//i2c: sda - a4, scl - a5
//spi: mosi - 11, miso - 12, sck - 13, ss - 10
//uart: d0, d1

//S1 S0:
// 0  0 - Wi-Fi
// 0  1 - Bluetooth
// 1  0 - GPS

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
bool file_was_written;
bool start_writing;
uint8_t sat_mas[30][7];
byte h_spd_cnt;
String current_time;
String year_str, month_str, day_str, hours_str, minutes_str;

bool main_menu_select;
bool sample_mode_menu_select;
bool freq_sample_menu_select;
bool count_rot_select;
bool gps_en_select;
bool print_disp_select;
bool count_by_time;
bool menu_active;
bool btnStateNext;
bool btnStateSelect;

int FREQ;
int ROTATION_BOUND;
bool GPS_SPEED;
bool GPS_TIME;
bool PRINT_PARAMS_EN;
bool h_spd_mode;

byte cnt_main;
byte cnt_sample_mode;
byte cnt_freq_sample;
byte cnt_count_rot;
byte cnt_gps_en;
byte cnt_print_disp;
bool start;
bool timeout;

bool flagNext = true;
uint32_t btnTimerNext;
bool flagSelect = true;
uint32_t btnTimerSelect;

char *main_menu[] = {
  "Режим отметок ",
  "Частота записи",
  "Число оборотов",
  "GPS активен   ",
  "Выв. на дисп. ",
  "Старт!        "
};

char *sample_mode_menu[] = {
  "Отм. по врем. ",
  "Отм. по расст.",
  "Выход         "
};

char *freq_sample_menu[] = {
  "1 раз в сек   ",
  "2 раза в сек  ",
  "4 раза в сек  ",
  "10 раз в сек  ",
  "Выход         "
};

char *count_rot_menu[] = {
  "1  оборотов   ",
  "2  оборотов   ",
  "4  оборотов   ",
  "8  оборотов   ",
  "Выход         "
};

char *gps_en_menu[] = {
  "GPS скор. вкл.",
  "GPS скор. выкл",
  "Выход"
};

char *print_disp_menu[] = {
  "Вкл. печ. дисп",
  "Выкл.печ. дисп",
  "Выход"
};

char *high_spd_menu[] = {
  "Вкл. печ. дисп",
  "Выкл.печ. дисп",
  "Выход"
};

iarduino_Position_BMX055 sensor_a(BMA);
iarduino_Pressure_BMP sensor_p;
iarduino_I2C_SHT sht;
iarduino_OLED_txt myOLED(0x3C);
iarduino_GPS_NMEA gps;
iarduino_GPS_ATGM336 SettingsGPS;
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
//sample sample_mas[MAS_SIZE];

void setup() {
  #ifdef DEBUG_LOG
  Serial.begin(9600);
  while(!Serial);
  #endif
  
  myOLED.begin();
  myOLED.setFont(SmallFontRus);
  //print_check();
  //Serial.println("Initializing SD card...");
  pinMode(PIN_CHIP_SELECT, OUTPUT);//for SD card
  pinMode(IND_PIN, INPUT);
  pinMode(A4, OUTPUT); //S0
  pinMode(A5, OUTPUT); //S1

  counter = 0;
  timer = 0;
  timer_end = 0;
  timer_last = 0;
  voltage = 0;
  current = 0;
  i = 0;
  btnTimerNext = 0;
  btnTimerSelect = 0;
  cnt_main = 0;
  cnt_sample_mode = 0;
  cnt_freq_sample = 0;
  h_spd_cnt = 10;
  
  FREQ = HZ1;
  ROTATION_BOUND = 20;
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
  flagNext = true;
  flagSelect = true;
  file_was_written = false;
  start_writing = false;
  timeout = false;
  pinMode(2, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  myOLED.clrScr();
  if(!SD.begin(PIN_CHIP_SELECT)) {
    myOLED.print("SD карта не обнаружена", 0, 0);
    while(true);
  }
  else
    myOLED.print("SD карта найдена", 0, 0);

  delay(1500);

  print_main_menu(cnt_main);
    
  //Serial.println("Card OK");
  //gps_setup();

  //if(SD.exists(current_time))//"arduino.csv"
  //  SD.remove(current_time);//

  //current_time = "2021_09_07_20_20.csv";
  
  prepare_to_start();
}

void loop() {
  //Serial.println("1111");
  if(menu_active)
  {
    btnStateNext = !digitalRead(NEXT_PIN);
    if(btnStateNext && !flagNext && millis() - btnTimerNext > 100)
    {
      flagNext = true;
      btnTimerNext = millis();
      push_next();
    }
    if(!btnStateNext && flagNext && millis() - btnTimerNext > 100)
    {
      flagNext = false;
      btnTimerNext = millis();
    }
  
    btnStateSelect = !digitalRead(SELECT_PIN);
    if (btnStateSelect && !flagSelect && millis() - btnTimerSelect > 100)
    {
      flagSelect = true;
      btnTimerSelect = millis();
      push_select();
    }
    
    if(!btnStateSelect && flagSelect && millis() - btnTimerSelect > 100)
    {
      flagSelect = false;
      btnTimerSelect = millis();
    }
  }
  else
  {
    //if(start_writing)
    timeout = ((millis() - timer_end) > TIMEOUT)? true : false;
    /*
    Serial.print("millis() = ");
    Serial.println(millis());
    Serial.print("timer_end = ");
    Serial.println(timer_end);

    timer_d = millis() / 1000;
    Serial.print("timer_d = ");
    Serial.println(timer_d);
    */
    /*
    Serial.print("timeout = ");
    Serial.println(timeout);
    Serial.print("millis() = ");
    Serial.println(millis());
    Serial.print("timer_end = ");
    Serial.println(timer_end);
    Serial.print("start_writing = ");
    Serial.println(start_writing);
    Serial.print("file_was_written = ");
    Serial.println(file_was_written);
    */
    if(start_writing & timeout & !file_was_written)
    { 
      Serial.print("millis() = ");
      Serial.println(millis());
      Serial.print("timer_end = ");
      Serial.println(timer_end);   
      myFile.close();
      file_was_written = true;
      print_finish();
      start_writing = false;
      //Serial.println("2222");
    }
    else if(~start_writing & (counter > 3) & ~file_was_written)
    {
      //write first string in *.csv file
      start_writing = write_header();
      counter = 0;
      //Serial.println("Module was started");
      if(start_writing)
        print_started();
      else
      {
        myOLED.clrScr();
        myOLED.print("Ошибка записи", 0, 0);
      }
      timer_end = millis();
      timeout = false;
      //Serial.println("3333");
    }
    else if(count_by_time & start_writing)//count by time
    {
      //Serial.println("count by time");
      if(millis() - timer >= FREQ)//F
      {
        //Serial.println("count by time");
        timer_begin = millis();
        read_sensors();
        mul_cnt();
        //write_to_mas(i);
        write_sample();
        print_params();
        timer = millis();
        //sample_mas[i].delta = timer - timer_begin;
        i++;
        counter = 0;
        //timer_end = millis();
      }
    }
    else if(!count_by_time & (counter >= ROTATION_BOUND) & start_writing)//count by rotations
    {
      //Serial.println("count by rotations");
      timer_begin = millis();
      
      //write_to_mas(i);
      //if((i % 10) == 0){
        //read_sensors();
        //mul_cnt();
        
        print_params();
      //}
      write_sample();
      //timer = millis();
      //sample_mas[i].delta = timer - timer_begin;
      i++;
      counter = 0;
      timer_end = millis();
    }
  }
  //Serial.println("end loop");
}

void read_sensors(){
  //Serial.println("4444");
  timer_begin = timer;//millis();
  sensor_a.read(BMA_DEG);
  sensor_p.read(1);
  current = dataI.readCurrentAC();
  voltage = analogRead(VOLT_PIN) * VREF * V_COEF * ((DIV_R1 + DIV_R2) / DIV_R2) / 1024;

  if(GPS_SPEED)
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
  
  ////Serial.print("Скорость: ");
  ////Serial.print(gps.speed);
  ////Serial.print("км/ч. ");
  
  return;
}


/*
bool write_to_file(){
  if(myFile)
  {
    myFile.print("timer");
    myFile.print(";");
    myFile.print("timer delta");
    myFile.print(";");
    myFile.print("counter");
    myFile.print(";");
    myFile.print("altitude");
    myFile.print(";");
    //pressure
    myFile.print("temp");
    myFile.print(";");
    myFile.print("humidity");//sht.getTem()
    myFile.print(";");
    myFile.print("axis_x");
    myFile.print(";");
    myFile.print("accel_x");
    myFile.print(";");
    myFile.print("voltage");
    myFile.print(";");
    myFile.print("current");
    myFile.print(";");
    myFile.println("speed");
    for(i = 0; i < MAS_SIZE; i++)
    {
      myFile.print(sample_mas[i].timer);
      myFile.print(";");
      myFile.print(sample_mas[i].delta);
      myFile.print(";");
      myFile.print(sample_mas[i].counter);
      myFile.print(";");
      myFile.print(sample_mas[i].altitude);
      myFile.print(";");
      //pressure
      myFile.print(sample_mas[i].temperature);
      myFile.print(";");
      myFile.print(sample_mas[i].hum);//sht.getTem()
      myFile.print(";");
      myFile.print(sample_mas[i].axis_x);
      myFile.print(";");
      myFile.print(sample_mas[i].accel_x);
      myFile.print(";");
      myFile.print(sample_mas[i].voltage);
      myFile.print(";");
      myFile.print(sample_mas[i].current);
      myFile.print(";");
      myFile.println(sample_mas[i].speed);
    }
    myFile.close();
    //Serial.println("File arduino.csv was written");
    return true;
  }
  else
  {
    //Serial.println("Error opening file arduino.csv");
    return false;
  }
}
*/
bool write_header()
{
  //Serial.println("write header");
  if(myFile)
  {
    if(count_by_time)
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
    if(GPS_TIME)
      myFile.println("GPS time is on");
    else
      myFile.println("GPS time is off");
    if(GPS_SPEED)
      myFile.println("GPS speed is on");
    else
      myFile.println("GPS speed is off");

    if(h_spd_mode)
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
    //pressure
    myFile.print("temp");
    myFile.print(";");
    myFile.print("humidity");//sht.getTem()
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
  //Serial.println("5555");
  /*
    read_sensors();
    myFile.print(timer_begin);
    myFile.print(";");
    myFile.print(gps_speed);
    myFile.print(";");
    myFile.print(counter);
    myFile.print(";");
    myFile.print(sensor_p.altitude);
    myFile.print(";");
    //pressure
    myFile.print(sensor_p.temperature);
    myFile.print(";");
    myFile.print(sht.getHum());//sht.getTem()
    myFile.print(";");
    myFile.print(sensor_a.axisY);//axisX
    myFile.print(";");
    sensor_a.read(BMA_G);//BMA_M_S
    myFile.print(sensor_a.axisY);//axisX
    myFile.print(";");
    myFile.print(voltage);
    myFile.print(";");
    myFile.print(current);
    myFile.print(";");
    timer = millis();
    //timer_delta = timer - timer_last;
    myFile.println(timer - timer_last);
  */
    
    //myFile.print(";");
    
    //myFile.println(rpm);
    //timer_last = timer;
    
  
  if(!h_spd_mode | (h_spd_mode & ((i % h_spd_cnt) == 0)))
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
    //pressure
    myFile.print(sensor_p.temperature);
    myFile.print(";");
    myFile.print(sht.getHum());//sht.getTem()
    myFile.print(";");
    myFile.print(sensor_a.axisY);//axisX
    myFile.print(";");
    sensor_a.read(BMA_G);//BMA_M_S
    myFile.print(sensor_a.axisY);//axisX
    myFile.print(";");
    myFile.print(voltage);
    myFile.print(";");
    myFile.print(current);
    myFile.print(";");
    timer = millis();
    myFile.print(timer - timer_last);
    myFile.print(";");
    //rpm = (counter * 60000) / (timer - timer_last);
    //myFile.println(rpm);
    myFile.println((counter * 60000) / (timer - timer_last));
  }
  else if(h_spd_mode)// & ((i % h_spd_cnt) == 0))
  {
    myFile.print(timer_begin);
    myFile.print("; ;");
    myFile.print(counter);
    myFile.print("; ; ; ; ; ; ; ;");
    timer = millis();
    myFile.print(timer - timer_last);
    myFile.print(";");
    //rpm = (counter * 60000) / (timer - timer_last);
    //myFile.println(rpm);
    myFile.println((counter * 60000) / (timer - timer_last));
  }
  timer_last = timer;
}
/*
void write_to_mas(int j)
{
  sample_mas[j].timer = timer;
  sample_mas[j].counter = counter * 60;
  sample_mas[j].altitude = sensor_p.altitude;
  sample_mas[j].temperature = sensor_p.temperature;
  sample_mas[j].hum = sht.getHum();
  sample_mas[j].axis_x = sensor_a.axisX;
  sensor_a.read(BMA_G);//BMA_M_S
  sample_mas[j].accel_x = sensor_a.axisX;
  sample_mas[j].voltage = voltage;
  sample_mas[j].current = current;
  sample_mas[j].speed = speed;
}
*/
void print_params(){
  //Serial.println("print params");
  if(PRINT_PARAMS_EN){
    myOLED.clrScr();
    //myOLED.setCoding(TXT_UTF8);
    myOLED.print("Время:", 0, 0);
    myOLED.print(timer/1000, 40, 0);
    myOLED.print("Темп.:", 0, 1);
    myOLED.print(sensor_p.temperature, 40, 1);
    myOLED.print("Обор.:", 0, 2);
    myOLED.print(counter*60, 40, 2);
    myOLED.print("Таг.: ", 0, 3);
    myOLED.print(sensor_a.axisY, 40, 3);//axisX
    myOLED.print("Выс.: ", 0, 4);
    myOLED.print(sensor_p.altitude, 40, 4);
    myOLED.print("Напр.:", 0, 5);
    myOLED.print(voltage, 40, 5);
    myOLED.print("Ток.:", 0, 6);
    myOLED.print(dataI.readCurrentAC(), 40, 6);
    myOLED.print("Скор.:", 0, 7);
    myOLED.print(gps_speed, 40, 7);
    return;
  }
}

void print_finish(){
  myOLED.clrScr();
  if(file_was_written){
    myOLED.print("Файл записан на", 0, 0);
    myOLED.print("SD успешно", 0, 1);
  }
  else
    myOLED.print("Ошибка записи", 0, 0);
  return;
}

void print_ready(){
  myOLED.clrScr();
  myOLED.print("Готов к запуску", 0, 0);
  return;
}

void print_started(){
  myOLED.clrScr();
  //myOLED.setCoding(TXT_UTF8);
  myOLED.print("Параметры пишутся", 0, 0);
  return;
}

void print_check(){
  myOLED.clrScr();
  //myOLED.setCoding(TXT_UTF8);
  myOLED.print("Проверка датчиков", 0, 0);
  return;
}

void gps_setup(){
  byte sat_cnt = 0;
  bool datatime_err = true;
  digitalWrite(A5, HIGH);//select GPS
  digitalWrite(A4, LOW);
  
  myOLED.clrScr();
  myOLED.print("Ожидание сигнала", 0, 0);
  myOLED.print("от спутников", 0, 1);
  
  SettingsGPS.begin(Serial);
  gps.begin(Serial);
//   Настраиваем работу модуля:
  SettingsGPS.baudrate(9600);
  SettingsGPS.system(GPS_GP, GPS_GL);
  SettingsGPS.composition(NMEA_GSA, NMEA_GSV, NMEA_ZDA, NMEA_VTG);
  //SettingsGPS.composition(NMEA_ZDA, NMEA_VTG);//speed
  //SettingsGPS.updaterate(10);
  SettingsGPS.model(GPS_VEHICLE);
  //SettingsGPS.updaterate(1);//update 1 time a second
  SettingsGPS.updaterate(10);
  while(sat_cnt < 4)//counting gps sattelites
  {
    gps.read(sat_mas, true);
    sat_cnt = 0;
    for(int i = 0; i < 30; i++){
      if(sat_mas[i][0])//sattelite id != 0
        sat_cnt++;
    }
    myOLED.clrScr();
    myOLED.print("Спутн.: ", 0, 0);
    myOLED.print(sat_cnt, 40, 0);
  }
  SettingsGPS.updaterate(10);//update 1 time a second
  
  while(datatime_err)
  {
    gps.read();//reading time and date
    datatime_err = gps.errTim | gps.errDat;
  }
    
  current_time = "";
  year_str = String("20") + gps.year;
  
  if(gps.month < 10)
    month_str = String("0") + gps.month;
  else
    month_str = gps.month;

  if(gps.day < 10)
    day_str = String("0") + gps.day;
  else
    day_str = gps.day;

  if(gps.Hours < 10)
    hours_str = String("0") + gps.Hours;
  else
    hours_str = gps.Hours;

  if(gps.minutes < 10)
    minutes_str = String("0") + gps.minutes;
  else
    minutes_str = gps.minutes;
    
  //current_time = year_str + String("_") + month_str + String("_") + day_str
  //       + String("_") + hours_str + String("_") + minutes_str;

  current_time = month_str + day_str + hours_str + minutes_str + String(".csv");
  
  myOLED.clrScr();
  myOLED.print("GPS готов", 0, 0);
  myOLED.print(current_time, 0, 1);

      myOLED.print("Месяц: ", 0, 3);
    myOLED.print(gps.month, 50, 3);
    myOLED.print("День : ", 0, 4);
    myOLED.print(gps.day, 50, 4);
    myOLED.print("Час  : ", 0, 5);
    myOLED.print(gps.Hours, 50, 5);
    myOLED.print("Мин  : ", 0, 6);
    myOLED.print(gps.minutes, 50, 6);

  //settings for measuring speed
  //SettingsGPS.composition(NMEA_ZDA, NMEA_VTG);
  //SettingsGPS.updaterate(10);//update 10 time a second
  /*
   * //gps debug
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
       myOLED.clrScr();
       myOLED.print("Ошибка скорости", 0, 0);
       delay(2000);
    }  
    myOLED.clrScr();
    myOLED.print("Скор.:", 0, 0);
    myOLED.print(gps.speed, 50, 0);
    myOLED.print("Курс : ", 0, 1);
    myOLED.print(gps.course, 50, 1);
    delay(1000);
  }
  */
  return;
}

void mul_cnt(){
  if(FREQ == HZ2)
    counter = counter * 2;
  else if(FREQ == HZ4)
    counter = counter * 4;
  else if(FREQ == HZ10)
    counter = counter * 10;
  return;
}

void prepare_to_start(){
  char filename[current_time.length()+1];
  myOLED.clrScr();
  myOLED.print("Проверка датчиков", 0, 0);
  delay(1500);
  sensor_a.begin();
  sensor_a.setFastOffset();
  sensor_p.begin();
  sensor_p.measurement(4);//precision 0-4, max - 4
  sht.begin();
  
  if(GPS_TIME){
    gps_setup();
    current_time.toCharArray(filename, current_time.length()+1);
    myFile = SD.open(filename, FILE_WRITE);
  }
  else
    myFile = SD.open("tstrde.csv", FILE_WRITE);
    
  menu_active = false;
  
  //if(!myFile)
  //print error
    
  attachInterrupt(digitalPinToInterrupt(IND_PIN), inc_cnt, RISING);
  print_ready();
  return;
}

void print_main_menu(byte n){
  myOLED.clrScr();
  for(byte i = 0; i<6; i++)
  {
    if(i == n){
      myOLED.print(main_menu[i], 0, i);
      myOLED.print("<<<", 90, i);
    }
    else
      myOLED.print(main_menu[i], 0, i);
  }
  return;
}

void print_sample_mode_menu(byte n){
  myOLED.clrScr();
  for(byte i = 0; i<3; i++)
  {
    if(i == n){
      myOLED.print(sample_mode_menu[i], 0, i);
      myOLED.print("<<<", 90, i);
    }
    else
      myOLED.print(sample_mode_menu[i], 0, i);
  }
  return;
}

void print_freq_sample_menu(byte n){
  myOLED.clrScr();
  for(byte i = 0; i<5; i++)
  {
    if(i == n){
      myOLED.print(freq_sample_menu[i], 0, i);
      myOLED.print("<<<", 90, i);
    }
    else
      myOLED.print(freq_sample_menu[i], 0, i);
  }
  return;
}

void print_count_rot_menu(byte n){
  myOLED.clrScr();
  for(byte i = 0; i<5; i++)
  {
    if(i == n){
      myOLED.print(count_rot_menu[i], 0, i);
      myOLED.print("<<<", 90, i);
    }
    else
      myOLED.print(count_rot_menu[i], 0, i);
  }
  return;
}

void print_gps_en_menu(byte n){
  myOLED.clrScr();
  for(byte i = 0; i<3; i++)
  {
    if(i == n){
      myOLED.print(gps_en_menu[i], 0, i);
      myOLED.print("<<<", 90, i);
    }
    else
      myOLED.print(gps_en_menu[i], 0, i);
  }
  return;
}

void print_print_disp_menu(byte n){
  myOLED.clrScr();
  for(byte i = 0; i<3; i++)
  {
    if(i == n){
      myOLED.print(print_disp_menu[i], 0, i);
      myOLED.print("<<<", 90, i);
    }
    else
      myOLED.print(print_disp_menu[i], 0, i);
  }
  return;
}

void print_new_params_accepted(){
  myOLED.clrScr();
  myOLED.print("Параметры установлены", 0, 0);
  delay(1500);
  return;
}

void push_next(){
  if(main_menu_select){
    if(cnt_main < 5)
      cnt_main = cnt_main + 1;
    else
      cnt_main = 0;
    print_main_menu(cnt_main);
  }
  else if(sample_mode_menu_select){
    if(cnt_sample_mode < 2)
      cnt_sample_mode = cnt_sample_mode + 1;
    else
      cnt_sample_mode = 0;
    print_sample_mode_menu(cnt_sample_mode);
  }
  else if(freq_sample_menu_select){
    if(cnt_freq_sample < 4)
      cnt_freq_sample = cnt_freq_sample + 1;
    else
      cnt_freq_sample = 0;
    print_freq_sample_menu(cnt_freq_sample);
  }
  else if(count_rot_select){
    if(cnt_count_rot < 4)
      cnt_count_rot = cnt_count_rot + 1;
    else
      cnt_count_rot = 0;
    print_count_rot_menu(cnt_count_rot);
  }
  else if(gps_en_select){
    if(cnt_gps_en < 2)
      cnt_gps_en = cnt_gps_en + 1;
    else
      cnt_gps_en = 0;
    print_gps_en_menu(gps_en_select);
  }
  else if(print_disp_select){
    if(cnt_print_disp < 2)
      cnt_print_disp = cnt_print_disp + 1;
    else
      cnt_print_disp = 0;
    print_print_disp_menu(cnt_print_disp);
  }
  return;
}

void push_select(){
  if(main_menu_select){
    if(cnt_main == 0){
      main_menu_select = false;
      sample_mode_menu_select = true;
      print_sample_mode_menu(cnt_sample_mode);
    }
    else if(cnt_main == 1){
      main_menu_select = false;
      freq_sample_menu_select = true;
      print_freq_sample_menu(cnt_freq_sample);
    }
    else if(cnt_main == 2){
      main_menu_select = false;
      count_rot_select = true;
      print_count_rot_menu(cnt_count_rot);
    }
    else if(cnt_main == 3){
      main_menu_select = false;
      gps_en_select = true;
      print_gps_en_menu(cnt_gps_en);
    }
    else if(cnt_main == 4){
      main_menu_select = false;
      print_disp_select = true;
      print_print_disp_menu(cnt_print_disp);
    }
    else if(cnt_main == 5){
      main_menu_select = false;
      start = true;
      prepare_to_start();
    }
  }
  else if(sample_mode_menu_select){
    if(cnt_sample_mode == 0){
      count_by_time = true;//отметки по времени
      print_new_params_accepted();
    }
    else if(cnt_sample_mode == 1){
      count_by_time = false;//отметки по расстоянию
      print_new_params_accepted();
    }
    sample_mode_menu_select = false;
    main_menu_select = true;
    print_main_menu(cnt_main);
  }
  else if(freq_sample_menu_select){
    if(cnt_freq_sample == 0){
      FREQ = HZ1;//"1 раз в сек   ",
      print_new_params_accepted();
    }
    else if(cnt_freq_sample == 1){
      FREQ = HZ2;//"2 раза в сек  ",
      print_new_params_accepted();
    }
    else if(cnt_freq_sample == 2){
      FREQ = HZ4;//"4 раза в сек  ",
      print_new_params_accepted();
    }
    else if(cnt_freq_sample == 3){
      FREQ = HZ10;//"10 раз в сек  ",
      print_new_params_accepted();
    }
    freq_sample_menu_select = false;
    main_menu_select = true;
    print_main_menu(cnt_main);
  }
  else if(count_rot_select){
    if(cnt_count_rot == 0){
      ROTATION_BOUND = 1;//"10 оборотов   ",
      print_new_params_accepted();
    }
    else if(cnt_count_rot == 1){
      ROTATION_BOUND = 2;//"20 оборотов   ",
      print_new_params_accepted();
    }
    else if(cnt_count_rot == 2){
      ROTATION_BOUND = 4;//"40 оборотов   ",
      print_new_params_accepted();
    }
    else if(cnt_count_rot == 3){
      ROTATION_BOUND = 8;//"80 оборотов   ",
      print_new_params_accepted();
    }
    count_rot_select = false;
    main_menu_select = true;
    print_main_menu(cnt_main);
  }
  else if(gps_en_select){
    if(cnt_gps_en == 0){
      GPS_SPEED = true;//gps speed is on
      print_new_params_accepted();
    }
    else if(cnt_gps_en == 1){
      GPS_SPEED = false;//gps speed is off
      print_new_params_accepted();
    }
    gps_en_select = false;
    main_menu_select = true;
    print_main_menu(cnt_main);
  }
  else if(print_disp_select){
    if(cnt_print_disp == 0){
      PRINT_PARAMS_EN = true;//printing params is on
      print_new_params_accepted();
    }
    else if(cnt_print_disp == 1){
      PRINT_PARAMS_EN = false;//printing params is off
      print_new_params_accepted();
    }
    print_disp_select = false;
    main_menu_select = true;
    print_main_menu(cnt_main);
  }
  return;
}

void inc_cnt(){
  counter++;
  timer_end = millis();
  
  /*
  Serial.print("---rotation--- time = ");
  Serial.print(millis());
  Serial.print(" counter = ");
  Serial.println(counter);
  */
  return;
}
