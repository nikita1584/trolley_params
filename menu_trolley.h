#define HZ1 930
#define HZ2 430
#define HZ4 180
#define HZ10 30

bool main_menu_select;
bool sample_mode_menu_select;
bool freq_sample_menu_select;
bool count_rot_select;
bool gps_en_select;
bool print_disp_select;
bool high_spd_select;
bool count_by_time;
bool menu_active;
bool btnStateNext;
bool btnStateSelect;

bool flagNext = true;
uint32_t btnTimerNext = 0;
bool flagSelect = true;
uint32_t btnTimerSelect = 0;

byte cnt_main;
byte cnt_sample_mode;
byte cnt_freq_sample;
byte cnt_count_rot;
byte cnt_gps_en;
byte cnt_print_disp;
byte cnt_high_spd;

int FREQ;
int ROTATION_BOUND;
bool GPS_SPEED;
bool GPS_TIME;
bool PRINT_PARAMS_EN;
bool h_spd_mode;
byte h_spd_cnt;

void prepare_to_start();
void push_next(iarduino_OLED_txt oled);
void push_select(iarduino_OLED_txt oled, bool start);

char *main_menu[] = {
    "Режим отметок ",
    "Частота записи",
    "Число оборотов",
    "GPS активен   ",
    "Выв. на дисп. ",
    "Высокоск. реж.",
    "Старт!        "};

char *sample_mode_menu[] = {
    "Отм. по врем. ",
    "Отм. по расст.",
    "Выход         "};

char *freq_sample_menu[] = {
    "1 раз в сек   ",
    "2 раза в сек  ",
    "4 раза в сек  ",
    "10 раз в сек  ",
    "Выход         "};

char *count_rot_menu[] = {
    "1  оборотов   ",
    "2  оборотов   ",
    "4  оборотов   ",
    "8  оборотов   ",
    "Выход         "};

char *gps_en_menu[] = {
    "GPS скор. вкл.",
    "GPS скор. выкл",
    "Выход         "};

char *print_disp_menu[] = {
    "Вкл. печ. дисп",
    "Выкл.печ. дисп",
    "Выход         "};

char *high_spd_menu[] = {
    "10 оборотов   ",
    "20 оборотов   ",
    "50 оборотов   ",
    "100 оборотов  ",
    "Выкл.высокоск.",
    "Выход         "};

void menu_actions(iarduino_OLED_txt oled, bool start, const byte NEXT_PIN, const byte SELECT_PIN)
{
  btnStateNext = !digitalRead(NEXT_PIN);
  if (btnStateNext && !flagNext && millis() - btnTimerNext > 100)
  {
    flagNext = true;
    btnTimerNext = millis();
    push_next(oled);
  }
  if (!btnStateNext && flagNext && millis() - btnTimerNext > 100)
  {
    flagNext = false;
    btnTimerNext = millis();
  }

  btnStateSelect = !digitalRead(SELECT_PIN);
  if (btnStateSelect && !flagSelect && millis() - btnTimerSelect > 100)
  {
    flagSelect = true;
    btnTimerSelect = millis();
    push_select(oled, start);
  }

  if (!btnStateSelect && flagSelect && millis() - btnTimerSelect > 100)
  {
    flagSelect = false;
    btnTimerSelect = millis();
  }
}

void print_finish(iarduino_OLED_txt oled, bool file_was_written)
{
  oled.clrScr();
  if (file_was_written)
  {
    oled.print("Файл записан на", 0, 0);
    oled.print("SD успешно", 0, 1);
  }
  else
    oled.print("Ошибка записи", 0, 0);
  return;
}

void print_ready(iarduino_OLED_txt oled)
{
  oled.clrScr();
  oled.print("Готов к запуску", 0, 0);
  return;
}

void print_started(iarduino_OLED_txt oled)
{
  oled.clrScr();
  // oled.setCoding(TXT_UTF8);
  oled.print("Параметры пишутся", 0, 0);
  return;
}

void print_check(iarduino_OLED_txt oled)
{
  oled.clrScr();
  // oled.setCoding(TXT_UTF8);
  oled.print("Проверка датчиков", 0, 0);
  return;
}

void print_main_menu(iarduino_OLED_txt oled, byte n)
{
  oled.clrScr();
  for (byte i = 0; i < 7; i++)
  {
    if (i == n)
    {
      oled.print(main_menu[i], 0, i);
      oled.print("<<<", 90, i);
    }
    else
      oled.print(main_menu[i], 0, i);
  }
  return;
}

void print_sample_mode_menu(iarduino_OLED_txt oled, byte n)
{
  oled.clrScr();
  for (byte i = 0; i < 3; i++)
  {
    if (i == n)
    {
      oled.print(sample_mode_menu[i], 0, i);
      oled.print("<<<", 90, i);
    }
    else
      oled.print(sample_mode_menu[i], 0, i);
  }
  return;
}

void print_freq_sample_menu(iarduino_OLED_txt oled, byte n)
{
  oled.clrScr();
  for (byte i = 0; i < 5; i++)
  {
    if (i == n)
    {
      oled.print(freq_sample_menu[i], 0, i);
      oled.print("<<<", 90, i);
    }
    else
      oled.print(freq_sample_menu[i], 0, i);
  }
  return;
}

void print_count_rot_menu(iarduino_OLED_txt oled, byte n)
{
  oled.clrScr();
  for (byte i = 0; i < 5; i++)
  {
    if (i == n)
    {
      oled.print(count_rot_menu[i], 0, i);
      oled.print("<<<", 90, i);
    }
    else
      oled.print(count_rot_menu[i], 0, i);
  }
  return;
}

void print_gps_en_menu(iarduino_OLED_txt oled, byte n)
{
  oled.clrScr();
  for (byte i = 0; i < 3; i++)
  {
    if (i == n)
    {
      oled.print(gps_en_menu[i], 0, i);
      oled.print("<<<", 90, i);
    }
    else
      oled.print(gps_en_menu[i], 0, i);
  }
  return;
}

void print_print_disp_menu(iarduino_OLED_txt oled, byte n)
{
  oled.clrScr();
  for (byte i = 0; i < 3; i++)
  {
    if (i == n)
    {
      oled.print(print_disp_menu[i], 0, i);
      oled.print("<<<", 90, i);
    }
    else
      oled.print(print_disp_menu[i], 0, i);
  }
  return;
}

void print_high_spd_menu(iarduino_OLED_txt oled, byte n)
{
  oled.clrScr();
  for (byte i = 0; i < 6; i++)
  {
    if (i == n)
    {
      oled.print(high_spd_menu[i], 0, i);
      oled.print("<<<", 90, i);
    }
    else
      oled.print(high_spd_menu[i], 0, i);
  }
  return;
}

void print_new_params_accepted(iarduino_OLED_txt oled)
{
  oled.clrScr();
  oled.print("Параметры установлены", 0, 0);
  delay(1500);
  return;
}

void push_next(iarduino_OLED_txt oled)
{
  if (main_menu_select)
  {
    if (cnt_main < 6)
      cnt_main = cnt_main + 1;
    else
      cnt_main = 0;
    print_main_menu(oled, cnt_main);
  }
  else if (sample_mode_menu_select)
  {
    if (cnt_sample_mode < 2)
      cnt_sample_mode = cnt_sample_mode + 1;
    else
      cnt_sample_mode = 0;
    print_sample_mode_menu(oled, cnt_sample_mode);
  }
  else if (freq_sample_menu_select)
  {
    if (cnt_freq_sample < 4)
      cnt_freq_sample = cnt_freq_sample + 1;
    else
      cnt_freq_sample = 0;
    print_freq_sample_menu(oled, cnt_freq_sample);
  }
  else if (count_rot_select)
  {
    if (cnt_count_rot < 4)
      cnt_count_rot = cnt_count_rot + 1;
    else
      cnt_count_rot = 0;
    print_count_rot_menu(oled, cnt_count_rot);
  }
  else if (gps_en_select)
  {
    if (cnt_gps_en < 2)
      cnt_gps_en = cnt_gps_en + 1;
    else
      cnt_gps_en = 0;
    print_gps_en_menu(oled, gps_en_select);
  }
  else if (print_disp_select)
  {
    if (cnt_print_disp < 2)
      cnt_print_disp = cnt_print_disp + 1;
    else
      cnt_print_disp = 0;
    print_print_disp_menu(oled, cnt_print_disp);
  }
  else if (high_spd_select)
  {
    if (cnt_high_spd < 5)
      cnt_high_spd = cnt_high_spd + 1;
    else
      cnt_high_spd = 0;
    print_high_spd_menu(oled, cnt_high_spd);
  }
  return;
}

void push_select(iarduino_OLED_txt oled, bool start)
{
  if (main_menu_select)
  {
    if (cnt_main == 0)
    {
      main_menu_select = false;
      sample_mode_menu_select = true;
      print_sample_mode_menu(oled, cnt_sample_mode);
    }
    else if (cnt_main == 1)
    {
      main_menu_select = false;
      freq_sample_menu_select = true;
      print_freq_sample_menu(oled, cnt_freq_sample);
    }
    else if (cnt_main == 2)
    {
      main_menu_select = false;
      count_rot_select = true;
      print_count_rot_menu(oled, cnt_count_rot);
    }
    else if (cnt_main == 3)
    {
      main_menu_select = false;
      gps_en_select = true;
      print_gps_en_menu(oled, cnt_gps_en);
    }
    else if (cnt_main == 4)
    {
      main_menu_select = false;
      print_disp_select = true;
      print_print_disp_menu(oled, cnt_print_disp);
    }
    else if (cnt_main == 5)
    {
      main_menu_select = false;
      high_spd_select = true;
      print_high_spd_menu(oled, cnt_high_spd);
    }
    else if (cnt_main == 6)
    {
      main_menu_select = false;
      start = true;
      prepare_to_start();
    }
  }
  else if (sample_mode_menu_select)
  {
    if (cnt_sample_mode == 0)
    {
      count_by_time = true; //отметки по времени
      h_spd_mode = false;
      print_new_params_accepted(oled);
    }
    else if (cnt_sample_mode == 1)
    {
      count_by_time = false; //отметки по расстоянию
      print_new_params_accepted(oled);
    }
    sample_mode_menu_select = false;
    main_menu_select = true;
    print_main_menu(oled, cnt_main);
  }
  else if (freq_sample_menu_select)
  {
    if (cnt_freq_sample == 0)
    {
      FREQ = HZ1; //"1 раз в сек   ",
      print_new_params_accepted(oled);
      h_spd_mode = false;
    }
    else if (cnt_freq_sample == 1)
    {
      FREQ = HZ2; //"2 раза в сек  ",
      print_new_params_accepted(oled);
      h_spd_mode = false;
    }
    else if (cnt_freq_sample == 2)
    {
      FREQ = HZ4; //"4 раза в сек  ",
      print_new_params_accepted(oled);
      h_spd_mode = false;
    }
    else if (cnt_freq_sample == 3)
    {
      FREQ = HZ10; //"10 раз в сек  ",
      print_new_params_accepted(oled);
      h_spd_mode = false;
    }
    freq_sample_menu_select = false;
    main_menu_select = true;
    print_main_menu(oled, cnt_main);
  }
  else if (count_rot_select)
  {
    if (cnt_count_rot == 0)
    {
      ROTATION_BOUND = 1; //"10 оборотов   ",
      print_new_params_accepted(oled);
    }
    else if (cnt_count_rot == 1)
    {
      ROTATION_BOUND = 2; //"20 оборотов   ",
      print_new_params_accepted(oled);
    }
    else if (cnt_count_rot == 2)
    {
      ROTATION_BOUND = 4; //"40 оборотов   ",
      print_new_params_accepted(oled);
    }
    else if (cnt_count_rot == 3)
    {
      ROTATION_BOUND = 8; //"80 оборотов   ",
      print_new_params_accepted(oled);
    }
    count_rot_select = false;
    main_menu_select = true;
    print_main_menu(oled, cnt_main);
  }
  else if (gps_en_select)
  {
    if (cnt_gps_en == 0)
    {
      GPS_SPEED = true; // gps speed is on
      print_new_params_accepted(oled);
    }
    else if (cnt_gps_en == 1)
    {
      GPS_SPEED = false; // gps speed is off
      print_new_params_accepted(oled);
    }
    gps_en_select = false;
    main_menu_select = true;
    print_main_menu(oled, cnt_main);
  }
  else if (print_disp_select)
  {
    if (cnt_print_disp == 0)
    {
      PRINT_PARAMS_EN = true; // printing params is on
      print_new_params_accepted(oled);
    }
    else if (cnt_print_disp == 1)
    {
      PRINT_PARAMS_EN = false; // printing params is off
      print_new_params_accepted(oled);
    }
    print_disp_select = false;
    main_menu_select = true;
    print_main_menu(oled, cnt_main);
  }
  else if (high_spd_select)
  {
    if (cnt_high_spd == 0)
    {
      h_spd_mode = true; // high speed mode is on
      h_spd_cnt = 10;
      print_new_params_accepted(oled);
    }
    else if (cnt_high_spd == 1)
    {
      h_spd_mode = true; // high speed mode is on
      h_spd_cnt = 20;
      print_new_params_accepted(oled);
    }
    else if (cnt_high_spd == 2)
    {
      h_spd_mode = true; // high speed mode is on
      h_spd_cnt = 50;
      print_new_params_accepted(oled);
    }
    else if (cnt_high_spd == 3)
    {
      h_spd_mode = true; // high speed mode is on
      h_spd_cnt = 100;
      print_new_params_accepted(oled);
    }
    else if (cnt_high_spd == 4)
    {
      h_spd_mode = false; // high speed mode is off
      print_new_params_accepted(oled);
    }
    // else if(cnt_high_spd == 5){
    //   h_spd_mode = false;//high speed mode is off
    //   print_new_params_accepted();
    // }
    high_spd_select = false;
    main_menu_select = true;
    print_main_menu(oled, cnt_main);
  }
  return;
}
