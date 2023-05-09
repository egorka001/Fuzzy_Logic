#include <MQ135.h>

#include <DHT.h>
#include <DHT_U.h>

#include <GyverOLED.h>
#include <charMap.h>
#include <icons_7x7.h>
#include <icons_8x8.h>

#define DT 3
#define CLK 4
#define DHTPIN 5
#define DHTTYPE DHT11
#define MQPIN A0

const int e = 2.718281;

//mq135 vars
MQ135 mq(MQPIN);
uint16_t gas, gas_old, gas_ms = 1000;
uint16_t gas_min = 400, gas_max = 3000;

//dht vars
DHT dht(DHTPIN, DHTTYPE);
uint8_t hum = 0, temp = 0, hum_old, temp_old;
uint8_t hum_min = 0, hum_max = 100, temp_min = 18, temp_max = 32;

//encoder vars
uint8_t clk_st;
uint8_t clk_st_old = 1;
uint8_t dt_st;
uint8_t count = 128;
const uint8_t c_count = 128;
const uint8_t div_count = 4; 

//screen vars
const uint8_t screen_count = 4;
uint8_t curr_screen_num = 3;
GyverOLED<SSD1306_128x32, OLED_NO_BUFFER> screen;

//4 экрана: hum, temp, gas, status
uint8_t status = 0, status_old = 0;

uint8_t rules[27][4] = {{1, 1, 1, 1},
                 {2, 1, 1, 2},
                 {3, 1, 1, 2},
                 {1, 2, 1, 1},
                 {2, 2, 1, 2},
                 {3, 2, 1, 3},
                 {1, 3, 1, 3},
                 {2, 3, 1, 3},
                 {3, 3, 1, 4},
                 {1, 1, 2, 1}, 
                 {2, 1, 2, 2},
                 {3, 1, 2, 3}, 
                 {1, 2, 2, 1}, 
                 {2, 2, 2, 3}, 
                 {3, 2, 2, 4}, 
                 {1, 3, 2, 3}, 
                 {2, 3, 2, 4}, 
                 {3, 3, 2, 4}, 
                 {1, 1, 3, 1}, 
                 {2, 1, 3, 2}, 
                 {3, 1, 3, 2}, 
                 {1, 2, 3, 2}, 
                 {2, 2, 3, 3}, 
                 {3, 2, 3, 4}, 
                 {1, 3, 3, 3},
                 {2, 3, 3, 4}, 
                 {3, 3, 3, 4}};

float max_full[101];

void setup() 
{
  Serial.begin(9600);

  dht.begin();

  screen.init();
  screen.clear();
  screen.setScale(3);
  screen.home();
  screen.print("NeoHome");
  //delay(1000);

  screen.clear();
  screen.setScale(2);
  screen.home();
  screen.print("heating");
  //delay(1000);  
  screen.print(".");
  //delay(1000); 
  screen.print(".");
  //delay(1000); 
  screen.print(".");
  //delay(1000); 

  hum = dht.readHumidity();
  temp = dht.readTemperature();
  gas = mq.getCorrectedPPM(temp, hum);

  out_screen(curr_screen_num);
}

void loop() 
{
  static uint32_t tmr = millis();

  if(millis() - tmr >= gas_ms)
  {
    gas = mq.getCorrectedPPM(temp, hum);
    if(gas < gas_min)
      gas = gas_min;
    if(gas > gas_max)
      gas = gas_max;

    hum = dht.readHumidity();
    if(hum < hum_min)
      hum = hum_min;
    if(hum > hum_max)
      hum = hum_max;

    temp = dht.readTemperature();
    if(temp < temp_min)
      temp = temp_min;
    if(temp > temp_max)
      temp = temp_max;

    status = fuzzy_logic(gas, hum, temp);

    tmr = millis();
  }

  check_encoder(count);

  if(curr_screen_num == 0 && hum != hum_old)
    out_screen(curr_screen_num);

  if(curr_screen_num == 1 && temp != temp_old)
    out_screen(curr_screen_num);

  if(curr_screen_num == 2 && gas != gas_old)
    out_screen(curr_screen_num);  

  if(curr_screen_num == 3 && status != status_old)
    out_screen(curr_screen_num);

  if(change_screen(curr_screen_num, count))
    out_screen(curr_screen_num);

  hum_old = hum;
  temp_old = temp;
  gas_old = gas;
  status_old = status;
}

float gaussmf(float x, float a, float b)
{
  return pow(e, -((x - b) * (x - b)) / (2 * a * a));
}

float trapmf(float x, float a, float b, float c, float d)
{
  if(x >= b && x <= c)
    return 1;

  if(x <= a || x >= d)
    return 0;

  if(x < b)
    return x * (1 / (b - a)) + a / (a - b);

  if(x > c)
    return x * (1 / (c - d)) + d / (d - c);
}

float min3(float a, float b, float c)
{
  return min(a, min(b, c));
}

float max4(float a, float b, float c, float d)
{
  return max(max(a, b), max(c, d));
}

uint8_t fuzzy_logic(uint16_t gas, uint8_t hum, uint8_t temp)
{
  float gas_fuz[3], hum_fuz[3], temp_fuz[3];

  gas_fuz[0] = gaussmf(float(gas), 300, 400);
  gas_fuz[1] = gaussmf(float(gas), 300, 1400);
  gas_fuz[2] = gaussmf(float(gas), 600, 3000);

  temp_fuz[0] = gaussmf(float(temp), 2, 18);
  temp_fuz[1] = trapmf(float(temp), 20, 22, 25, 27);
  temp_fuz[2] = gaussmf(float(temp), 3, 32);

  hum_fuz[0] = gaussmf(float(hum), 15, 0);
  hum_fuz[1] = gaussmf(float(hum), 10, 50);
  hum_fuz[2] = gaussmf(float(hum), 15, 100);

  float rules_min[27];
  float act_max[4] = {0, 0, 0, 0};

  for(uint8_t i = 0; i < 27; ++i)
  {
    rules_min[i] = min3(gas_fuz[rules[i][0] - 1], 
                       temp_fuz[rules[i][1] - 1], 
                        hum_fuz[rules[i][2] - 1]);
  }

  for(uint8_t i = 0; i < 27; ++i)
  {
    if(rules_min[i] > act_max[rules[i][3] - 1])
      act_max[rules[i][3] - 1] = rules_min[i];
  }

  float a, b, c, d;
  for(uint8_t i = 0; i < 101; ++i)
  {
    a = gaussmf(i, 10, 0);
    b = gaussmf(i, 10, 35);
    c = gaussmf(i, 10, 65);
    d = gaussmf(i, 15, 100);

    if(a > act_max[0])
      a = act_max[0];
    if(b > act_max[1])
      b = act_max[1];
    if(c > act_max[2])
      c = act_max[2];
    if(d > act_max[3])
      d = act_max[3];

    max_full[i] = max4(a, b, c, d);
  }

  float up = 0, down = 0, out;

  for(uint8_t i = 0; i < 101; ++i)
  { 
    up += float(i) * max_full[i];
    down += max_full[i];
  }

  out = up / down;

  return up / down;
}

void out_screen(uint8_t curr_screen_num)
{
  screen.clear();
  screen.home();

  if(curr_screen_num == 0)
  {
    screen.print("hum: ");
    screen.print(hum);
    screen.print(" %");
  }

  if(curr_screen_num == 1)
  {
    screen.print("temp: ");
    screen.print(temp);
    screen.print(" C");
  }

  if(curr_screen_num == 2)
  {
    screen.print("gas: ");
    screen.setCursor(0, 2);
    screen.print(gas);
    screen.print(" ppm");
  }

  if(curr_screen_num == 3)
  {
    screen.print("window pos: ");
    screen.setCursor(0, 2);
    screen.print(status);
  }
}

void check_encoder(uint8_t& count)
{
  clk_st = digitalRead(CLK);
  dt_st = digitalRead(DT);

  if(clk_st_old != clk_st)
  {
    if(clk_st == 0)
    {
      if(dt_st)
        ++count;
      else
        --count;
    }
    clk_st_old = clk_st;
  }
}

uint8_t change_screen(uint8_t& curr_screen_num, uint8_t& count)
{
  if(count < c_count - div_count)
  {
    curr_screen_num = prev_screen(curr_screen_num);
    count = c_count;
    return 1;
  }

  if(count > c_count + div_count)
  {
    curr_screen_num = next_screen(curr_screen_num);
    count = c_count;
    return 1;
  }

  return 0;
}

uint8_t next_screen(uint8_t curr_screen_num)
{
  if(curr_screen_num == screen_count - 1)
    return 0;
  return ++curr_screen_num;
}

uint8_t prev_screen(uint8_t curr_screen_num)
{
  if(curr_screen_num == 0)
    return screen_count - 1;
  return --curr_screen_num;
}
