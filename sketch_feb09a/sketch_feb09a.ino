// Подключение библиотек
#include <iarduino_MultiServo.h>                             
#include <iarduino_OLED_txt.h>
#include <FlexiTimer2.h> //to set a timer to manage all servos
#include <Ultrasonic.h>
#include <CyberLib.h>
#include <dht11.h> 
//#include "Ai_WS2811.h"


iarduino_MultiServo MSS; 
iarduino_OLED_txt myOLED(0x78);
Ultrasonic ultrasonic(9, 8);    // Trig - 9, Echo - 8 
dht11 DHT;                      // Объявление переменной класса dht11
//Ai_WS2811 ws2811;

// Пины на модуле для сервоприводов от 1 до 4 ноги
//const int servo_pin[4][3] = {{4, 5, 6}, {12, 13, 14}, {0, 1, 2}, {8, 9, 10}};/* Size of the robot ---------------------------------------------------------*/
const int servo_pin[4][3] = {{8, 9, 10}, {12, 13, 14}, {4, 5, 6}, {0, 1, 2}};
const float length_a = 50;
const float length_b = 77.1;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;
/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;
/* Переменные для движений ----------------------------------------------------*/
volatile float site_now[4][3];    //real-time coordinates of the end of each leg
volatile float site_expect[4][3]; //expected coordinates of the end of each leg
float temp_speed[4][3];   //each axis' speed, needs to be recalculated before each movement
float move_speed;     //movement speed
float speed_multiple = 1; //movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;      //+1/0.02s, for automatic rest
//functions' parameter
const float KEEP = 255;
// Константы для поворотов --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

/* Переменные для измерений  ---------------------------------------------------------------------------*/
const int voltInPin = A0;  // Analog input pin
float volt = 0;
unsigned long time = 0, timer = 0;
/* Переменные связаные с lcd ---------------------------------------------------------------------------*/
#define oled_interval 5000 // 5 сек.
extern uint8_t SmallFontRus[];
//extern uint8_t MediumFontRus[];
/* Остальное ---------------------------------------------------------------------------*/
bool  auto_mode= false, status = false, first_info = true;
int distance = 30;
float dist_cm = 0;
byte a = 0x00, b = 0xff;
unsigned long wait_time = 0, wait_timer = 0;
#define wait_interval 5000
#define DHT11_PIN 5
#define buzzer 4
#define DATA_PIN 8

/* ---------------------------------------------------------------------------*/

void setup() 
{
  Serial.begin(9600);
  
  myOLED.begin();                                    // Инициируем работу с дисплеем.
  myOLED.setFont(SmallFontRus);

  //ws2811.init(DATA_PIN);
  
  MSS.servoSet(0, SERVO_SG90);   
  MSS.servoSet(1, SERVO_SG90);   
  MSS.servoSet(2, SERVO_SG90);
      
  MSS.servoSet(4, SERVO_SG90);   
  MSS.servoSet(5, SERVO_SG90);   
  MSS.servoSet(6, SERVO_SG90);
      
  MSS.servoSet(8, SERVO_SG90);   
  MSS.servoSet(9, SERVO_SG90);   
  MSS.servoSet(10, SERVO_SG90);
      
  MSS.servoSet(12, SERVO_SG90);   
  MSS.servoSet(13, SERVO_SG90);   
  MSS.servoSet(14, SERVO_SG90);       
                       
  MSS.begin();
  _delay_us(1000);
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  //start servo service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  stand();
  _delay_us(1000);
  step_forward(5);
  //_delay_us(1000);
 // turn_left(5);
//  hand_wave(3);
//  delay(1000);
//  hand_shake(3);
  //ws2811.setColor(a,a,b);
  //sendLEDs();
  /*
  _delay_us(1000);
  step_forward(10);
  _delay_us(1000);
  turn_right(5);
  _delay_us(1000);
  step_back(5);
  _delay_us(1000);
  hand_wave(3);
  _delay_us(1000);
  turn_left(10);
  _delay_us(1000);
  step_forward(10);
  _delay_us(1000);
  hand_shake(3);
  _delay_us(1000);     
  turn_right(5);
  _delay_us(1000); 
  step_forward(3);
  _delay_us(1000); 
  turn_right(5);
  _delay_us(1000); 
  step_forward(10);
  _delay_us(1000); 
  sit();
  _delay_us(5000);
  */
}

void loop() 
{

  char input;
  if(Serial.available() > 0){
    input = Serial.read();
    Serial.println(input);
    if (input == 'f'){
      step_forward(1);
    } else if (input == 'b'){
      step_back(1);
    } else if (input == 'l'){
      turn_left(1);
    } else if (input == 'r'){
      turn_right(1);
    }else if(input == 'w'){
      hand_wave(3);
    }else if(input == 's'){
      hand_shake(3);
    }
    while (Serial.available()) { Serial.read(); delay(5); } // чистка буфера
  }
  
  dist_cm = ultrasonic.read(CM); // ultrasonic.Ranging(CM);
  // Автоматический режим
  if(auto_mode == true){
    if (dist_cm > distance){
      step_forward(2);
    }else{
      //while(dist_cm > distance){
      turn_right(2);
      //dist_cm = ultrasonic.Ranging(CM);
      //Serial.println(dist_cm);
      //}
    }
  }
  
  //char firstChar = input.charAt(0);
  //String data = input.substring(0);
  
}
/*
void sendLEDs()
{
  cli();
  for (byte i = 0; i < 1; i++) {
    ws2811.send();
  }
  sei();
}
*/
/*
  - Функция аналог map только для float
   ---------------------------------------------------------------------------*/
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
/*
  - Функция которая измеряет вольты с аккумулятора робота
   ---------------------------------------------------------------------------*/
float get_volt() {
  float inputVolt = 0;
  float volt = 0;
  for(int i = 0; i < 100; i++){
    inputVolt = analogRead(voltInPin);
    volt += inputVolt;
    _delay_us(5);
  }
  volt = volt / 100;
  return mapfloat(volt, 0, 1023, 0, 3.8) * 2;
}

void sit(void)
{
  status = false;
  move_speed = stand_seat_speed;
  for(int leg = 0; leg < 4; leg++){
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}
/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void stand(void)
{
  status = true;
  wait_time = 0, wait_timer = 0;
  move_speed = stand_seat_speed;
  for(int leg = 0; leg < 4; leg++){
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}


/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while(step-- > 0){
    if(site_now[3][1] == y_start){
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }else{
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while(step-- > 0){
    if(site_now[2][1] == y_start){
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }else{
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while(step-- > 0){
    if(site_now[2][1] == y_start){
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }else{
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while(step-- > 0){
    if(site_now[3][1] == y_start){
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }else{
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

// add by RegisHsu

void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if(site_now[3][1] == y_start){
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for(int j = 0; j < i; j++){
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }else{
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if(site_now[3][1] == y_start){
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for(int j = 0; j < i; j++){
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }else{
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for(int j = 0; j < i; j++){
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void head_up(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
}

void head_down(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}
/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;
  //if(!status) stand();
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 3; j++){
      if(abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
  // Вывод на lcd
  time = millis();
  if(time - timer > oled_interval || first_info){
    first_info = false;
    timer = time;
    volt = get_volt();
    int chk;
    // Мониторинг ошибок
    chk = DHT.read(DHT11_PIN);    // Чтение данных
    // Выводим показания влажности и температуры
    myOLED.clrScr();
    myOLED.print("V: ", 0, 1);
    myOLED.print(volt, 15, 1);
    myOLED.print("T: ", 0, 3);
    myOLED.print(DHT.temperature, 15, 3);
    myOLED.print("Влага: ", 0, 5);
    myOLED.print(DHT.humidity, 40, 5);
  }
  // Спящий режим
//  if (wait_time - wait_timer > wait_interval && status == true){
//    wait_timer = wait_time;
//    sit();
//  }
}

/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if(x != KEEP)
    length_x = x - site_now[leg][0];
  if(y != KEEP)
    length_y = y - site_now[leg][1];
  if(z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if(x != KEEP)
    site_expect[leg][0] = x;
  if(y != KEEP)
    site_expect[leg][1] = y;
  if(z != KEEP)
    site_expect[leg][2] = z;

  wait_time = millis();
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  while(1)
    if(site_now[leg][0] == site_expect[leg][0])
      if(site_now[leg][1] == site_expect[leg][1])
        if(site_now[leg][2] == site_expect[leg][2])
          break;
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for(int i = 0; i < 4; i++)
    wait_reach(i);
}

/*
  - trans site from cartesian to polar
  - mathematical model 2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
// CHANGED!!!

void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if(leg == 0){
    alpha += 90;
    beta = 180 - beta;
    gamma += 90;
  }
  else if(leg == 1){
    alpha = 90 - alpha;
    beta = beta;
    gamma = 70 - gamma;
  }
  else if(leg == 2){
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if(leg == 3){
    
    alpha = 90 - alpha;
    beta = beta;
    gamma += 110;
  }

  MSS.servoWrite(servo_pin[leg][0], alpha);
  MSS.servoWrite(servo_pin[leg][1], beta);
  MSS.servoWrite(servo_pin[leg][2], gamma);
}
