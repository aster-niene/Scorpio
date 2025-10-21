/*
 * ======================================================================
 * Адаптированная прошивка "Scorpio" для ESP32-S3
 * * Оригинальный код: "Mark W" для Arduino Mega
 * Адаптация: Gemini (для Freenove ESP32-S3)
 * * Что изменено:
 * 1. Удалена библиотека PS2X_lib.
 * 2. Добавлена поддержка Bluetooth Low Energy (BLE) для управления с мобильного приложения.
 * 3. Библиотека Servo.h заменена на ESP32Servo.h.
 * 4. Удалена логика для 8-сегментного LED-индикатора.
 * 5. Пины сервоприводов и батареи вынесены в отдельный блок для настройки.
 * ======================================================================
*/

//***********************************************************************
// Includes
//***********************************************************************
#include <ESP32Servo.h>     // Используем библиотеку для ESP32
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//***********************************************************************
// [!!!] НАСТРОЙКА ПИНОВ [!!!]
//***********************************************************************

// [!!!] ЗАДАЙТЕ ПИН, к которому подключен делитель напряжения от батареи
// На плате Freenove это, вероятно, пин "VB" (GPIO 1)
const int BATT_VOLTAGE_PIN = 1;

/*
 [!!!] ВАЖНО: Укажите, к какому GPIO-пину ESP32 подключен КАЖДЫЙ из 18 сервоприводов.
 Я не могу видеть вашу точную распайку на фото.
 Замените "???" на номера пинов с вашей платы Freenove (например, 1, 2, 3... 48).
*/
const int COXA1_SERVO  = 2;   // ПРИМЕР: Нога 1, серво 1 (тазобедренный)
const int FEMUR1_SERVO = 3;   // ПРИМЕР: Нога 1, серво 2 (бедро)
const int TIBIA1_SERVO = 4;   // ПРИМЕР: Нога 1, серво 3 (голень)

const int COXA2_SERVO  = 5;   // ПРИМЕР
const int FEMUR2_SERVO = 6;   // ПРИМЕР
const int TIBIA2_SERVO = 7;   // ПРИМЕР

const int COXA3_SERVO  = 8;   // ПРИМЕР
const int FEMUR3_SERVO = 9;   // ПРИМЕР
const int TIBIA3_SERVO = 10;  // ПРИМЕР

const int COXA4_SERVO  = 40;  // ПРИМЕР
const int FEMUR4_SERVO = 41;  // ПРИМЕР
const int TIBIA4_SERVO = 42;  // ПРИМЕР

const int COXA5_SERVO  = 37;  // ПРИМЕР
const int FEMUR5_SERVO = 38;  // ПРИМЕР
const int TIBIA5_SERVO = 39;  // ПРИМЕР

const int COXA6_SERVO  = 34;  // ПРИМЕР
const int FEMUR6_SERVO = 35;  // ПРИМЕР
const int TIBIA6_SERVO = 36;  // ПРИМЕР


//***********************************************************************
// Константы BLE (для мобильного приложения)
//***********************************************************************
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define JOYSTICK_CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8" // 4 байта (LX, LY, RX, RY)
#define BUTTONS_CHAR_UUID      "a519c676-135a-48a5-aa27-6c826a0c0a32" // 1 байт (битовая маска кнопок)

// Глобальные переменные для хранения данных с BLE
// volatile, т.к. они изменяются в прерывании (callback)
volatile uint8_t g_ble_joystick_data[4] = {128, 128, 128, 128}; // LX, LY, RX, RY (128 = центр)
volatile uint8_t g_ble_buttons_data = 0;
volatile bool g_ble_new_data = false;
volatile bool g_ble_connected = false;

//***********************************************************************
// Константы робота (из оригинального файла)
//***********************************************************************
const int COXA_LENGTH = 46;   [cite: 15] // Длины частей ног
const int FEMUR_LENGTH = 43;  [cite: 16]
const int TIBIA_LENGTH = 90;  [cite: 16]
const int TRAVEL = 30;        [cite: 16]

const long A12DEG = 209440;   [cite: 17]
const long A30DEG = 523599;   [cite: 17]

const int FRAME_TIME_MS = 20; [cite: 18]

const float HOME_X[6] = {  63.0,   0.0, -63.0,  -63.0,    0.0,  63.0}; [cite: 19]
const float HOME_Y[6] = {  63.0, 89.0,  63.0,  -63.0, -89.0, -63.0}; [cite: 20]
const float HOME_Z[6] = { -60.0, -60.0, -60.0,  -60.0,  -60.0, -60.0}; [cite: 21]
const float BODY_X[6] = { 66.0,  0.0, -66.0, -66.0,    0.0, 66.0}; [cite: 22]
const float BODY_Y[6] = {  49.5, 61.6,   49.5,  -49.5,  -61.6, -49.5}; [cite: 23]
const float BODY_Z[6] = {   0.0,  0.0,    0.0,    0.0,    0.0,   0.0}; [cite: 24]
const int COXA_CAL[6]  = {2, -1, -1, -3, -2, -3};   [cite: 25]
const int FEMUR_CAL[6] = {4, -2,  0, -1,  0,  0};   [cite: 26]
const int TIBIA_CAL[6] = {0, -3, -3, -2, -3, -1};   [cite: 27]

//***********************************************************************
// Variable Declarations
//***********************************************************************
unsigned long currentTime;
unsigned long previousTime;

int temp;
int mode;
int gait;
int gait_speed;
int reset_position;
int capture_offsets;

int batt_voltage; 
int batt_voltage_index;
int batt_voltage_array[50];
long batt_voltage_sum;

float L0, L3;
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

int leg1_IK_control, leg6_IK_control;
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

int leg_num;
int z_height_LED_color;
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int z_height_left, z_height_right;
int commandedX, commandedY, commandedR;
int translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

int tripod_case[6]   = {1,2,1,2,1,2};   [cite: 34]
int ripple_case[6]   = {2,6,4,1,3,5};   [cite: 35]
int wave_case[6]     = {1,2,3,4,5,6};   [cite: 35]
int tetrapod_case[6] = {1,3,2,1,2,3};   [cite: 36]

//***********************************************************************
// Object Declarations
//***********************************************************************
Servo coxa1_servo;
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;

// Объекты BLE
BLEServer* pServer = NULL;
BLECharacteristic* pJoystickCharacteristic = NULL;
BLECharacteristic* pButtonsCharacteristic = NULL;


//***********************************************************************
// BLE Callbacks
//***********************************************************************

// Класс для обработки событий подключения/отключения
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      g_ble_connected = true;
      Serial.println("BLE Client Connected");
    }

    void onDisconnect(BLEServer* pServer) {
      g_ble_connected = false;
      Serial.println("BLE Client Disconnected");
      BLEDevice::startAdvertising(); // Перезапускаем рекламу
    }
};

// Класс для обработки записи в характеристики
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string uuid = pCharacteristic->getUUID().toString();
      
      if (uuid == JOYSTICK_CHAR_UUID) {
        std::string value = pCharacteristic->getValue();
        if (value.length() == 4) {
          g_ble_joystick_data[0] = value[0]; // LX
          g_ble_joystick_data[1] = value[1]; // LY
          g_ble_joystick_data[2] = value[2]; // RX
          g_ble_joystick_data[3] = value[3]; // RY
          g_ble_new_data = true;
        }
      }
      
      if (uuid == BUTTONS_CHAR_UUID) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
          g_ble_buttons_data = value[0];
          g_ble_new_data = true;
        }
      }
    }
};

//***********************************************************************
// Initialization Routine
//***********************************************************************
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting Scorpio ESP32 Setup...");

  // Настройка пина батареи
  pinMode(BATT_VOLTAGE_PIN, INPUT);

  // --- Настройка сервоприводов ---
  // ESP32Servo позволяет выделить таймеры для серво
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Устанавливаем частоту ШИМ для серво
  coxa1_servo.setPeriodHertz(50);
  // ... (можно установить для всех)

  // Подключаем серво к пинам.
  // Используются мин/макс значения из оригинального кода [cite: 39, 40]
  coxa1_servo.attach(COXA1_SERVO, 610, 2400);
  femur1_servo.attach(FEMUR1_SERVO, 610, 2400);
  tibia1_servo.attach(TIBIA1_SERVO, 610, 2400);
  coxa2_servo.attach(COXA2_SERVO, 610, 2400);
  femur2_servo.attach(FEMUR2_SERVO, 610, 2400);
  tibia2_servo.attach(TIBIA2_SERVO, 610, 2400);
  coxa3_servo.attach(COXA3_SERVO, 610, 2400);
  femur3_servo.attach(FEMUR3_SERVO, 610, 2400);
  tibia3_servo.attach(TIBIA3_SERVO, 610, 2400);
  coxa4_servo.attach(COXA4_SERVO, 610, 2400);
  femur4_servo.attach(FEMUR4_SERVO, 610, 2400);
  tibia4_servo.attach(TIBIA4_SERVO, 610, 2400);
  coxa5_servo.attach(COXA5_SERVO, 610, 2400);
  femur5_servo.attach(FEMUR5_SERVO, 610, 2400);
  tibia5_servo.attach(TIBIA5_SERVO, 610, 2400);
  coxa6_servo.attach(COXA6_SERVO, 610, 2400);
  femur6_servo.attach(FEMUR6_SERVO, 610, 2400);
  tibia6_servo.attach(TIBIA6_SERVO, 610, 2400);

  // --- Настройка BLE Server ---
  Serial.println("Initializing BLE...");
  BLEDevice::init("Scorpio_ESP32");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Характеристика для Джойстиков (4 байта, Read/Write)
  pJoystickCharacteristic = pService->createCharacteristic(
                             JOYSTICK_CHAR_UUID,
                             BLECharacteristic::PROPERTY_READ |
                             BLECharacteristic::PROPERTY_WRITE
                           );
  pJoystickCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pJoystickCharacteristic->setValue(g_ble_joystick_data, 4);

  // Характеристика для Кнопок (1 байт, Read/Write)
  pButtonsCharacteristic = pService->createCharacteristic(
                             BUTTONS_CHAR_UUID,
                             BLECharacteristic::PROPERTY_READ |
                             BLECharacteristic::PROPERTY_WRITE
                           );
  pButtonsCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pButtonsCharacteristic->setValue(&g_ble_buttons_data, 1);
  
  pService->start();

  // Начинаем "рекламировать" (advertising) себя
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions properly on iPhone
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE Server Started. Waiting for client...");

  // Инициализация переменных (из оригинального кода [cite: 48-52])
  for(batt_voltage_index=0; batt_voltage_index<50; batt_voltage_index++)
    batt_voltage_array[batt_voltage_index] = 0;
  batt_voltage_sum = 0;
  batt_voltage_index = 0;

  for(leg_num=0; leg_num<6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
  capture_offsets = false;
  step_height_multiplier = 1.0;
  mode = 0;
  gait = 0;
  gait_speed = 0;
  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;
}


//***********************************************************************
// Main Program
//***********************************************************************
void loop() 
{
  currentTime = millis();
  
  // Выполняем логику только по таймеру (каждые FRAME_TIME_MS)
  if((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime;

    // Только если есть подключение по BLE, обрабатываем ввод
    if (g_ble_connected) {
      process_ble_gamepad();
    } else {
      // Если нет подключения, робот стоит на месте
      mode = 0;
      reset_position = true;
    }
    
    // Код из оригинального loop() [cite: 56-69]
    if(reset_position == true)
    {
      for(leg_num=0; leg_num<6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }
    
    if(mode < 99)
    {
      for(leg_num=0; leg_num<6; leg_num++)
        leg_IK(leg_num,current_X[leg_num]+offset_X[leg_num],current_Y[leg_num]+offset_Y[leg_num],current_Z[leg_num]+offset_Z[leg_num]);
    }

    if(mode != 4) 
    {
      leg1_IK_control = true;
      leg6_IK_control = true; 
    }

    battery_monitor();
    // print_debug(); // Раскомментируйте для отладки

    if(mode == 1)
    {
      if(gait == 0) tripod_gait();
      if(gait == 1) wave_gait();
      if(gait == 2) ripple_gait();
      if(gait == 3) tetrapod_gait();
    }
    if(mode == 2) translate_control();
    if(mode == 3) rotate_control();
    if(mode == 4) one_leg_lift();
    if(mode == 99) set_all_90();
  }
}


//***********************************************************************
// Process BLE controller inputs
//
// НОВАЯ ФУНКЦИЯ для замены process_gamepad()
//
// Вам нужно будет создать мобильное приложение, которое отправляет
// битовую маску для кнопок.
// Пример:
#define PSB_PAD_DOWN   (1 << 0)  // 1
#define PSB_PAD_LEFT   (1 << 1)  // 2
#define PSB_PAD_UP     (1 << 2)  // 4
#define PSB_PAD_RIGHT  (1 << 3)  // 8
#define PSB_TRIANGLE   (1 << 4)  // 16
#define PSB_SQUARE     (1 << 5)  // 32
#define PSB_CIRCLE     (1 << 6)  // 64
#define PSB_CROSS      (1 << 7)  // 128
// (Остальные кнопки (L1/R1, L2/R2, START, SELECT) можно добавить во второй байт,
// но для этого нужно изменить BLE характеристику)
//***********************************************************************
void process_ble_gamepad()
{
  // Данные джойстиков (значения 0-255)
  // [0] = LX, [1] = LY, [2] = RX, [3] = RY
  uint8_t lx = g_ble_joystick_data[0];
  uint8_t ly = g_ble_joystick_data[1];
  uint8_t rx = g_ble_joystick_data[2];
  uint8_t ry = g_ble_joystick_data[3];

  // Данные кнопок (битовая маска)
  uint8_t buttons = g_ble_buttons_data;

  // Сбрасываем флаг новых данных
  g_ble_new_data = false;
  // Сбрасываем данные кнопок, чтобы они не "залипали"
  // (Приложение должно отправлять "0", когда все кнопки отпущены)
  // g_ble_buttons_data = 0; // -> Раскомментируйте, если приложение отправляет нажатия однократно

  // --- Логика из оригинального process_gamepad() ---
  // [cite: 69]
  if(buttons & PSB_PAD_DOWN)
  {
    mode = 0;
    gait = 0;
    reset_position = true;
  }
  // [cite: 70]
  if(buttons & PSB_PAD_LEFT)
  {
    mode = 0;
    gait = 1;
    reset_position = true;
  }
  // [cite: 71]
  if(buttons & PSB_PAD_UP)
  {
    mode = 0;
    gait = 2;
    reset_position = true;
  }
  // [cite: 72]
  if(buttons & PSB_PAD_RIGHT)
  {
    mode = 0;
    gait = 3;
    reset_position = true;
  }
  
  // [cite: 78]
  if(buttons & PSB_TRIANGLE)
  {
    mode = 1;
    reset_position = true;
  }
  
  // [cite: 80]
  if(buttons & PSB_SQUARE)
  {
    mode = 2;
    reset_position = true;
  }
  
  // [cite: 81]
  if(buttons & PSB_CIRCLE)
  {
    mode = 3;
    reset_position = true;
  }
  
  // [cite: 82]
  if(buttons & PSB_CROSS)
  {
    mode = 4;
    reset_position = true;
  }

  // Кнопки START и SELECT не реализованы в 1 байте.
  // Вы можете переназначить их, например, на L1/R1, если добавите их.
  
  /* [cite: 86]
  if(ps2x.ButtonPressed(PSB_SELECT))
  {
    mode = 99;
  }
  */
  
  // Логика L1/R1/L2/R2 также требует добавления
  // ...
}


//***********************************************************************
// Leg IK Routine (БЕЗ ИЗМЕНЕНИЙ)
//***********************************************************************
void leg_IK(int leg_number,float X,float Y,float Z)
{
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH; [cite: 91]
  L3 = sqrt(sq(L0) + sq(Z)); [cite: 92]

  if((L3 < (TIBIA_LENGTH+FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH-FEMUR_LENGTH)))  
  {
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3))/(2*FEMUR_LENGTH*TIBIA_LENGTH)); [cite: 92]
    theta_tibia = phi_tibia*RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number]; [cite: 93]
    theta_tibia = constrain(theta_tibia,0.0,180.0);
  
    gamma_femur = atan2(Z,L0); [cite: 93]
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH))/(2*FEMUR_LENGTH*L3)); [cite: 94]
    theta_femur = (phi_femur + gamma_femur)*RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number]; [cite: 94]
    theta_femur = constrain(theta_femur,0.0,180.0);  [cite: 95]

    theta_coxa = atan2(X,Y)*RAD_TO_DEG + COXA_CAL[leg_number]; [cite: 95]
    
    switch(leg_number)
    {
      case 0:
        if(leg1_IK_control == true)
        {
          theta_coxa = theta_coxa + 45.0; [cite: 96]
          theta_coxa = constrain(theta_coxa,0.0,180.0);
          coxa1_servo.write(int(theta_coxa)); 
          femur1_servo.write(int(theta_femur)); 
          tibia1_servo.write(int(theta_tibia)); [cite: 97]
        }
        break;
      case 1:
        theta_coxa = theta_coxa + 90.0; [cite: 98]
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa2_servo.write(int(theta_coxa)); 
        femur2_servo.write(int(theta_femur)); 
        tibia2_servo.write(int(theta_tibia)); 
        break;
      case 2:
        theta_coxa = theta_coxa + 135.0; [cite: 100]
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa3_servo.write(int(theta_coxa)); 
        femur3_servo.write(int(theta_femur)); 
        tibia3_servo.write(int(theta_tibia)); 
        break;
      case 3:
        if(theta_coxa < 0)
          theta_coxa = theta_coxa + 225.0; [cite: 102]
        else
          theta_coxa = theta_coxa - 135.0; [cite: 103]
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa4_servo.write(int(theta_coxa)); 
        femur4_servo.write(180-int(theta_femur)); 
        tibia4_servo.write(180-int(theta_tibia)); [cite: 104]
        break;
      case 4:
        if(theta_coxa < 0)
          theta_coxa = theta_coxa + 270.0; [cite: 105]
        else
          theta_coxa = theta_coxa - 90.0; [cite: 106]
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa5_servo.write(int(theta_coxa)); 
        femur5_servo.write(180-int(theta_femur)); 
        tibia5_servo.write(180-int(theta_tibia)); [cite: 107]
        break;
      case 5:
        if(leg6_IK_control == true)
        {
          if(theta_coxa < 0)
            theta_coxa = theta_coxa + 315.0; [cite: 109]
          else
            theta_coxa = theta_coxa - 45.0; [cite: 110]
          theta_coxa = constrain(theta_coxa,0.0,180.0);
          coxa6_servo.write(int(theta_coxa)); 
          femur6_servo.write(180-int(theta_femur)); 
          tibia6_servo.write(180-int(theta_tibia)); [cite: 111]
        }
        break;
    }
  }
}

//***********************************************************************
// Tripod Gait
//***********************************************************************
void tripod_gait()
{
  // Получаем команды из BLE
  commandedX = map(g_ble_joystick_data[3], 0, 255, 127, -127); // RY
  commandedY = map(g_ble_joystick_data[2], 0, 255, -127, 127); // RX
  commandedR = map(g_ble_joystick_data[0], 0, 255, 127, -127); // LX
    
  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0); [cite: 114]
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tripod_case[leg_num])
      {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks); [cite: 115]
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks); [cite: 116]
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tripod_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = HOME_X[leg_num] + amplitudeX*cos(M_PI*tick/numTicks); [cite: 117]
          current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY*cos(M_PI*tick/numTicks); [cite: 118]
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tripod_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++; [cite: 119]
    else tick = 0;
  }
}

//***********************************************************************
// Wave Gait (логика походки без изменений, заменены вводы)
//***********************************************************************
void wave_gait()
{
  commandedX = map(g_ble_joystick_data[3], 0, 255, 127, -127); // RY [cite: 120]
  commandedY = map(g_ble_joystick_data[2], 0, 255, -127, 127); // RX [cite: 121]
  commandedR = map(g_ble_joystick_data[0], 0, 255, 127, -127); // LX [cite: 121]

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); [cite: 122]
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(wave_case[leg_num])
      {
        // [cite: 123-135] ... (логика case 1-6 без изменений)
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) wave_case[leg_num] = 6;
          break;
        case 2:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 1;
          break;
        case 3:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 2;
          break;
        case 4:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) 
            wave_case[leg_num] = 3;
          break;
        case 5:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 4;
          break;
        case 6:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 5;
          break;
      }
    }
    if(tick < numTicks-1) tick++; [cite: 135]
    else tick = 0;
  }
}

//***********************************************************************
// Ripple Gait (логика походки без изменений, заменены вводы)
//***********************************************************************
void ripple_gait()
{
  commandedX = map(g_ble_joystick_data[3], 0, 255, 127, -127); // RY [cite: 136]
  commandedY = map(g_ble_joystick_data[2], 0, 255, -127, 127); // RX [cite: 137]
  commandedR = map(g_ble_joystick_data[0], 0, 255, 127, -127); // LX [cite: 137]

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); [cite: 138]
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(ripple_case[leg_num])
      {
        // [cite: 139-151] ... (логика case 1-6 без изменений)
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/(numTicks*2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/(numTicks*2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/(numTicks*2));
          if(tick >= numTicks-1) ripple_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*(numTicks+tick)/(numTicks*2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*(numTicks+tick)/(numTicks*2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*(numTicks+tick)/(numTicks*2));
          if(tick >= numTicks-1) ripple_case[leg_num] = 3;
          break;
        case 3:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 4;
          break;
        case 4:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 5;
          break;
        case 5:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 6;
          break;
        case 6:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++; [cite: 151]
    else tick = 0;
  }  
}

//***********************************************************************
// Tetrapod Gait (логика походки без изменений, заменены вводы)
//***********************************************************************
void tetrapod_gait()
{
  commandedX = map(g_ble_joystick_data[3], 0, 255, 127, -127); // RY
  commandedY = map(g_ble_joystick_data[2], 0, 255, -127, 127); // RX
  commandedR = map(g_ble_joystick_data[0], 0, 255, 127, -127); // LX [cite: 153]
  
  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 3.0); [cite: 155]
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tetrapod_case[leg_num])
      {
        // [cite: 156-162] ... (логика case 1-3 без изменений)
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 3;
          break;
        case 3:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++; [cite: 162]
    else tick = 0;
  } 
}


//***********************************************************************
// Compute walking stride lengths (БЕЗ ИЗМЕНЕНИЙ)
//***********************************************************************
void compute_strides()
{
  strideX = 90*commandedX/127; [cite: 163]
  strideY = 90*commandedY/127; [cite: 164]
  strideR = 35*commandedR/127; [cite: 164]

  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));
  
  if(gait_speed == 0) duration = 1080; [cite: 165]
  else duration = 3240; [cite: 166]
}


//***********************************************************************
// Compute walking amplitudes (БЕЗ ИЗМЕНЕНИЙ)
//***********************************************************************
void compute_amplitudes()
{
  totalX = HOME_X[leg_num] + BODY_X[leg_num]; [cite: 167]
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num]; [cite: 168]

  rotOffsetX = totalY*sinRotZ + totalX*cosRotZ - totalX; [cite: 168]
  rotOffsetY = totalY*cosRotZ - totalX*sinRotZ - totalY; [cite: 169]

  amplitudeX = ((strideX + rotOffsetX)/2.0); [cite: 169]
  amplitudeY = ((strideY + rotOffsetY)/2.0); [cite: 170]
  amplitudeX = constrain(amplitudeX,-50,50);
  amplitudeY = constrain(amplitudeY,-50,50);
  
  if(abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) /4.0; [cite: 171]
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0; [cite: 172]
}
      

//***********************************************************************
// Body translate with controller (заменены вводы)
//***********************************************************************
void translate_control()
{
  // RY -> translateX
  translateX = map(g_ble_joystick_data[3], 0, 255, -2*TRAVEL, 2*TRAVEL); [cite: 173]
  for(leg_num=0; leg_num<6; leg_num++)
    current_X[leg_num] = HOME_X[leg_num] + translateX;
    
  // RX -> translateY
  translateY = map(g_ble_joystick_data[2], 0, 255, 2*TRAVEL, -2*TRAVEL); [cite: 174]
  for(leg_num=0; leg_num<6; leg_num++)
    current_Y[leg_num] = HOME_Y[leg_num] + translateY;

  // LY -> translateZ
  int translateZ_raw = g_ble_joystick_data[1]; [cite: 175]
  if(translateZ_raw > 127)
    translateZ = map(translateZ_raw, 128, 255, 0, TRAVEL); [cite: 176]
  else
    translateZ = map(translateZ_raw, 0, 127, -3*TRAVEL, 0); [cite: 176]
  
  for(leg_num=0; leg_num<6; leg_num++)
    current_Z[leg_num] = HOME_Z[leg_num] + translateZ; [cite: 177]
  
  // Логика захвата (capture_offsets) не изменена [cite: 178-181]
  if(capture_offsets == true)
  {
    for(leg_num=0; leg_num<6; leg_num++)
    {
      offset_X[leg_num] = offset_X[leg_num] + translateX;
      offset_Y[leg_num] = offset_Y[leg_num] + translateY;
      offset_Z[leg_num] = offset_Z[leg_num] + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }

  if(capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}


//***********************************************************************
// Body rotate with controller (заменены вводы)
//***********************************************************************
void rotate_control()
{
  // RX -> sinRotX/cosRotX
  sinRotX = sin((map(g_ble_joystick_data[2], 0, 255, A12DEG, -A12DEG))/1000000.0); [cite: 181]
  cosRotX = cos((map(g_ble_joystick_data[2], 0, 255, A12DEG, -A12DEG))/1000000.0); [cite: 182]
  // RY -> sinRotY/cosRotY
  sinRotY = sin((map(g_ble_joystick_data[3], 0, 255, A12DEG, -A12DEG))/1000000.0);
  cosRotY = cos((map(g_ble_joystick_data[3], 0, 255, A12DEG, -A12DEG))/1000000.0);
  // LX -> sinRotZ/cosRotZ
  sinRotZ = sin((map(g_ble_joystick_data[0], 0, 255, -A30DEG, A30DEG))/1000000.0);
  cosRotZ = cos((map(g_ble_joystick_data[0], 0, 255, -A30DEG, A30DEG))/1000000.0); [cite: 182]
  
  // LY -> translateZ
  int translateZ_raw = g_ble_joystick_data[1];
  if(translateZ_raw > 127)
    translateZ = map(translateZ_raw, 128, 255, 0, TRAVEL); [cite: 183]
  else
    translateZ = map(translateZ_raw, 0, 127, -3*TRAVEL, 0); [cite: 184]

  // Остальная логика без изменений [cite: 184-194]
  for(int leg_num=0; leg_num<6; leg_num++)
  {
    totalX = HOME_X[leg_num] + BODY_X[leg_num];
    totalY = HOME_Y[leg_num] + BODY_Y[leg_num];
    totalZ = HOME_Z[leg_num] + BODY_Z[leg_num];
    
    rotOffsetX =  totalX*cosRotY*cosRotZ + totalY*sinRotX*sinRotY*cosRotZ + totalY*cosRotX*sinRotZ - totalZ*cosRotX*sinRotY*cosRotZ + totalZ*sinRotX*sinRotZ - totalX;
    rotOffsetY = -totalX*cosRotY*sinRotZ - totalY*sinRotX*sinRotY*sinRotZ + totalY*cosRotX*cosRotZ + totalZ*cosRotX*sinRotY*sinRotZ + totalZ*sinRotX*cosRotZ - totalY;
    rotOffsetZ =  totalX*sinRotY         - totalY*sinRotX*cosRotY                                  + totalZ*cosRotX*cosRotY                                  - totalZ;
    
    current_X[leg_num] = HOME_X[leg_num] + rotOffsetX;
    current_Y[leg_num] = HOME_Y[leg_num] + rotOffsetY;
    current_Z[leg_num] = HOME_Z[leg_num] + rotOffsetZ + translateZ;
    
    if(capture_offsets == true)
    {
      offset_X[leg_num] = offset_X[leg_num] + rotOffsetX;
      offset_Y[leg_num] = offset_Y[leg_num] + rotOffsetY;
      offset_Z[leg_num] = offset_Z[leg_num] + rotOffsetZ + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }

  if(capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}


//***********************************************************************
// One leg lift mode (заменены вводы)
//***********************************************************************
void one_leg_lift()
{
  // [cite: 194-196] ... (логика leg1/leg6_IK_control без изменений)
  if(leg1_IK_control == true)
  {
    leg1_coxa  = coxa1_servo.read();
    leg1_femur = femur1_servo.read(); 
    leg1_tibia = tibia1_servo.read(); 
    leg1_IK_control = false;
  }

  if(leg6_IK_control == true)
  {
    leg6_coxa  = coxa6_servo.read();
    leg6_femur = femur6_servo.read(); 
    leg6_tibia = tibia6_servo.read(); 
    leg6_IK_control = false;
  }

  // RX -> Управление ногой 1
  temp = g_ble_joystick_data[2]; // RX [cite: 196]
  temp = map(temp, 0, 255, 45, -45);
  coxa1_servo.write(constrain(int(leg1_coxa+temp),45,135));

  // RY -> Управление ногой 1
  temp = g_ble_joystick_data[3]; // RY [cite: 197]
  if(temp < 117)
  {
    temp = map(temp,116,0,0,24); [cite: 198]
    femur1_servo.write(constrain(int(leg1_femur+temp),0,170));
    tibia1_servo.write(constrain(int(leg1_tibia+4*temp),0,170)); [cite: 199]
  }
  else
  {
    z_height_right = constrain(temp,140,255); [cite: 199]
    z_height_right = map(z_height_right,140,255,1,8); [cite: 200]
  }

  // LX -> Управление ногой 6
  temp = g_ble_joystick_data[0]; // LX [cite: 201]
  temp = map(temp, 0, 255, 45, -45);
  coxa6_servo.write(constrain(int(leg6_coxa+temp),45,135));
  
  // LY -> Управление ногой 6
  temp = g_ble_joystick_data[1]; // LY [cite: 202]
  if(temp < 117)
  {
    temp = map(temp,116,0,0,24); [cite: 202]
    femur6_servo.write(constrain(int(leg6_femur+temp),0,170));
    tibia6_servo.write(constrain(int(leg6_tibia+4*temp),0,170)); [cite: 203]
  }
  else
  {
    z_height_left = constrain(temp,140,255); [cite: 204]
    z_height_left = map(z_height_left,140,255,1,8); [cite: 204]
  }

  // Логика Z-height (без LED) [cite: 205-208]
  if(z_height_left>z_height_right) 
    z_height_right = z_height_left;
  
  // LED_Bar(...) - УДАЛЕНО

  if(capture_offsets == true)
  {
    step_height_multiplier = 1.0 + ((z_height_right - 1.0) / 3.0);
    capture_offsets = false;
  }
}


//***********************************************************************
// Set all servos to 90 degrees (БЕЗ ИЗМЕНЕНИЙ)
//***********************************************************************
void set_all_90()
{
  // [cite: 208-210]
  coxa1_servo.write(90+COXA_CAL[0]);
  femur1_servo.write(90+FEMUR_CAL[0]); 
  tibia1_servo.write(90+TIBIA_CAL[0]); 
  coxa2_servo.write(90+COXA_CAL[1]); 
  femur2_servo.write(90+FEMUR_CAL[1]); 
  tibia2_servo.write(90+TIBIA_CAL[1]); 
  coxa3_servo.write(90+COXA_CAL[2]); 
  femur3_servo.write(90+FEMUR_CAL[2]); 
  tibia3_servo.write(90+TIBIA_CAL[2]); 
  coxa4_servo.write(90+COXA_CAL[3]); 
  femur4_servo.write(90+FEMUR_CAL[3]); 
  tibia4_servo.write(90+TIBIA_CAL[3]); 
  coxa5_servo.write(90+COXA_CAL[4]); 
  femur5_servo.write(90+FEMUR_CAL[4]); 
  tibia5_servo.write(90+TIBIA_CAL[4]); 
  coxa6_servo.write(90+COXA_CAL[5]); 
  femur6_servo.write(90+FEMUR_CAL[5]); 
  tibia6_servo.write(90+TIBIA_CAL[5]);
}


//***********************************************************************
// Battery monitor routine (изменена пин и убран вывод LED)
//***********************************************************************
void battery_monitor()
{
  batt_voltage_sum = batt_voltage_sum - batt_voltage_array[batt_voltage_index]; [cite: 210]
  
  // Читаем с нового пина.
  // [!!!] ВАМ НУЖНО ОТРЕГУЛИРОВАТЬ map() под ваш делитель напряжения!
  // Этот map() откалиброван для 12.6V -> 4095 (макс. АЦП ESP32)
  batt_voltage_array[batt_voltage_index] = map(analogRead(BATT_VOLTAGE_PIN), 0, 4095, 0, 1260); // 0-12.6V
  
  batt_voltage_sum = batt_voltage_sum + batt_voltage_array[batt_voltage_index]; [cite: 212]
  batt_voltage_index = batt_voltage_index + 1;
  if(batt_voltage_index > 49) batt_voltage_index = 0; [cite: 212]
  
  batt_voltage = batt_voltage_sum / 50; [cite: 213]
  
  // Логика LED_Bar() УДАЛЕНА [cite: 214-215]
}


//***********************************************************************
// LED Bar Graph Routine (УДАЛЕНО)
//***********************************************************************
// void LED_Bar(int LED_color,int LED_count) { ... } [cite: 216-220]


//***********************************************************************
// Print Debug Data (БЕЗ ИЗМЕНЕНИЙ)
//***********************************************************************
void print_debug()
{
  // [cite: 220-226]
  // ... (весь код отладки остался, но закомментирован)

  currentTime = millis();
  Serial.print(currentTime-previousTime);
  Serial.print(",");
  Serial.print(float(batt_voltage)/100.0); 
  Serial.print("\n");
}