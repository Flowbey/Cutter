#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <RotaryEncoder.h>
#include <Switch.h>

#define DEBUG1

// RotaryEncoder
#define ENCODER_PIN_A A2
#define ENCODER_PIN_B A1

// Switch
#define SWITCH_PIN A3

// Stepper 1
#define DIR_PIN_1 6  // Direction   Neue Platine 4
#define STEP_PIN_1 7 // Step        Neue Platine 5
#define EN_PIN_1 8   // Enable
#define ENDSTOP_1 A0 // Endstop
//#define ENDSTOP_1 A6 // Endstop

// Stepper2
#define DIR_PIN_2 4  // Direction   Neue Platine 6
#define STEP_PIN_2 5 // Step        Neue Platine 7
#define EN_PIN_2 8   // Enable
// #define ENDSTOP_2          A6 // Endstop

// TMC Adress
#define DRIVER_ADDRESS_1 0 // 0b00 for null  TMC2209 driverGroup1 address according to MS1 and MS2
#define DRIVER_ADDRESS_2 1 // 0b00 for null  TMC2209 driverGroup2 address according to MS1 and MS2
#define SW_RX 9            // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX 10           // TMC2208/TMC2224 SoftwareSerial transmit pin
// #define SERIAL_PORT mySerial // TMC2208/TMC2224 HardwareSerial port
#define R_SENSE 0.11f // Match to your Drive

// Define Global Val
int Sensor1 = 0;
int moveLength = 4000;
unsigned long timer1 = 0;
unsigned long timerCicle = 0;
bool cutterFlag = 0;
bool forearmFlag = 0;
int direction = 0;
uint8_t menuPos = 1;
int oldPosition = 0;
int newPosition = 0;
int feederFactor = 60; // 48 * 4.33 Umfang zu

// Declare Function
void endstopTest();
// void cutting();
// void feeding();
void error();
void pause();
int checkMovePositionEncoder();
void menuRefresh();
void checkEncoderButton();
void moveMenu(int);
void cuttingChose();
void cutting(int quantity, int length);

// Create TMC Object
TMC2209Stepper driverGroup1(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS_1);
TMC2209Stepper driverGroup2(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS_2);

// Create Stepper Object
AccelStepper cutterStepper = AccelStepper(cutterStepper.DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper feederStepper = AccelStepper(feederStepper.DRIVER, STEP_PIN_2, DIR_PIN_2);

// Create Encoder Button Object
Switch encoderButton = Switch(SWITCH_PIN);

// Create Rotary Encoder Object
RotaryEncoder encoder(ENCODER_PIN_A, ENCODER_PIN_B, RotaryEncoder::LatchMode::FOUR3);

// Create Display Object
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
  }
  Serial.print("Begin")                                                                                                                                                                      ;
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  pinMode(ENDSTOP_1, INPUT);
  // pinMode(ENDSTOP_2, INPUT_PULLUP);

  u8x8.setCursor(0, 1);
  u8x8.print("                ");
  u8x8.setCursor(0, 2);
  u8x8.print("  Cutting Tool  ");
  u8x8.setCursor(0, 4);
  u8x8.print("  Version 1.1   ");
  delay(2000);
  u8x8.clear();
  //*************
  // driverGroup1       Cutter            // Base and Underarm  // Stepper1 and Stepper4
  //************
  driverGroup1.begin(); // Initiate pins and registeries
  driverGroup1.toff(1);
  // driverGroup1.rms_current(2500);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driverGroup1.en_spreadCycle(true);
  driverGroup1.microsteps(8);
  // driverGroup1.pwm_autoscale(true);     // Needed for stealthChop
  driverGroup1.irun(31);
  driverGroup1.ihold(1);

  //*************
  // driverGroup2        feeder           // Forearm   // Stepper 2 and Stepper 3
  //************
  driverGroup2.begin(); // Initiate pins and registeries
  driverGroup2.toff(1);
  driverGroup2.rms_current(1500); // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  // driverGroup2.en_spreadCycle(true);
  driverGroup2.microsteps(8);
  // driverGroup2.pwm_autoscale(true);     // Needed for stealthChop
  driverGroup2.irun(31);
  driverGroup2.ihold(2);

  // stepper1 cutter
  cutterStepper.setEnablePin(EN_PIN_1);
  cutterStepper.setMaxSpeed(1600);     // 100mm/s @ 80 steps/mm
  cutterStepper.setAcceleration(1500); // 2000mm/s^2
  cutterStepper.setPinsInverted(true, false, true);
  cutterStepper.enableOutputs();
  cutterStepper.setSpeed(1600);
  cutterStepper.setCurrentPosition(0);
  // cutterStepper.move(1000);

  // stepper2 feeder
  feederStepper.setEnablePin(EN_PIN_2);
  feederStepper.setMaxSpeed(1600);    // 100mm/s @ 80 steps/mm
  feederStepper.setAcceleration(900); // 2000mm/s^2
  feederStepper.setPinsInverted(false, false, true);
  feederStepper.enableOutputs();
  feederStepper.setCurrentPosition(0);
  feederStepper.setSpeed(1600);

  menuRefresh();
}

void loop()
{
  encoder.tick();
  moveMenu(int(encoder.getDirection()));
  checkEncoderButton();

  // cutting();
  // feeding();
  //endstopTest();
  // delay(500);
}

void endstopTest()
{
  Serial.print("ENDSTOP_1: ");
  Serial.print(analogRead(ENDSTOP_1));
  delay(100);
  Serial.print("    ENDSTOP_1: ");
  Serial.println(digitalRead(ENDSTOP_1));
}

/*
void feeding(){
feederStepper.runToNewPosition(150);
feederStepper.setCurrentPosition(0);
}
*/

int checkMovePositionEncoder()
{
  newPosition = encoder.getPosition();
  if (newPosition >= (oldPosition))
  {
    oldPosition = newPosition;
    return -1;
  }
  if (newPosition <= (oldPosition))
  {
    oldPosition = newPosition;
    return 1;
  }
  return 0;
}

void menuRefresh()
{
  switch (menuPos)
  {
  case 1:
    u8x8.setCursor(0, 1);
    u8x8.print("                ");
    u8x8.setCursor(0, 2);
    u8x8.print(">1. Schneiden   ");
    u8x8.setCursor(0, 3);
    u8x8.print(" 2. Einstellung ");
    break;

  case 2:
    u8x8.setCursor(0, 1);
    u8x8.print(" 1. Schneiden   ");
    u8x8.setCursor(0, 2);
    u8x8.print(">2. Einstellung ");
    u8x8.setCursor(0, 3);
    u8x8.print("                ");
    break;

  case 21:
    u8x8.setCursor(0, 1);
    u8x8.print("                ");
    u8x8.setCursor(0, 2);
    u8x8.print(">1 Geschwindig. ");
    u8x8.setCursor(0, 3);
    u8x8.print(" 2 Beschleunig. ");
    break;

  case 22:
    u8x8.setCursor(0, 1);
    u8x8.print(" 1 Geschwindig. ");
    u8x8.setCursor(0, 2);
    u8x8.print(">2 Beschleunig. ");
    u8x8.setCursor(0, 3);
    u8x8.print(" 3 Zurueck      ");
    break;

  case 23:
    u8x8.setCursor(0, 1);
    u8x8.print(" 2 Beschleunig. ");
    u8x8.setCursor(0, 2);
    u8x8.print(">3 Zurueck      ");
    u8x8.setCursor(0, 3);
    u8x8.print("                ");
    break;

  default:
    break;
  }
}

void checkEncoderButton()
{
  encoderButton.poll();
  if (encoderButton.pushed())
  {
    switch (menuPos)
    {
    case 1:
      menuRefresh();
#ifdef DEBUG1
      Serial.println("Schneiden geklickt");
#endif
      cuttingChose();
      menuRefresh();
      break;

    case 2:
      menuRefresh();
#ifdef DEBUG1
      Serial.println("Einstellung geklickt");
#endif
      menuPos = 21;
      menuRefresh();
      break;

    case 21:
      menuRefresh();
#ifdef DEBUG1
      Serial.println("Geschwindigkeit geklickt");
#endif
      // calibrating();
      menuRefresh();
      break;

    case 22:
      menuRefresh();
#ifdef DEBUG1
      Serial.println("Beschleunigung geklickt");
#endif
      // setIntervall();
      menuRefresh();
      break;

    case 23:
      menuRefresh();
#ifdef DEBUG1
      Serial.println("Zurueck geklickt");
#endif
      // setIntervall();
      menuPos = 1;
      menuRefresh();
      break;

    default:
      break;
    }
  }
}

void moveMenu(int direction)
{
  if (direction > 0)
  {
    if (menuPos != 2 && menuPos != 23)
    { // Do not move out of the Menu
      menuPos++;
      menuRefresh();
#ifdef DEBUG1
      Serial.print("Menu Pos erhöt: ");
      Serial.println(menuPos);
#endif
    }
  }
  if (direction < 0)
  {
    if (menuPos != 1 && menuPos != 21)
    { // Do not move out of the Menu
      menuPos--;
      menuRefresh();
#ifdef DEBUG1
      Serial.print("Menu Pos reduziert: ");
      Serial.println(menuPos);
#endif
    }
  }
}

void cuttingChose()
{
  int anzahlStuecke = 1;
  int laengeStuecke = 30;

  u8x8.setCursor(0, 2);
  u8x8.print(" Anzahl Stuecke ");
  u8x8.setCursor(0, 3);
  u8x8.print(" Anzahl =       ");
  u8x8.setCursor(11, 3);
  u8x8.print(anzahlStuecke);
  u8x8.setCursor(0, 4);
  u8x8.print("                ");
  // u8x8.setCursor(0, 3);
  // u8x8.print(" Weiter =>      ");
  do
  {
    encoder.tick();
    encoderButton.poll();
    direction = int(encoder.getDirection());
    if (direction == 1)
    {
      anzahlStuecke += 1;
      u8x8.setCursor(11, 3);
      u8x8.print("   ");
      u8x8.setCursor(11, 3);
      u8x8.print(anzahlStuecke);
    }
    if (direction == -1)
    {
      anzahlStuecke -= 1;
      u8x8.setCursor(11, 3);
      u8x8.print("   ");
      u8x8.setCursor(11, 3);
      u8x8.print(anzahlStuecke);
    }
  } while (encoderButton.pushed() == false);
  u8x8.setCursor(0, 2);
  u8x8.print(" Laenge in mm   ");
  u8x8.setCursor(0, 3);
  u8x8.print(" Laenge =       ");
  u8x8.setCursor(11, 3);
  u8x8.print(laengeStuecke);
  u8x8.setCursor(0, 4);
  u8x8.print("                ");
  // u8x8.setCursor(0, 3);
  // u8x8.print(" Weiter =>      ");
  do
  {
    encoder.tick();
    encoderButton.poll();
    direction = int(encoder.getDirection());
    if (direction == 1)
    {
      laengeStuecke += 1;
      u8x8.setCursor(11, 3);
      u8x8.print("   ");
      u8x8.setCursor(11, 3);
      u8x8.print(laengeStuecke);
    }
    if (direction == -1)
    {
      laengeStuecke -= 1;
      u8x8.setCursor(11, 3);
      u8x8.print("   ");
      u8x8.setCursor(11, 3);
      u8x8.print(laengeStuecke);
    }
  } while (encoderButton.pushed() == false);
  cutting(anzahlStuecke, laengeStuecke);
}

void cutting(int quantity, int lenght)
{
  // cutterStepper.setSpeed(1600);

  cutterStepper.setSpeed(1600);
  while (analogRead(ENDSTOP_1) < 500)
  {
    cutterStepper.runSpeed();
  }
  cutterStepper.setCurrentPosition(0);
  cutterStepper.runToNewPosition(-2500);
  if (analogRead(ENDSTOP_1) > 500)
  {
    error();
  }
  u8x8.clear();
  for (int i = 0; i < quantity; i++)
  {

    u8x8.setCursor(1, 2);
    u8x8.print("Stueck ");
    // u8x8.setCursor(8, 1);
    u8x8.print(i + 1);
    u8x8.print(" / ");
    u8x8.print(quantity);

    feederStepper.setCurrentPosition(0);
    feederStepper.runToNewPosition(lenght * feederFactor);

    cutterStepper.setSpeed(1600);
    while (analogRead(ENDSTOP_1) < 500)
    {
      cutterStepper.runSpeed();
    }

    if (i < quantity - 1)
    {
      cutterStepper.setCurrentPosition(0);
      cutterStepper.runToNewPosition(-2500);
      if (analogRead(ENDSTOP_1) > 500)
      {
        error();
      }
    }
    encoderButton.poll();
    if (encoderButton.pushed() == true)
    {
      pause();
    }
  }
  u8x8.clear();
  u8x8.setCursor(0, 2);
  u8x8.print(quantity);
  u8x8.setCursor(3, 2);
  u8x8.print("Stk. fertig");
  u8x8.setCursor(0, 4);
  u8x8.print("Weiter mit Knopf");
  do
  {
    encoderButton.poll();
  } while (encoderButton.pushed() == false);
  u8x8.clear();
}

void error()
{
  // u8x8.setCursor(0, 2);
  // u8x8.print(quantity);
  u8x8.clear();
  u8x8.setCursor(0, 2);
  u8x8.print("Sensor Fehler");
  u8x8.setCursor(0, 3);
  u8x8.print("Fehler loesen!");
  u8x8.setCursor(0, 4);
  u8x8.print("Weiter mit Knopf");
  do
  {
    encoderButton.poll();
  } while (encoderButton.pushed() == false);
  if (analogRead(ENDSTOP_1) > 500)
  {
    error();
  }
  u8x8.clear();
}

void pause()
{
  // u8x8.setCursor(0, 2);
  // u8x8.print(quantity);
  u8x8.clear();
  u8x8.setCursor(0, 2);
  u8x8.print("Pause");
  u8x8.setCursor(0, 4);
  u8x8.print("Weiter mit Knopf");
  do
  {
    encoderButton.poll();
  } while (encoderButton.pushed() == false);
  delay(1000);
  u8x8.clear();
}