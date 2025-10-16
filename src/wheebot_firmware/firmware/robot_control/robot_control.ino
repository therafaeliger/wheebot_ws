#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dacAxisX;
Adafruit_MCP4725 dacAxisY;

#define ADDR_AXIS_X 0x60
#define ADDR_AXIS_Y 0x61
#define IN_AXIS_X A0
#define IN_AXIS_Y A2

int inAxisXVal = 0, inAxisYVal = 0;
int serAxisXVal = 0, serAxisYVal = 0;
int inAxisXVal_TEMP = 0, inAxisYVal_TEMP = 0;
int dacAxisXVal = 0, dacAxisYVal = 0;
bool emergency_activated = 0;

void setup()
{
  Wire.begin();
  dacAxisX.begin(ADDR_AXIS_X);
  dacAxisY.begin(ADDR_AXIS_Y);

  pinMode(IN_AXIS_X, INPUT);
  pinMode(IN_AXIS_Y, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("Robot System Ready!");
  Serial.println("Waiting for command ...");
}

void loop()
{
  // untuk kondisi
  inAxisXVal_TEMP = inAxisXVal;
  inAxisYVal_TEMP = inAxisYVal;

  // ambil input joystick
  inAxisXVal  = map(analogRead(IN_AXIS_X), 0, 1023, -255, 255);
  inAxisYVal = map(analogRead(IN_AXIS_Y), 0, 1023, -255, 255);
  // safety
  if(inAxisXVal >= -20 && inAxisXVal <= 20) inAxisXVal = 0;
  if(inAxisYVal >= -20 && inAxisYVal <= 20) inAxisYVal = 0;

  // Serial.print(inAxisXVal);
  // Serial.print(" | ");
  // Serial.println(inAxisYVal);

  // ambil input serial
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int values[2] = {0, 0};
    int i = 0;
    char *token = strtok((char*)input.c_str(), ",");
    while (token != NULL && i < 2) {
      values[i++] = constrain(atoi(token), -255, 255);
      token = strtok(NULL, ",");
    }

    if (i == 2) {
      serAxisXVal = values[0];
      serAxisYVal = values[1];
    }
    else
    {
      Serial.println("Invalid input!");
    }

    // Serial.print(serAxisXVal);
    // Serial.print(" | ");
    // Serial.println(serAxisYVal);
  }

  // PRIORITAS 1
  if(inAxisXVal != 0 || inAxisYVal !=0)
  {
    dacAxisXVal = map(inAxisXVal, -255, 255, 0, 4095);
    dacAxisYVal = map(inAxisYVal, -255, 255, 0, 4095);
  }
  else if((inAxisXVal == 0 || inAxisYVal == 0) && inAxisYVal_TEMP > 0)
  {
    emergency_activated = 1;
    // Serial.println("ON");
    digitalWrite(LED_BUILTIN, HIGH);

    dacAxisXVal = map(inAxisXVal, -255, 255, 0, 4095);
    dacAxisYVal = map(inAxisYVal, -255, 255, 0, 4095);
  }
  else if((inAxisXVal == 0 || inAxisYVal == 0) && inAxisYVal_TEMP < 0)
  {
    emergency_activated = 0;
    // Serial.println("OFF");
    digitalWrite(LED_BUILTIN, LOW);

    dacAxisXVal = map(inAxisXVal, -255, 255, 0, 4095);
    dacAxisYVal = map(inAxisYVal, -255, 255, 0, 4095);
  }
  else if(emergency_activated)
  {
    dacAxisXVal = map(0, -255, 255, 0, 4095);
    dacAxisYVal = map(0, -255, 255, 0, 4095);
  }
  else
  {
    dacAxisXVal = map(serAxisXVal, -255, 255, 0, 4095);
    dacAxisYVal = map(serAxisYVal, -255, 255, 0, 4095);
  }

  dacAxisX.setVoltage(dacAxisXVal, false);
  dacAxisY.setVoltage(dacAxisYVal, false);

  delay(10);
}
