// Motor Driver Connection PINs
#define ENA_RIGHT 5   // PWM
#define IN1_RIGHT 10  // Dir Motor A
#define IN2_RIGHT 9   // Dir Motor A
#define ENA_LEFT 6    // PWM
#define IN3_LEFT 8    // Dir Motor B
#define IN4_LEFT 7    // Dir Motor B

// Millis Definition
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "000";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// Command
int right_wheel_cmd = 0; // 0-255
int left_wheel_cmd = 0;  // 0-255


void setup() {
  // Init Motor Driver Connection PINs
  pinMode(ENA_RIGHT, OUTPUT);
  pinMode(ENA_LEFT, OUTPUT);
  pinMode(IN1_RIGHT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);
  pinMode(IN3_LEFT, OUTPUT);
  pinMode(IN4_LEFT, OUTPUT);

  // Set Motor Rotation Direction
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
  digitalWrite(IN3_LEFT, HIGH);
  digitalWrite(IN4_LEFT, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left Wheel Motor
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(IN1_RIGHT, HIGH - digitalRead(IN1_RIGHT));
        digitalWrite(IN2_RIGHT, HIGH - digitalRead(IN2_RIGHT));
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(IN3_LEFT, HIGH - digitalRead(IN3_LEFT));
        digitalWrite(IN4_LEFT, HIGH - digitalRead(IN4_LEFT));
        is_left_wheel_forward = true;
      }
    }
    // Negative direction
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(IN1_RIGHT, HIGH - digitalRead(IN1_RIGHT));
        digitalWrite(IN2_RIGHT, HIGH - digitalRead(IN2_RIGHT));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(IN3_LEFT, HIGH - digitalRead(IN3_LEFT));
        digitalWrite(IN4_LEFT, HIGH - digitalRead(IN4_LEFT));
        is_left_wheel_forward = false;
      }
    }
    // Separator
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd = atoi(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd = atoi(value);
        is_cmd_complete = true;
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';
    }
    // Command Value
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Encoder
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    // Serial.print("Input\t: ");
    // Serial.print(right_wheel_cmd);
    // Serial.print(" | ");
    // Serial.println(left_wheel_cmd);

    if(right_wheel_cmd > 255)
    {
      right_wheel_cmd = 255;
    }
    else if(right_wheel_cmd < 0)
    {
      right_wheel_cmd = 0;
    }
    if(left_wheel_cmd > 255)
    {
      left_wheel_cmd = 255;
    }
    else if(left_wheel_cmd < 0)
    {
      left_wheel_cmd = 0;
    }

    // Serial.print("Output\t: ");
    // Serial.print(right_wheel_cmd);
    // Serial.print(" | ");
    // Serial.println(left_wheel_cmd);

    if(right_wheel_cmd !=0) digitalWrite(LED_BUILTIN, HIGH);
    else if(left_wheel_cmd !=0) digitalWrite(LED_BUILTIN, HIGH);
    else digitalWrite(LED_BUILTIN, 0);

    analogWrite(ENA_RIGHT, right_wheel_cmd);
    analogWrite(ENA_LEFT, left_wheel_cmd);
   
    last_millis = current_millis;
  }
}
