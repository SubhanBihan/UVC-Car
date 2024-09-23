#include <Arduino.h>
#include <NewPing.h> // Library for ultrasonic sensor
#include <EEPROM.h>
#include <FastPID.h>

// Mode Switch and UV LEDs
#define MODE_SWITCH 13
#define LED_SWITCH 10
// Encoders
#define ENCPIN_LEFT 2
#define ENCPIN_RIGHT 3
// Ultrasonic Sensor
#define MAX_DISTANCE 200
#define TRIGGER_PIN 4 // Check if it's suitable
#define ECHO_PIN 5
// IR Sensors
/* #define IR_FRONT 4 */
#define IR_LEFT 11
#define IR_RIGHT 12
// DC Motors; MUST BE SET TO PWM PINS
#define MotorBp 7
#define MotorBn 6
/* #define MotorB 5 */
#define MotorAp 8
#define MotorAn 9
/* #define MotorA 10 */
// Extras
#define min_objwidth 5
#define max_objwidth_cm 40
#define NOMINAL_SPEED_PWM 200;
#define IR_RANGE_CM 2
#define SMALL_STEP 5
#define TURN_STEP 10
#define OBSTACLE_DIST_CM 8
#define CM_TO_COUNT_MULTIPLIER 12
#define NOMINAL_OBS_WIDTH_COUNT 30
#define NOMINAL_OBS_LENGTH_COUNT 100

volatile unsigned int enc_Left = 0;
volatile unsigned int enc_Right = 0;
unsigned int map_address = sizeof(unsigned int);
unsigned int normal_address;
bool mode = 0; // 0 for MAPPING, 1 for NORMAL

int just_in = 0;
int just_out = 0;
byte tol = 2;

unsigned int y = 0;
unsigned int prev_y = 0;
byte turn_type;  // 0, 1, 2, 3, 4, 5 for left, right, left-U, right-U, left-rev, right-rev
byte nominalDir; // 0 for left, 1 for right

// Function Declarations; Delete if compiling on Arduino IDE
void right_counter();
void left_counter();
void MoveForward(unsigned int);
void MoveReverse(unsigned int);
void SpinLeft(unsigned int);
void SpinRight(unsigned int);
void turn(byte);
int read_sensor(byte);
void save_y();
void gap_cover();
/* void wheel_drive(int, int);
void moveForward(int); */

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

float Kp = 0.6, Ki = 0.4, Kd = 0.0, Hz = 4.0;
int output_bits = 8;
int MotorSpeed_A = 200;
int MotorSpeed_B = 200;
int setpoint;
bool output_signed = false;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

void setup()
{
  Serial.begin(9600);

  // SWITCHES LEFT!
  pinMode(MotorBp, OUTPUT);
  pinMode(MotorBn, OUTPUT);
  pinMode(MotorAp, OUTPUT);
  pinMode(MotorAn, OUTPUT);
  /* pinMode(MotorB, OUTPUT);
  pinMode(MotorA, OUTPUT); */

  pinMode(ENCPIN_LEFT, INPUT_PULLUP);
  pinMode(ENCPIN_RIGHT, INPUT_PULLUP);

  /* pinMode(IR_FRONT, INPUT); */
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  cli(); // stop interrupts

  // set timer1 interrupt at 4Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0

  // set compare match register for 4hz increments
  OCR1A = 3905; // = (16*10^6) / (4*1024) - 1 (must be <65536)

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); // allow interrupts

  setpoint = 0;

  // Turn off motors - Initial state
  digitalWrite(MotorAp, LOW);
  digitalWrite(MotorAn, LOW);
  digitalWrite(MotorBp, LOW);
  digitalWrite(MotorBn, LOW);

  /*   // Set initial speed of Motors
    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed); */

  attachInterrupt(digitalPinToInterrupt(ENCPIN_LEFT), left_counter, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCPIN_RIGHT), right_counter, RISING);

  if (mode == 0)
  {
    // Find nom dir
    if (read_sensor(1) == LOW)
      nominalDir = 1;
    else if (read_sensor(0) == LOW)
      nominalDir = 0;

    int eepromSize = EEPROM.length();

    for (int address = 0; address < eepromSize; address++)
    {
      EEPROM.write(address, 0x00); // Write all zeros to the current address
    }
  }
}

void loop()
{
  // MAPPING
  if (mode == 0)
  {
    static bool room_left = 0;

    if (read_sensor(3) > IR_RANGE_CM)
    {
      MoveForward(SMALL_STEP);

      if (read_sensor(nominalDir) == LOW)
        room_left = 1;
    }

    if (read_sensor(3) <= IR_RANGE_CM)
    {
      if (read_sensor(nominalDir) == HIGH)
      {
        if (room_left == 0)
        { // True END
          save_y();
          EEPROM.put(0, map_address);
          return;
        }
        else
        { // False END
          turn(nominalDir + 4);
          nominalDir = !nominalDir;

          while (read_sensor(nominalDir) == HIGH)
          {
            MoveForward(SMALL_STEP);
          }

          turn(nominalDir);
          MoveForward(TURN_STEP);
          turn(!nominalDir);

          room_left = 0;
        }
      }
      else
      {
        turn(nominalDir + 2);     // U-turn in nominal direction
        nominalDir = !nominalDir; // Reverse the nominal direction at the end if a U-turn
        room_left = 0;
      }
    }

    if (enc_Left > prev_y + min_objwidth + tol && prev_y > 2 * SMALL_STEP && read_sensor(!nominalDir) == LOW && just_in <= 0 && just_out <= 0)
    {
      gap_cover();
    }
  }

  // NORMAL (OPERATION) MODE
  if (mode == 1)
  {
    enc_Left = 0;
    enc_Right = 0;
    unsigned int stop_address;
    EEPROM.get(0, stop_address);

    if (stop_address == 0)
      return;

    for (normal_address = sizeof(unsigned int); normal_address < stop_address;)
    {
      EEPROM.get(normal_address, y);
      while (enc_Left < y)
      {
        MoveForward(SMALL_STEP);

        // OBSTACLE AVOIDANCE
        if (read_sensor(3) <= OBSTACLE_DIST_CM && y - enc_Left > OBSTACLE_DIST_CM * CM_TO_COUNT_MULTIPLIER + tol)
        {
          // Take care of LED switching, buzzer, delay etc in required places
          unsigned int count_covered = enc_Left;
          unsigned int width_count = 0;
          unsigned int length_count = 0;
          byte avoid_dir = 0;

          do
          {
            turn(avoid_dir);

            // Code to move forward by nominal obstacle width count.
            MoveForward(NOMINAL_OBS_WIDTH_COUNT);
            width_count += enc_Left;

            turn(!avoid_dir);
          } while (read_sensor(3) <= OBSTACLE_DIST_CM);

          // Code to move forward by obstacle detection distance + nominal obstacle length in count.
          MoveForward(OBSTACLE_DIST_CM * CM_TO_COUNT_MULTIPLIER + NOMINAL_OBS_LENGTH_COUNT);
          length_count += enc_Left;

          while (1)
          {
            turn(!avoid_dir);

            if (read_sensor(3) * CM_TO_COUNT_MULTIPLIER > width_count)
              break;

            turn(avoid_dir);

            // Code to move forward by nominal obstacle length count
            MoveForward(NOMINAL_OBS_LENGTH_COUNT);
            length_count += enc_Left;
          }

          MoveForward(width_count);
          turn(avoid_dir);
          enc_Left = count_covered + length_count;
          enc_Right = count_covered + length_count;
        }
      }

      normal_address += sizeof(unsigned int);

      if (normal_address < stop_address)
      {
        EEPROM.get(normal_address, turn_type);
        turn(turn_type);
        normal_address += sizeof(byte);
      }
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  static int pError;
  pError = 0;
  pError = enc_Left - enc_Right;
  uint8_t output = myPID.step(setpoint, pError);

  MotorSpeed_A = NOMINAL_SPEED_PWM - output;
  MotorSpeed_B = NOMINAL_SPEED_PWM + output;

  /* analogWrite(enA, (motorSpeed - output));
  analogWrite(enB, (motorSpeed + output)); */
}

void save_y()
{
  prev_y = y;
  y = enc_Left;

  // Code to save the current y in a file
  EEPROM.put(map_address, y);
  map_address += sizeof(unsigned int);
}

void gap_cover()
{
  just_in = 2;
  unsigned int width_to_cover_index;
  unsigned int i = 0;
  byte pos;

  turn(!nominalDir);

  while (1)
  {
    MoveForward(TURN_STEP);
    i++;

    if (read_sensor(!nominalDir) == LOW) // LEDGE
    {
      turn(!nominalDir);

      if (read_sensor(3) <= max_objwidth_cm)
      {
        while (read_sensor(3) > IR_RANGE_CM)
        {
          MoveForward(SMALL_STEP);
        }

        turn(nominalDir);
        /* i++; */
      }
      else
      {
        // Code to move back into position; also take care of index
        // THIS PART IS VERY CRUCIAL AS IT'S THE END OF WIDTH
        turn(!nominalDir);
        MoveForward(TURN_STEP);
        i--;
        turn(nominalDir + 4);

        pos = 1;
        break;
      }
    }
    else if (read_sensor(3) <= IR_RANGE_CM) // CLIFF
    {
      turn(nominalDir);

      while (read_sensor(!nominalDir) == HIGH)
      {
        MoveForward(SMALL_STEP);

        if (read_sensor(3) <= IR_RANGE_CM)
          break;
      }

      if (read_sensor(3) > IR_RANGE_CM && read_sensor(!nominalDir) == LOW)
      {
        turn(!nominalDir);
        /* i++; */
        MoveForward(TURN_STEP);
        i++;
      }
      else
      {
        pos = 2;
        break;
      }
    }
  }

  width_to_cover_index = i;

  if (pos == 1)
  {
    turn(nominalDir);
  }
  else if (pos == 2)
  {
    turn(nominalDir + 4);
    nominalDir = !nominalDir;
  }

  bool room_left = 0;
  for (unsigned int j = 0; j <= width_to_cover_index + 1;)
  {
    if (read_sensor(3) > IR_RANGE_CM)
    {
      MoveForward(SMALL_STEP);

      if (read_sensor(nominalDir) == LOW)
        room_left = 1;
    }

    if (read_sensor(3) <= IR_RANGE_CM)
    {
      if (read_sensor(nominalDir) == HIGH && room_left == 1)
      {
        // False END
        turn(nominalDir + 4);
        nominalDir = !nominalDir;

        while (read_sensor(nominalDir) == HIGH)
        {
          MoveForward(SMALL_STEP);
        }

        turn(nominalDir);
        MoveForward(TURN_STEP);
        j++;
        turn(!nominalDir);

        room_left = 0;
      }
      else
      {
        turn(nominalDir + 2); // U-turn in nominal direction
        j++;
        just_in--;
        nominalDir = !nominalDir; // Reverse the nominal direction at the end if a U-turn
        room_left = 0;
      }
    }

    if (enc_Left > prev_y + min_objwidth + tol && prev_y > 2 * SMALL_STEP && read_sensor(!nominalDir) == LOW && just_in <= 0 && just_out <= 0)
    {
      gap_cover();
    }
  }

  just_out = 2;
}

int read_sensor(byte dir)
{
  switch (dir)
  {
  case 0:
    return digitalRead(IR_LEFT);

  case 1:
    return digitalRead(IR_RIGHT);

    /* case 2:
      return digitalRead(IR_FRONT); */

  case 3:
    return sonar.ping_cm();
  }
}

void left_counter()
{
  enc_Left++;
}

void right_counter()
{
  enc_Right++;
}

/* void moveForward(int time)
{
  wheel_drive(200, 200);
  delay(time);
} */

void turn(byte type)
{
  if (mode == 0)
  {
    save_y();

    turn_type = type;
    EEPROM.put(map_address, type);
    map_address += sizeof(byte);
  }

  enc_Left = 0;
  enc_Right = 0;
  // This might or might not be beneficial - Check!

  MotorSpeed_A = NOMINAL_SPEED_PWM;
  MotorSpeed_B = NOMINAL_SPEED_PWM;

  // if(*ind >= 0) (*ind)++;

  switch (type)
  {
  case 0: // Left turn
    SpinLeft(48);
    break;

  case 1: // Right turn
    SpinRight(48);
    break;

  case 2: // Left U turn; U turn = dir + 2
    SpinLeft(48);
    MoveForward(TURN_STEP);
    SpinLeft(48);

    just_out--;
    break;

  case 3: // Right U turn
    SpinRight(48);
    MoveForward(TURN_STEP);
    SpinRight(48);

    just_out--;
    break;

  case 4: // Left reverse; Reverse = dir + 4
    SpinLeft(96);
    break;

  case 5: // Right reverse
    SpinRight(96);
    break;
  }

  enc_Left = 0;
  enc_Right = 0;
  MotorSpeed_A = NOMINAL_SPEED_PWM;
  MotorSpeed_B = NOMINAL_SPEED_PWM;
}

void MoveForward(unsigned int steps)
{
  // Set Motor A forward
  digitalWrite(MotorAn, LOW);

  // Set Motor B forward
  digitalWrite(MotorBn, LOW);

  // Go forward until step value is reached
  while (steps > enc_Left && steps > enc_Right)
  {
    analogWrite(MotorAp, MotorSpeed_A);
    analogWrite(MotorBp, MotorSpeed_B);
    delay(50);
  }

  // Stop when done
  digitalWrite(MotorAp, LOW);
  digitalWrite(MotorAn, LOW);
  digitalWrite(MotorBp, LOW);
  digitalWrite(MotorBn, LOW);
}

// Function to Move in Reverse
void MoveReverse(unsigned int steps)
{
  // Set Motor A forward
  digitalWrite(MotorAp, LOW);

  // Set Motor B forward
  digitalWrite(MotorBp, LOW);

  // Go forward until step value is reached
  while (steps > enc_Left && steps > enc_Right)
  {
    analogWrite(MotorAn, MotorSpeed_A);
    analogWrite(MotorBn, MotorSpeed_B);
    delay(50);
  }

  // Stop when done
  digitalWrite(MotorAp, LOW);
  digitalWrite(MotorAn, LOW);
  digitalWrite(MotorBp, LOW);
  digitalWrite(MotorBn, LOW);
}

// Function to Spin Right
void SpinRight(unsigned int steps)
{
  // Set Motor A forward
  digitalWrite(MotorAp, LOW);

  // Set Motor B forward
  digitalWrite(MotorBn, LOW);

  // Go forward until step value is reached
  while (steps > enc_Left && steps > enc_Right)
  {
    analogWrite(MotorAn, MotorSpeed_A);
    analogWrite(MotorBp, MotorSpeed_B);
    delay(50);
  }

  // Stop when done
  digitalWrite(MotorAp, LOW);
  digitalWrite(MotorAn, LOW);
  digitalWrite(MotorBp, LOW);
  digitalWrite(MotorBn, LOW);
}

// Function to Spin Left
void SpinLeft(unsigned int steps)
{
  // Set Motor A forward
  digitalWrite(MotorAn, LOW);

  // Set Motor B forward
  digitalWrite(MotorBp, LOW);

  // Go forward until step value is reached
  while (steps > enc_Left && steps > enc_Right)
  {
    analogWrite(MotorAp, MotorSpeed_A);
    analogWrite(MotorBn, MotorSpeed_B);
    delay(50);
  }

  // Stop when done
  digitalWrite(MotorAp, LOW);
  digitalWrite(MotorAn, LOW);
  digitalWrite(MotorBp, LOW);
  digitalWrite(MotorBn, LOW);
}