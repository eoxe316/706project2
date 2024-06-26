/**INITIALISATION OF VARIABLES*/
#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>

#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

// Serial Data Pins
#define BLUETOOTH_RX 10
#define BLUETOOTH_TX 11

// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

// Board measurements
#define BOARD_WIDTH 150
#define BOARD_LENGTH 1990

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

/*********STATE MACHINES************/
//Main state machine
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

enum FIND_LIGHT_STATE {
    TURNING,
    FOUND_LIGHT
};

FIND_LIGHT_STATE light_state = TURNING;


/*******************COMPONENT SET-UP**********************/
/***WHEEL MOTORS***/
const byte left_front = 47;
const byte left_rear = 46;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29

int speed_val = 350;
double speed_change;


/***IRS***/
//IR Equation Variables - MAY NEED TO UPDATE
double MR1coeff = 19.000;
double MR1power = -0.94;

double MR2coeff = 20.000;
double MR2power = -0.97;

double LR1coeff = 325.000;
double LR1power = -1.33;

double LR3coeff = 540.000;
double LR3power = -1.42;

//IR Sensor working variables
double MR1mm, MR2mm, LR1mm, LR3mm;
double MR1mm_reading, MR2mm_reading, LR1mm_reading, LR3mm_reading;

int MR1pin = A5;
int MR2pin = A6;
int LR1pin = A4;
int LR3pin = A7;


/***ULTRASONIC***/
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
const unsigned int MAX_DIST = 23200;

double sonar_cm = 0;
double cm = 0;
int sonar_MA_n = 20;

//Sonar Kalman
double sensor_noise_sonar = 10;
double process_noise_sonar = 1;
double sonar_variance = 0;



/***GYRO***/
int gyroPin = A2;
int gyroVal = 0;

double gyroZeroVoltage = 500;
double gyroRate = 0;
double gyroAngleChange = 0;
double gyroAngle = 0;
double gyro_average;
double gyroTime;

//Gyro Kalman
double prev_val_gyro = 0;
double last_var_gyro = 999;

double sensor_noise_gyro = 8;
double process_noise_gyro = 1;


/***SERVO***/
Servo turret_motor;


/***PHOTO TRANSISTORS***/
double photo1, photo2, photo3, photo4;
double photo_reading1, photo_reading2, photo_reading3, photo_reading4;
bool photo_light1, photo_light2, photo_light3, photo_light4;

//Pins
int photo_pin1 = A12;
int photo_pin2 = A13;
int photo_pin3 = A14;
int photo_pin4 = A15;

//Thresholds for deciding if light exists
double photo_thresh1 = 1000;
double photo_thresh2 = 1000;
double photo_thresh3 = 1000;
double photo_thresh4 = 1000;


void setup(void)
{
  BluetoothSerial.begin(115200);
  BluetoothSerial.println("");
  BluetoothSerial.println("");
  BluetoothSerial.println("");
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialise IR sensor pins
  pinMode(MR1pin, INPUT);
  pinMode(MR2pin, INPUT);
  pinMode(LR1pin, INPUT);
  pinMode(LR3pin, INPUT);

  //Initialise photo transistor sensor pins
  pinMode(photo_pin1, INPUT);
  pinMode(photo_pin2, INPUT);
  pinMode(photo_pin3, INPUT);
  pinMode(photo_pin4, INPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  //Initilaise Gyro Pins
  pinMode(gyroPin, INPUT);
  pinMode(A14, INPUT);

  delay(1000); //settling time but noT really needed
}


/*******************SUPER LOOP**********************/
void loop(void) //main loop
{
    static STATE machine_state = INITIALISING;
    switch (machine_state) {
      case INITIALISING:
        machine_state = initialising();
        break;
      case RUNNING:
        machine_state = running();
        break;
      case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
        machine_state = stopped();
        break;
    };
    Sonar();
    Gyro();
    delay(10);
}

STATE initialising() {
  BluetoothSerial.println("INITIALISING....");

  enable_motors();

  double sum1 = 0;
  for (int i = 0; i < 200; i++)
  { // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
    gyroVal = analogRead(gyroPin);
    // gyroVal = constrain(analogRead(gyroPin), 500, 530);
    delay(10);
    sum1 += gyroVal;
    // BluetoothSerial.println(gyroVal);
  }
  gyroZeroVoltage = sum1 / 200; // average the sum as the zero drifting

  SonarCheck(90); //Initialise sonar values
  initialise_IR();
  initialise_transistors();

  return RUNNING;
}

STATE running() {
  update_transistors();

  switch (light_state){
    case TURNING:
        cw();
        if(photo_light1 + photo_light2 + photo_light3 + photo_light4 >= 3){
            stop();
            light_state = FOUND_LIGHT;
        }
        break;

    case FOUND_LIGHT:
        stop();
        BluetoothSerial.print("FOUND LIGHT");
  }
  return RUNNING;
}


/*******************STOPPED**********************/
//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    BluetoothSerial.println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      BluetoothSerial.print("Lipo OK waiting of voltage Counter 10 < ");
      BluetoothSerial.println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        BluetoothSerial.println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}


/*******************GYRO FUNCTIONS**********************/
void Gyro()
{
    // convert the 0-1023 signal to 0-5v
    // BluetoothSerial.println("started gyro");
    double gyro_reading = (analogRead(gyroPin));

    gyroRate = (gyro_reading * 5.00) / 1023;

    // find the voltage offset the value of voltage when gyro is zero (still)
    gyroRate -= (gyroZeroVoltage * 5.00) / 1023;

    // read out voltage divided the gyro sensitivity to calculate the angular velocity
    double angularVelocity = (gyroRate / 0.007); // from Data Sheet, gyroSensitivity is 0.007 V/dps

    gyro_average = (1 ? angularVelocity : KalmanGyro(angularVelocity));
    // average_gyro(angularVelocity);

    // if the angular velocity is less than the threshold, ignore it
    if (gyro_average >= 2 || gyro_average <= -2)
    {
        gyroAngleChange = millis()-gyroTime;
        gyroAngleChange = 1000 / gyroAngleChange;
        gyroAngleChange = gyro_average/ gyroAngleChange;
        gyroAngle += gyroAngleChange;   
    }

    gyroTime = millis();

    // BluetoothSerial.print("Average ANGULAR VELOCITY");
    // BluetoothSerial.println(gyro_average);
    // BluetoothSerial.print("Gyro Angle:");
    // BluetoothSerial.println(gyroAngle);
    // BluetoothSerial.println("");
}

double KalmanGyro(double rawdata){   // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_var = last_var_gyro + process_noise_gyro; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise_gyro);
  a_post_est = prev_val_gyro + kalman_gain*(rawdata-prev_val_gyro);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var_gyro = a_post_var;
  prev_val_gyro = a_post_est;
  return a_post_est;
}

double ki_integral = 0;
#define GYRO_CONSTRAIN 100

void Drive(int speed_val, bool strafe)
{
    double u = 0;
    double e = 0;
  float kp = 0;
  float ki = 0;
  float kd = 0;

  (gyroAngleChange > 3) ? e = 0 : e = gyroAngleChange;

  u = constrain(kp*e + ki*ki_integral, -GYRO_CONSTRAIN, GYRO_CONSTRAIN);

  ki_integral += e;
  
  left_font_motor.writeMicroseconds(1500 + speed_val - u);
  right_rear_motor.writeMicroseconds(1500 - speed_val - u);

  if (strafe)
  {
    left_rear_motor.writeMicroseconds(1500 - speed_val - u);
    right_font_motor.writeMicroseconds(1500 + speed_val - u);
  }
  else
  {
    left_rear_motor.writeMicroseconds(1500 + speed_val - u);
    right_font_motor.writeMicroseconds(1500 - speed_val - u);
  }
}


/*******************SONAR FUNCTIONS**********************/
void Sonar()
{
    unsigned long t1, t2, pulse_width; 
    
    // Hold the trigger pin high for at least 10 us
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Wait for pulse on echo pin
    t1 = micros();
    while (digitalRead(ECHO_PIN) == 0 ) {
        t2 = micros();
        pulse_width = t2 - t1;
        if (pulse_width > (MAX_DIST + 1000)) {
            // BluetoothSerial.println("HC-SR04: NOT found");
            return;
        }
    }

    // Measure how long the echo pin was held high (pulse width)
    // Note: the micros() counter will overflow after ~70 min
    t1 = micros();
    while (digitalRead(ECHO_PIN) == 1)
    {
        t2 = micros();
        pulse_width = t2 - t1;
        if ( pulse_width > (MAX_DIST + 1000) ) {
            // BluetoothSerial.println("HC-SR04: Out of range");
            return;
        }
    }

    t2 = micros();

    pulse_width = t2 - t1;

    // Calculate distance in centimeters and inches. The constants
    // are found in the datasheet, and calculated from the assumed speed
    //of sound in air at sea level (~340 m/s).
    cm = pulse_width / 58.0;

    
    (sonar_cm == 0 ? sonar_cm = cm : sonar_cm = KalmanSonar(cm));

    // BluetoothSerial.print("Raw Sonar Reading");
    // BluetoothSerial.println(cm);
    // BluetoothSerial.print("Last sonar reading");
    // BluetoothSerial.println(sonar_cm);
    // BluetoothSerial.println("");
}

double SonarCheck(double angle_in)
{
    turret_motor.write(angle_in);
    delay(300);

    double sonar_sum = 0;
    int value_count = 0;

    for (int i = 0; i < sonar_MA_n; i++)
    {
        sonar_cm = 0;
        Sonar();
        if (sonar_cm < 10 || sonar_cm > 190){
          sonar_sum +=sonar_cm;
          value_count++;
        }
        delay(50);
    }
    (value_count > 0 ? sonar_cm = sonar_sum/value_count : sonar_cm = sonar_cm);
    return (sonar_cm);
}

double KalmanSonar(double rawdata){   // Kalman Filter
  if (rawdata < 20){ //If the value is absolutely outrageous, ignore it and use the last recorded value
    return sonar_cm;
  }
  else{
    rawdata = constrain(rawdata, sonar_cm - 20, sonar_cm + 20);
    double a_post_est, a_priori_var, a_post_var, kalman_gain;

    a_priori_var = sonar_variance + process_noise_sonar; 

    kalman_gain = a_priori_var/(a_priori_var+sensor_noise_sonar);
    a_post_est = sonar_cm + kalman_gain*(rawdata-sonar_cm);
    sonar_variance = (1 * kalman_gain)*a_priori_var;
    sonar_cm = rawdata;
    return a_post_est;
  }   
}

/*******************IR FUNCTIONS**********************/
void initialise_IR(){
double MR1sum, MR2sum, LR1sum, LR3sum;
int iterations = 20;

  for (int i = 0; i<=iterations; i++){
    read_IR_sensors();
    MR1sum = MR1mm_reading;
    MR2sum = MR2mm_reading;
    LR1sum = LR1mm_reading;
    LR3sum = LR3mm_reading;
    delay(5);
  }
  MR1mm = MR1sum/iterations;
  MR2mm = MR2sum/iterations;
  LR1mm = LR1sum/iterations;
  LR3mm = LR3sum/iterations;
}

double read_IR(double coefficient, double power, double sensor_reading){
  double sensor_mm;
  sensor_mm = coefficient *1000*(pow(sensor_reading, power));
  return sensor_mm;
}

void read_IR_sensors(){
  // BluetoothSerial.println("READ SENSOR START");
  MR1mm_reading = read_IR(MR1coeff, MR1power, analogRead(MR1pin));
  MR2mm_reading = read_IR(MR2coeff, MR2power, analogRead(MR2pin));
  LR1mm_reading = read_IR(LR1coeff, LR1power, analogRead(LR1pin));
  LR3mm_reading = read_IR(LR3coeff, LR3power, analogRead(LR3pin));
}

void filter_IR_reading(){
  //Average these to final value 
  double mrbuffer = 150;
  double lrbuffer = 500;
  MR1mm = constrain(MR1mm_reading, MR1mm-mrbuffer, MR1mm+mrbuffer);
  MR2mm = constrain(MR2mm_reading, MR2mm-mrbuffer, MR2mm+mrbuffer);
  LR1mm = constrain(LR1mm_reading, LR1mm-lrbuffer, LR1mm+lrbuffer);
  LR3mm = constrain(LR3mm_reading, LR3mm-lrbuffer, LR3mm+lrbuffer);
}


/*******************PHOTOTRANSISTOR FUNCTIONS**********************/
void read_transistors(){
  // BluetoothSerial.println("READ SENSOR START");
  photo_reading1 = analogRead(photo_pin1);
  photo_reading2 = analogRead(photo_pin2);
  photo_reading3 = analogRead(photo_pin3);
  photo_reading4 = analogRead(photo_pin4);
}

void update_transistors(){ 
  //Updates transistor state based on transistor readings
  read_transistors();

  //Filters transistor values
  photo1 = photo_reading1;
  photo2 = photo_reading2;
  photo3 = photo_reading3;
  photo4 = photo_reading4;

  //Updates boolean light value based of transistor threshold
  photo_light1 = transistor_on(photo1, photo_thresh1);
  photo_light2 = transistor_on(photo2, photo_thresh2);
  photo_light3 = transistor_on(photo3, photo_thresh3);
  photo_light4 = transistor_on(photo4, photo_thresh4);
  transistors_print();
  delay(100);
}

bool transistor_on(double reading, double threshold){
  bool status;
  (reading >= threshold ? status = 0 : status = 1);
  return status;
}

void transistors_print(){
  BluetoothSerial.print("Photo transistor 1: ");
  BluetoothSerial.print(photo1);
  BluetoothSerial.print(" - STATUS -  ");
  BluetoothSerial.println(photo_light1);

  BluetoothSerial.print("Photo transistor2: ");
  BluetoothSerial.print(photo2);
  BluetoothSerial.print(" - STATUS -  ");
  BluetoothSerial.println(photo_light2);

  BluetoothSerial.print("Photo transistor3: ");
  BluetoothSerial.print(photo3);
  BluetoothSerial.print(" - STATUS -  ");
  BluetoothSerial.println(photo_light3);

  BluetoothSerial.print("Photo transistor4: ");
  BluetoothSerial.print(photo4);
  BluetoothSerial.print(" - STATUS -  ");
  BluetoothSerial.println(photo_light4);
  BluetoothSerial.println("");
}

void initialise_transistors(){
double photo_sum1, photo_sum2, photo_sum3, photo_sum4;
int iterations = 20;

  for (int i = 0; i<=iterations; i++){
    read_transistors();
    photo_sum1 = photo_reading1;
    photo_sum2 = photo_reading2;
    photo_sum3 = photo_reading3;
    photo_sum4 = photo_reading4;
    delay(5);
  }
  photo1 = photo_sum1/iterations;
  photo2 = photo_sum2/iterations;
  photo3 = photo_sum3/iterations;
  photo4 = photo_sum4/iterations;
}


/*******************PROVIDED FUNCTIONS**********************/
void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    BluetoothSerial.print("Lipo level:");
    BluetoothSerial.print(Lipo_level_cal);
    BluetoothSerial.print("%");
    // BluetoothSerial.print(" : Raw Lipo:");
    // BluetoothSerial.println(raw_lipo);
    BluetoothSerial.println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      BluetoothSerial.println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      BluetoothSerial.println("!Lipo is Overchanged!!!");
    else {
      BluetoothSerial.println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      BluetoothSerial.print("Please Re-charge Lipo:");
      BluetoothSerial.print(Lipo_level_cal);
      BluetoothSerial.println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif


/*******************PROVIDED MOTOR FUNCTIONS**********************/
void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward()
{
  
}

void reverse ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
  double speed = 200;
  left_font_motor.writeMicroseconds(1500 - speed);
  left_rear_motor.writeMicroseconds(1500 - speed);
  right_rear_motor.writeMicroseconds(1500 - speed);
  right_font_motor.writeMicroseconds(1500 - speed);
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}
