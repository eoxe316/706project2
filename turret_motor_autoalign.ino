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

//Other defines for other topics
#define CONTROL_CONSTRAINT_GYRO 100

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

/*********STATE MACHINES************/
//Main state machine
enum STATE {
  INITIALISING,
  MOTHING,
  RUNNING,
  STOPPED
};

/*********MOTION STATES**********/
enum MOTION{
    FORWARD,
    STRAFE_LEFT,
    STRAFE_RIGHT
    //add more later
};

enum SERVO_MOTION{
    LIGHT_FORWARD,
    LIGHT_LEFT,
    LIGHT_RIGHT
    //Dunno if this should be seperate but just bear with me
};

enum MOTHINGO{
  SERVO_TURNING,
  ROBOTO_TURNING,
  FINDING
};

/*********FUNCTION FLAGS AND OUTPUTS*********/
MOTION forward_command;
bool forward_flag;

MOTION avoid_command;
bool avoid_flag;
bool light_flag;

MOTION motor_input;
SERVO_MOTION servo_input;
SERVO_MOTION light_command;

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
int LR1pin = A5;
int LR3pin = A6;


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

//Gyro Control Loop
double ki_straight_gyro = 0;
double ki_strafe_gyro = 0;
double ki_integral_angle = 0;

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

//Light angle variables
double light_angle;

// double photo1[2][15] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150; 0.097560976, 151.85, 564.6944444, 740.8857143, 827.8461538, 878.0526316, 909.21875, 927.9512195, 941.2121212, 950.3142857, 955.9210526, 962.4864865, 966.6388889, 970.7179487, 973.7777778}


void setup(void)
{
  BluetoothSerial.begin(115200);
  BluetoothSerial.println("");
  BluetoothSerial.println("");
  BluetoothSerial.println("");
  pinMode(LED_BUILTIN, OUTPUT);

  turret_motor.attach(8);
  turret_motor.write(90);

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
      case MOTHING:
        machine_state = Mothing();
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
    avgphototrans();
    delay(10);
}


// variables to control light follow function
#define TURRET_MAX_TURN 10
// Sets initial angle to straight
float currentAngle = 90;
float photo_average = 0;

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

  //Locate the light before beginning
  // locate_light();

  for (int i = 0; i < 50; i++)
  {
    avgphototrans();
    delay(1);
  }

  return MOTHING;
}

float photo1_avg;
float photo2_avg;
float photo3_avg;
float photo4_avg;
int step_moth = 0;
float finAngleServ = 0;

float conv[3] = {1024, 1024, 1024};
float prev_conv = 0;
float prev_grad = 0;
float conv_avg = 9300;
float w =100;

STATE Mothing()
{

  ccw();

  float convsum = (0.1*photo1_avg + 100*photo2_avg - 100*photo3_avg - 0.1*photo4_avg + 5*conv_avg) / (5 + 1);

  conv_avg = (conv_avg + convsum) / 2;

  float grad = (((convsum - prev_conv) / 2) + prev_grad) / 2;
  
  if ( prev_grad < -1.8 && grad > prev_grad)
  {
    stop();
    BluetoothSerial.println("Stop Now");
    conv_avg = 6000;
    w = 0;
    return RUNNING;
  }

  prev_conv = convsum;
  prev_grad = grad;

  BluetoothSerial.print("Grad output: ");
  BluetoothSerial.println(grad);
  BluetoothSerial.println(grad - prev_grad);

  return MOTHING;
}

void avgphototrans()
{
  update_transistors();

  photo1_avg = photo1;
  photo2_avg = photo2;
  photo3_avg = photo3;
  photo4_avg = photo4;

  for (int i = 0; i < 10; i++)
  {
    update_transistors();

    delay(5);

    photo1_avg += photo1;
    photo2_avg += photo2;
    photo3_avg += photo3;
    photo4_avg += photo4;
  }

  photo1_avg = photo1_avg / 10;
  photo2_avg = photo2_avg / 10;
  photo3_avg = photo3_avg / 10;
  photo4_avg = photo4_avg / 10;

}


STATE running() {
  avgphototrans();
  read_IR_sensors();
  filter_IR_reading();

  ClosedLoopStrafe(300);
  


//   move_forward();
//   avoid();
//   arbitrate();

  
  return RUNNING;
}

void Sunflower()
{

  float k = 0.005;
  float w2 = 0;

  float convsum = (10*photo1 + 1*photo2 - 1*photo3 - 10*photo4 + w2*conv_avg) / (w2 + 1);

  conv_avg = (conv_avg + convsum) / 2;

  float newangle = constrain(currentAngle - k * convsum, 0, 180);
  
  if (abs(newangle - currentAngle) > 0.5) 
  {
    turret_motor.write(currentAngle);
    currentAngle = newangle;
  }

  // BluetoothSerial.print("Current Angle: ");
  // BluetoothSerial.println(currentAngle);
  // BluetoothSerial.print("Conv sum: ");
  // BluetoothSerial.println(convsum);


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

/*******************BEHAVIOURAL CONTROL FUNCTIONS**********************/
void move_forward(){
    forward_command = FORWARD;
    forward_flag = true;
}

void avoid(){
    BluetoothSerial.print("sonar distance: ");
    BluetoothSerial.println(sonar_cm);
    // if(sonar_cm < 10){
    //     if(LR1mm_reading < 300 && LR3mm_reading > 300){
    //         avoid_flag = true;
    //         avoid_command = STRAFE_RIGHT;
    //     }else{
    //         avoid_flag = true;
    //         avoid_command = STRAFE_LEFT;
    //     }
    // }else{
    //     avoid_flag = false;
    // }

    if(LR1mm_reading < 300 && LR3mm_reading > 300){
        avoid_flag = true;
        avoid_command = STRAFE_RIGHT;
    }else if(LR1mm_reading > 300 && LR3mm_reading < 300){
        avoid_flag = true;
        avoid_command = STRAFE_LEFT;
    }else if(LR1mm_reading < 300 && LR3mm_reading < 300){
        avoid_flag = true;
        avoid_command = STRAFE_RIGHT;        
    }else{
        avoid_flag = false;
    }
}

void turntolight(){
  //This function turns towards the light while in the running function
  //Find the light again if it has been found before and has been lost due to strafe
  bool facing_light;
  (photo_light1 + photo_light2 + photo_light3 + photo_light4 >= 3) ? facing_light = 1 : facing_light = 0;
  if(!facing_light && avoid_command == STRAFE_RIGHT){ 
    //If last strafe was to the right, light is probably to the left of robot
    light_flag = true;
    light_command = LIGHT_LEFT;
  }
  if(!facing_light && avoid_command == STRAFE_LEFT){
    //If last strafe was left, light is to the right of the robot
    light_flag = true;
    light_command = LIGHT_RIGHT;
  }
  else{ 
    //at this point assume is a misread, but later on we can fix this logic
    light_flag = false;
  }
}


void arbitrate(){
    if(forward_flag == true){
        motor_input = forward_command;
    }
    if(avoid_flag == true){
        motor_input = avoid_command;
    }
    if(light_flag == true){
        servo_input = light_command;
    }
    robot_move();
}

void robot_move(){
    switch (motor_input){
        case FORWARD:
            ClosedLoopStraight(300);
            break;
        case STRAFE_LEFT:
            ClosedLoopStrafe(-300);
            break;
        case STRAFE_RIGHT:
            ClosedLoopStrafe(300);
            break;
    }

    switch(servo_input){
      case LIGHT_LEFT:
        light_angle = light_angle - 5;
        turret_motor.write(light_angle);

        break;
      case LIGHT_RIGHT:
        light_angle =light_angle + 5;
        turret_motor.write(light_angle);

        break;
      case LIGHT_FORWARD:
        turret_motor.write(light_angle);

        break;
    }
}

void locate_light(){
  //This function initially finds where the light is
  //Turn servo at angles until light is detected
  for (int i = 0; i <= 180; i+10){
    turret_motor.write(i);
    delay(100);
    update_transistors();
    //Check if at least 3 transistors read the light
    if(photo_light1 + photo_light2 + photo_light3 + photo_light4 >= 3){
      light_angle = i - 90;
      break;
    }
  }
  BluetoothSerial.print("LIGHT DETECTED AT ANGLE ");
  BluetoothSerial.println(light_angle);
}

/*******************CONTROL LOOPS**********************/
void ClosedLoopStraight(int speed_val)
{
    double e, correction_val;

    double kp_gyro = 30;
    double ki_gyro = 1;

    e = -(currentAngle - 90);

    double correction_val_1 = kp_gyro * e + ki_gyro * ki_straight_gyro;

    correction_val = constrain(correction_val_1, -300, 300);

    ki_straight_gyro += e;

    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val - correction_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val - correction_val);
    right_font_motor.writeMicroseconds(1500 - speed_val - correction_val);
}

void ClosedLoopStrafe(int speed_val)
{
    // double e_gyro, e_ir, correction_val_gyro = 0, correction_val_ir = 0;
    // double kp_gyro = 25;
    // double ki_gyro = 5;

    // //double kp_ir = 0;
    // //double ki_ir = 0;
    // (abs(gyroAngleChange) < 3) ? e_gyro = gyroAngleChange : e_gyro = 0;

    // correction_val_gyro = constrain(kp_gyro * e_gyro + ki_gyro * ki_strafe_gyro, -CONTROL_CONSTRAINT_GYRO, CONTROL_CONSTRAINT_GYRO);
    
    // ki_strafe_gyro += e_gyro;

    // left_font_motor.writeMicroseconds(1500 + speed_val - correction_val_gyro );
    // left_rear_motor.writeMicroseconds(1500 - speed_val - correction_val_gyro);
    // right_rear_motor.writeMicroseconds(1500 - speed_val - correction_val_gyro);
    // right_font_motor.writeMicroseconds(1500 + speed_val - correction_val_gyro);

    // double e, correction_val;

    // double kp_gyro = 5;
    // double ki_gyro = 0;

    // BluetoothSerial.print("CURRENT SERVO ANGLE: ");
    // BluetoothSerial.println(currentAngle);
    double e, correction_val;

    double kp_gyro = 5;
    double ki_gyro = 0;

    e = (currentAngle - 90);

    double correction_val_1 = kp_gyro * e + ki_gyro * ki_straight_gyro;

    // correction_val = constrain(correction_val_1, -(500 - speed_val), (500 - speed_val));
    BluetoothSerial.print("CURRENT SERVO ANGLE: ");
    BluetoothSerial.println(currentAngle);
    BluetoothSerial.print("CORRECTION VAL ");
    BluetoothSerial.println(correction_val);

    ki_straight_gyro += e;

    double speed_in = constrain(speed_val, -(speed_val - (speed_val / 90) * abs(e)), (speed_val - (speed_val / 90) * abs(e)));
    correction_val = constrain(correction_val_1, -(500 - speed_in), (500 - speed_in));

    left_font_motor.writeMicroseconds(1500 + speed_in - correction_val);
    left_rear_motor.writeMicroseconds(1500 - speed_in - correction_val);
    right_rear_motor.writeMicroseconds(1500 - speed_in - correction_val);
    right_font_motor.writeMicroseconds(1500 + speed_in - correction_val);

    Sunflower();
}

double ClosedLoopTurn(double speed, double target_angle)
{
  double e, correction_val;
  double kp_angle = 6;
  double ki_angle = 0;

  e = target_angle-gyroAngle;

  correction_val = constrain(kp_angle * e + ki_angle * ki_integral_angle, -speed, speed);

  ki_integral_angle += e;

  left_font_motor.writeMicroseconds(1500 + correction_val);
  left_rear_motor.writeMicroseconds(1500 + correction_val);
  right_rear_motor.writeMicroseconds(1500 + correction_val);
  right_font_motor.writeMicroseconds(1500 + correction_val);

  BluetoothSerial.print("Current Error: ");
  BluetoothSerial.println(e);
  BluetoothSerial.print("Gyro aim: ");
  BluetoothSerial.println(target_angle);
  return e;
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
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float e, u;

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

double average_IR(double IR1, double IR2) {
  return (IR1 + IR2) / 2;
}

/*******************PHOTOTRANSISTOR FUNCTIONS**********************/
void read_transistors(){
  // BluetoothSerial.println("READ SENSOR START");
  photo_reading1 = analogRead(photo_pin1);
  photo_reading2 = analogRead(photo_pin2);
  photo_reading3 = analogRead(photo_pin3);
  photo_reading4 = analogRead(photo_pin4);
}

// double Poly(double x, double p4, double p3, double p2, double p1, double p0)
// {
//   return (p4 * pow(x,4)) + (p3 * pow(x,3)) + (p2 * pow(x,2)) + (p1 * (x)) + p0;
// }

double pht1_avg = 1024;
double pht2_avg = 1024;
double pht3_avg = 1024;
double pht4_avg = 1024;


void update_transistors(){ 
  //Updates transistor state based on transistor readings
  read_transistors();

  //Filters transistor values
  // photo1 = Poly(photo_reading1, -0.000008, 0.0037, -0.659, 50.851, -501.98);
  // photo2 = Poly(photo_reading2, -0.000004, 0.0023, -0.5042, 45.173, -477.75);
  // photo3 = Poly(photo_reading3, -0.00001, 0.0051, -0.8126, 56.867, -535.26);
  // photo4 = Poly(photo_reading4, -0.000001, 0.0016, -0.4128, 41.197, -460.74);

  photo1 = (photo_reading1 + (w * pht1_avg)) / (w + 1);
  photo2 = (photo_reading2 + (w * pht2_avg)) / (w + 1);
  photo3 = (photo_reading3 + (w * pht3_avg)) / (w + 1);
  photo4 = (photo_reading4 + (w * pht4_avg)) / (w + 1);

  pht1_avg = (pht1_avg + photo1) / 2;
  pht2_avg = (pht2_avg + photo2) / 2;
  pht3_avg = (pht3_avg + photo3) / 2;
  pht4_avg = (pht4_avg + photo4) / 2;

  BluetoothSerial.print("Photo 1: ");
  BluetoothSerial.println(photo1);
  BluetoothSerial.print("Photo 2: ");
  BluetoothSerial.println(photo2);
  BluetoothSerial.print("Photo 3: ");
  BluetoothSerial.println(photo3);
  BluetoothSerial.print("Photo 4: ");
  BluetoothSerial.println(photo4);


  //Updates boolean light value based of transistor threshold
  photo_light1 = transistor_on(photo1, photo_thresh1);
  photo_light2 = transistor_on(photo2, photo_thresh2);
  photo_light3 = transistor_on(photo3, photo_thresh3);
  photo_light4 = transistor_on(photo4, photo_thresh4);
  // transistors_print();
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

// This is stupid

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
