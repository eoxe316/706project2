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
    TURN_LEFT,
    TURN_RIGHT,
    FAN_ON,
    FAN_OFF,
    STOP
    //add more later
};


/*********IR BINARY************/
enum IR_BINARY{
  LR3,
  LR1,
  MR2,
  MR1,
  SONAR
};


/*********FUNCTION FLAGS AND OUTPUTS*********/
MOTION forward_command;
bool forward_flag;

MOTION avoid_command;
bool avoid_flag;

MOTION escape_command;
bool escape_flag;

MOTION fan_command;
bool fire_flag;
int fires_put_out = 0;
int fan_start_time;

MOTION stop_command;
bool stop_flag;

MOTION motor_input;

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
//IR Equation Variables
double MR1coeff = 2.54;
double MR1power = -1.06;

double MR2coeff = 3.80;
double MR2power = -1.26;

double LR1coeff = 5.03;
double LR1power = -1.00;

double LR3coeff = 4.04;
double LR3power = -0.93;

//IR Sensor working variables
double MR1mm, MR2mm, LR1mm, LR3mm;
double MR1mm_reading, MR2mm_reading, LR1mm_reading, LR3mm_reading;

int MR1pin = A4;
int MR2pin = A7;
int LR1pin = A5;
int LR3pin = A6;

//IR Binary
unsigned int IR_bin = 0b00000;

//IR Kalman
double MR1_var, MR2_var, LR1_var, LR3_var = 0;
double sensor_noise_ir = 8;
double process_noise_ir = 1;


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
double last_var_gyro = 0;

double sensor_noise_gyro = 8;
double process_noise_gyro = 1;


/***SERVO AND FAN***/
Servo turret_motor;
int fan_pin = 21;
double currentAngle = 90;


/***PHOTO TRANSISTORS***/
double photo1, photo2, photo3, photo4;
double photo_reading1, photo_reading2, photo_reading3, photo_reading4;
bool photo_light1, photo_light2, photo_light3, photo_light4;
float photo1_avg, photo2_avg, photo3_avg, photo4_avg;
float photo1_face, photo2_face, photo3_face, photo4_face;

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

/******FAN SETUP******/
int fan_timer;


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

  //Initialise and set turret motor to 0 position
  turret_motor.attach(8);
  turret_motor.write(90);

  //Initialise fan pin
  pinMode(fan_pin, OUTPUT);
  //Turn fan on for testing
  // digitalWrite(fan_pin, HIGH);

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
        Sonar();
        conv_binary(SONAR, sonar_cm);
        break;
      case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
        machine_state = stopped();
        break;
    };
    // Gyro();
    avgphototrans();
    delay(20);
}

float photo_average = 0;

STATE initialising() {
  BluetoothSerial.println("INITIALISING....");

  enable_motors();
  SonarCheck(90); //Initialise sonar values
  initialise_IR();
  initialise_transistors();

  for (int i = 0; i < 150; i++)
  {
    avgphototrans();
    delay(1);
  }

  BluetoothSerial.println("initialise complete");
  return MOTHING;
}


int step_moth = 0;
float finAngleServ = 0;

float prev_conv = 0;
float prev_grad = 0;
float conv_avg = 9300;

float w = 100;

STATE Mothing()
{

  ccw();

  w = 100;

  float convsum = (0.1*photo1_avg + 102*photo2_avg - 93*photo3_avg - 0.092*photo4_avg + 5*conv_avg) / (5 + 1);

  conv_avg = (conv_avg + convsum) / 2;

  float grad = (((convsum - prev_conv) / 2) + prev_grad) / 2;
  
  if ( prev_grad < -1.8 && grad > prev_grad)
  {
    stop();
    BluetoothSerial.println("Stop Now");
    conv_avg = 6000;

    if (fires_put_out == 1)
    {
      fires_put_out = 2;
    }
    return RUNNING;

    // delay(1000);
  }

  prev_conv = convsum;
  prev_grad = grad;

  BluetoothSerial.print("Grad output: ");
  BluetoothSerial.println(grad);
  BluetoothSerial.println(grad - prev_grad);

  return MOTHING;
}

STATE running() {
  read_IR_sensors();
  filter_IR_reading();


  move_forward();
  avoid();
  put_out_fire();
  all_fires_extinguished();
  arbitrate();

  //if puts out 1st fire
  if(fires_put_out == 1){
    return MOTHING;
  }

  //if all fires put out
  if(stop_flag){
    return STOPPED;
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

/*******************BEHAVIOURAL CONTROL FUNCTIONS**********************/
void move_forward(){
    forward_command = FORWARD;
    forward_flag = true;
}

void avoid(){
  BluetoothSerial.println(IR_bin);
  for (int i = 0; i <= 4; i++){
    BluetoothSerial.print("BIT POS");
    BluetoothSerial.print(i);
    BluetoothSerial.print(" READING ");
    BluetoothSerial.println(bitRead(IR_bin, i));
  }

  if(!check_bits(LR1) && !check_bits(LR3) && !check_bits(SONAR)){
    BluetoothSerial.println("NO AVOID REQUIRED");
    avoid_flag = false;
  }else if(check_bits(MR1) && check_bits(MR2)){
    avoid_flag = true;
    avoid_command = TURN_RIGHT;
  }else if(check_bits(MR1) || (!check_bits(MR2) && check_bits(LR1))){
    avoid_flag = true;
    avoid_command = TURN_RIGHT;
  }else{
    avoid_flag = true;
    avoid_command = TURN_LEFT;
  }
}

void put_out_fire(){
  //initial fire check
  float middle_avg = (photo2_avg+photo3_avg)/2;

  if((digitalRead(fan_pin) == LOW) && (middle_avg < 300) && (check_bits(SONAR) || check_bits(LR1) || check_bits(LR3))){
    // fire_count++;
    // if (fire_count == 3){
      fire_flag = true;
      fan_command = FAN_ON;
      BluetoothSerial.println("STARTED TIMER");
      fan_start_time = millis();    //start the 10s timer
      digitalWrite(fan_pin, HIGH);
      //if this is the first time its turning on the fan

    // }
  //check if the fan is already on
  }else if(digitalRead(fan_pin) == HIGH){
    //if fire goes out or 10s elapsed
    // BluetoothSerial.println("PUTTING PUTTIN GPTIEADLJLAKSJDLASKJDLASKDJLASKDJLASKDLJSAJD");
    if((middle_avg > 1200) || (millis() - fan_start_time >= 8000)){
      //BluetoothSerial.println("Fire has been put out --------------------------");
      //blocking code
      digitalWrite(fan_pin, LOW);
      ccw();
      delay(500);
      fire_flag = false;
      fan_command = FAN_OFF;
      fires_put_out++;
    }
  //if nothing
  }else{
    fire_flag = false;
    fan_command = FAN_OFF;
  }
}

void all_fires_extinguished(){
  if(fires_put_out == 3){
    stop_flag = true;
  }else{
    stop_flag = false;
  }
  stop_command = STOP;
}

void arbitrate(){
    if(forward_flag == true){
        BluetoothSerial.println("going forward");
        motor_input = forward_command;
    }
    if(avoid_flag == true){
        BluetoothSerial.println("Avoiding you");
        motor_input = avoid_command;
    }
    if(fire_flag == true){
        BluetoothSerial.println("FiRE FIRE FIRE"); 
        motor_input = fan_command;
    }
    if(stop_flag == true){
        BluetoothSerial.println("FIRES PUT OUT");
        motor_input = stop_command;
    }
    robot_move();
}

//SOMEONE CHANGE THESE DELAYS
void robot_move(){
    switch (motor_input){
        case FORWARD:
            ClosedLoopStraight(500);
            break;
        case TURN_LEFT:
            ccw();
            Sunflower();
            break;
        case TURN_RIGHT:
            cw();
            Sunflower();
            break;
        case FAN_ON:
            stop();
            digitalWrite(fan_pin, HIGH);
            // fan_on();
            break;
        case FAN_OFF:
            digitalWrite(fan_pin, LOW);
            // fan_off();
            break;
        case STOP:
            stop();
            break;
    }
}

float oldAngle = 0;

void Sunflower()
{

  float k = 0.005;
  float w2 = 0;
  w = 0;

  float convsum = (10*photo1 + 1*photo2 - 1*photo3 - 10*photo4 + w2*conv_avg) / (w2 + 1);

  conv_avg = (conv_avg + convsum) / 2;

  currentAngle = constrain(currentAngle - k * convsum, 0, 180);
  
  if (abs(currentAngle - oldAngle) > 0.5) 
  {
    turret_motor.write(currentAngle);
  }

  oldAngle = currentAngle;

  // BluetoothSerial.print("Current Angle: ");
  // BluetoothSerial.println(currentAngle);
  // BluetoothSerial.print("Conv sum: ");
  // BluetoothSerial.println(convsum);


}

/*******************CONTROL LOOPS**********************/
void ClosedLoopStraight(int speed_val)
{
    double e, correction_val;

    double kp_gyro = 5;
    double ki_gyro = 0;

    BluetoothSerial.print("CURRENT SERVO ANGLE: ");
    BluetoothSerial.println(currentAngle);

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
    left_rear_motor.writeMicroseconds(1500 + speed_in - correction_val);
    right_rear_motor.writeMicroseconds(1500 - speed_in - correction_val);
    right_font_motor.writeMicroseconds(1500 - speed_in - correction_val);

    Sunflower();
}


void ClosedLoopStrafe(int speed_val)
{
    double e_gyro, e_ir, correction_val_gyro = 0, correction_val_ir = 0;
    double kp_gyro = 25;
    double ki_gyro = 5;

    //double kp_ir = 0;
    //double ki_ir = 0;
    (abs(gyroAngleChange) < 3) ? e_gyro = gyroAngleChange : e_gyro = 0;

    correction_val_gyro = constrain(kp_gyro * e_gyro + ki_gyro * ki_strafe_gyro, -CONTROL_CONSTRAINT_GYRO, CONTROL_CONSTRAINT_GYRO);
    
    ki_strafe_gyro += e_gyro;

    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val_gyro - correction_val_ir);
    left_rear_motor.writeMicroseconds(1500 - speed_val - correction_val_gyro - correction_val_ir);
    right_rear_motor.writeMicroseconds(1500 - speed_val - correction_val_gyro + correction_val_ir);
    right_font_motor.writeMicroseconds(1500 + speed_val - correction_val_gyro + correction_val_ir);
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
  //rawdata = constrain(rawdata, sonar_cm - 20, sonar_cm + 20);
  double a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_var = sonar_variance + process_noise_sonar; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise_sonar);
  a_post_est = sonar_cm + kalman_gain*(rawdata-sonar_cm);
  sonar_variance = (1 * kalman_gain)*a_priori_var;
  sonar_cm = rawdata;
  return a_post_est; 
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
  // print_IR();
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
  // MR1mm = constrain(Kalman_IR(MR1mm_reading, MR1mm, &MR1_var), 0, 25);
  // MR2mm = constrain(Kalman_IR(MR2mm_reading, MR2mm, &MR2_var), 0, 25);
  // LR1mm = constrain(Kalman_IR(LR1mm_reading, LR1mm, &LR1_var), 0, 40);
  // LR3mm = constrain(Kalman_IR(LR3mm_reading, LR3mm, &LR3_var), 0, 40);
  MR1mm = constrain(MR1mm_reading, 0, 25);
  MR2mm = constrain(MR2mm_reading, 0, 25);
  LR1mm = constrain(LR1mm_reading, 0, 40);
  LR3mm = constrain(LR3mm_reading, 0, 40);

  print_IR();

  conv_binary(MR1, MR1mm);
  conv_binary(MR2, MR2mm);
  conv_binary(LR1, LR1mm);
  conv_binary(LR3, LR3mm);
  
}

double average_IR(double IR1, double IR2) {
  return (IR1 + IR2) / 2;
}

double Kalman_IR(double rawdata, double prev_val_ir, double* last_var_ir){   // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_var = *last_var_ir + process_noise_ir; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise_ir);
  a_post_est = prev_val_ir + kalman_gain*(rawdata-prev_val_ir);
  a_post_var = (1- kalman_gain)*a_priori_var;
  *last_var_ir = a_post_var;

  return a_post_est;
}

void print_IR(){

  // BluetoothSerial.print("LR3 RAW: ");
  // BluetoothSerial.print(LR3mm_reading);
  BluetoothSerial.print("LR3: ");
  BluetoothSerial.println(LR3mm);

  // BluetoothSerial.print("VLADISLAV COLPMAN");

  // BluetoothSerial.print("LR1 RAW: ");
  // BluetoothSerial.print(LR1mm_reading);
  BluetoothSerial.print("LR1: ");
  BluetoothSerial.println(LR1mm);

  // BluetoothSerial.print("ANDREW J. KOH"); // Not reaching??

  // BluetoothSerial.print("MR2 RAW: ");
  // BluetoothSerial.print(MR2mm_reading);
  //BluetoothSerial.print("MR2: ");
  //BluetoothSerial.println(MR2mm);

  // BluetoothSerial.print("MR1 RAW: ");
  // BluetoothSerial.print(MR1mm_reading);
  //BluetoothSerial.print(" MR1: ");
  //BluetoothSerial.println(MR1mm);

  BluetoothSerial.print("Sonar ");
  BluetoothSerial.println(sonar_cm);
  BluetoothSerial.println("");
}

/*******************PHOTOTRANSISTOR FUNCTIONS**********************/
void read_transistors(){
  // BluetoothSerial.println("READ SENSOR START");
  photo_reading1 = analogRead(photo_pin1);
  photo_reading2 = analogRead(photo_pin2);
  photo_reading3 = analogRead(photo_pin3);
  photo_reading4 = analogRead(photo_pin4);
}

double pht1_avg = 1024;
double pht2_avg = 1024;
double pht3_avg = 1024;
double pht4_avg = 1024;

void update_transistors(){ 
  //Updates transistor state based on transistor readings
  read_transistors();

  // double photo_wgt2 = 1.02;
  // double photo_wgt3 = 0.93;
  // double photo_wgt4 = 0.92;

  //Filters transistor values
  photo1 = (photo_reading1 + (w * pht1_avg)) / (w + 1);
  photo2 = (photo_reading2 + (w * pht2_avg)) / (w + 1);
  photo3 = (photo_reading3 + (w * pht3_avg)) / (w + 1);
  photo4 = (photo_reading4 + (w * pht4_avg)) / (w + 1);

  pht1_avg = (pht1_avg + photo1) / 2;
  pht2_avg = (pht2_avg + photo2) / 2;
  pht3_avg = (pht3_avg + photo3) / 2;
  pht4_avg = (pht4_avg + photo4) / 2;

  //Updates boolean light value based of transistor threshold
  // photo_light1 = transistor_on(photo1, photo_thresh1);
  // photo_light2 = transistor_on(photo2, photo_thresh2);
  // photo_light3 = transistor_on(photo3, photo_thresh3);
  // photo_light4 = transistor_on(photo4, photo_thresh4);
  // transistors_print();
}

bool transistor_on(double reading, double threshold){
  bool status;
  (reading >= threshold ? status = 0 : status = 1);
  return status;
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

void transistors_print(){
  BluetoothSerial.print("Photo transistor 1: ");
  BluetoothSerial.println(photo1_avg);

  BluetoothSerial.print("Photo transistor2: ");
  BluetoothSerial.println(photo2_avg);

  BluetoothSerial.print("Photo transistor3: ");
  BluetoothSerial.println(photo3_avg);

  BluetoothSerial.print("Photo transistor4: ");
  BluetoothSerial.println(photo4_avg);
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

/********************HELPER FUNCS***************************/
void conv_binary(IR_BINARY binary_type, double reading){
  //Default to MR threshold
  double threshold = 10;
  //If no MR change to other relevaant threshold
  if(binary_type == SONAR){
    threshold = 25;
  }
  else if (binary_type == LR3){
    threshold = 32;
  }
  else if(binary_type == LR1){
    threshold = 32;
  }
  else if(binary_type == MR2){
    threshold = 10;
  }

  if(reading <= threshold){
    IR_bin |= (1 << binary_type);   //flip its respective bit
  }
  else{
    //else if not under
    IR_bin &= ~(1 << binary_type);
  }
  
}

bool check_bits(int pos) {
  // // Create a mask with a 1 at the specified position and 0 in all other positions
  // unsigned int mask = 1 << pos;
  
  // // Use bitwise AND to check if the bit at the specified position is set
  // // If the result is non-zero, the bit is set; otherwise, it's not set
  // if (IR_bin & mask) {
  //     return 1; // Bit is set (1)
  // } else {
  //     return 0; // Bit is not set (0)
  // }
  return(bitRead(IR_bin, pos));
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
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
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

void left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}
