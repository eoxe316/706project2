/**INITIALISATION OF VARIABLES*/
#include <SoftwareSerial.h>

// Serial Data Pins
#define BLUETOOTH_RX 10
#define BLUETOOTH_TX 11

// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

int sensor_pin = A12;

void setup(void)
{
  BluetoothSerial.begin(115200);
  BluetoothSerial.println("");
  BluetoothSerial.println("");
  BluetoothSerial.println("");
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialise Sensor Pins
  pinMode(sensor_pin, INPUT);
  delay(1000); //settling time but noT really needed
}


/*******************SUPER LOOP**********************/
void loop(void) //main loop
{
  // BluetoothSerial.print("Pin reading:");
  BluetoothSerial.println(analogRead(sensor_pin));
  // BluetoothSerial.println("");
  delay(1000);
}