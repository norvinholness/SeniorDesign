#include <Wire.h>  // I2C Library 
// Accelerometer
#define ACCELEROMETER (0x53)   // Accelerometer Address 
#define ACC_READ (6)           // 6 Bytes are going to be read for x, y and z axis (2 bytesper each) 

// ITG3200 Gyroscope 
#define GYROSCOPE (0X68)       // Gyroscope Address (AD0 hooked up to ground) 
#define GYRO_READ (6)          // 6 Bytes are going to be read for x, y and z axis (2 bytes per each) 
#define POWER_MNG (0x3E)       // ************************************** 
#define INTERRUPT_CONF (0x17)  // Defined registers based on ITG3200 datasheet  
#define SAMPLE_RATE (0x15)         
#define FS_DLPF (0x16)     

float acc_data[3]; 
float gyro_data[3]; 

//Character that will be sent from android to communicate with the arduino
//Example if '1' do this, if '2' do this.. etc
char inputChar = 0;

//Arrays to store values of sensors
float sensorValueArray[4] = {0,0,0,0};
float voltageValue[4] = {0,0,0,0};

void startACC() { 
    writeTo(ACCELEROMETER, 0x2D, 0);   // Accelerometer is turned on, range is -2g to 2g by default 
    writeTo(ACCELEROMETER, 0x2D, 16); 
    writeTo(ACCELEROMETER, 0x2D, 8); 
}

void startGYRO(){ 
    writeTo(GYROSCOPE, POWER_MNG, 0x00);          // Powers the sensor (Internal Oscillator) 
    writeTo(GYROSCOPE, INTERRUPT_CONF, 0x00);     // We do not use interrupt operation 
    writeTo(GYROSCOPE, SAMPLE_RATE, 0x07);        // 125 Hz sample rate (every 8 ms) 
    writeTo(GYROSCOPE, FS_DLPF, 0x1D);            // Range +-2000 deg/s / 10 Hz low pass / 1KHz internal sample rate 
}

void getACC(float * data) { 
    int register_1st = (0x32);   // First register of all axises 
    byte buffer [ACC_READ];      // Buffer for storing data  
    float scale_cons = 0.0039;   // Constant used for converting data into scaled 

    // Read the accelerometer data 6 bytes starting from the register_1st 

    readFrom(ACCELEROMETER, register_1st, ACC_READ, buffer);     
    data[0] = ((((buffer[1]) << 8) | buffer[0])-20)*scale_cons;  // Data comes 10 bits 
    data[1] = ((((buffer[3]) << 8) | buffer[2])-16)*scale_cons;  // Shifting is performed   
    data[2] = (((buffer[5]) << 8) | buffer[4])*-scale_cons;      // Conversion from bytes to integer 
    data[0] = sqrt(sq(data[0])+sq(data[1])+sq(data[2]));     	 // Data is averaged 
} 

void getGYRO(float * data) { 
      int register_1st = (0x1D);          // First register of all axises (no temperature registers) 
      byte buffer [GYRO_READ];            // Buffer for storing data 
      // Read the gyroscope data 6 bytes starting from register_1st 
      readFrom(GYROSCOPE, register_1st, GYRO_READ, buffer);       
      int GYROFFSET_X = 25;               // ********************************** 
      int GYROFFSET_Y = 29;               // Gyroscope`s specific offset values 
      int GYROFFSET_Z = 35.5;             // ********************************** 
      float sens_cons = 14.375;  // Constant used for converting data into scaled 

      // Data comes 16 bits so shifting is performed 
      data[0] = ((((buffer[0]) << 8) | buffer[1]) + GYROFFSET_X) / sens_cons;     
      // Data is converted from bytes to integers 
      data[1] = ((((buffer[2]) << 8) | buffer[3]) + GYROFFSET_Y) / sens_cons;     
      // Gyroscope data is converted to deg/sec by using sensitivity constant (14.375) 
      data[2] = ((((buffer[4]) << 8) | buffer[5]) + GYROFFSET_Z) / sens_cons;     
      data[0] = sqrt(sq(data[0])+sq(data[1])+sq(data[2]));  // Data is averaged 
} 

// Writing value to register in ADXL345 
void writeTo(int SENSOR, byte REGADD, byte VALUE){ 
	Wire.beginTransmission(SENSOR);  // Beginning transmission  
	Wire.write(REGADD);              // Sending register address 
	Wire.write(VALUE);               // Sending value to write 
	Wire.endTransmission();          // Ending the transmission 
} 

// Reading the bytes starting from the first register of the axises 
void readFrom(int SENSOR, byte INITREG, int BYTES, byte BUFFER[]){ 
	Wire.beginTransmission(SENSOR);   // Beginning transmission  
	Wire.write(INITREG);              // Sending the address to be read 
	Wire.endTransmission();           // Ending the transmission 
	Wire.beginTransmission(SENSOR);   // Beginning transmission  
	Wire.requestFrom(SENSOR, BYTES);  // Requesting bytes  
	int i = 0; 
	while(Wire.available())                 
	{  
		BUFFER[i] = Wire.read();  // Receive one byte 
		i++;                      // When i increases 
	} 
	Wire.endTransmission();       // Ending transmission 
} 

void setup() {
	Wire.begin();               // Initiating I2C 
	startACC();                 // Calling the function  
	startGYRO();                // Calling the function
	delay(5);

    Serial.println("CLEARDATA"); //clears up any data left from previous projects
    //initialize the communication via serial '9600 baud'
    Serial.begin(9600);

    //Wait for serial to be ready
    //DEBUG AND DISCUSS THE REMOVAL OF THIS LINE OF CODE
    //MAY STALL PROGRAM
    while (! Serial); 
    Serial.println("Serial Communication Ready!");
}
 
void loop() {
	getACC(acc_data);       // Calling the function 
	getGYRO(gyro_data);     // Calling the function 
	
	//Reads the data from the gyroscope and accelerometer to store values
	//into an sensorValueArray array
	readSensors();

	//Process raw data to convert raw data to voltage
	//EXAMPLE
	getVoltageFromSensorValue();

	//Method of sending value to android in a single string
	//Android will then process the string with the added delimeters
	sendValuesToMobile();
  
	//This becomes true when any serial values are detected
	if (Serial.available() > 0)
	{
    //Stores the read char into variable
    inputChar = Serial.read();
		if (inputChar == '0'){
			Serial.println("0 was pressed");
		}
		if (inputChar == '1') {
			Serial.println("1 was pressed");
		}
	}
	
	//2s delay
	//MAY NEED TO INCREASE
	delay(2000);
}
 
void readSensors()
{
	//Delay is given 
	delay(76);   
	//Reads x,y,z values of gyroscope and accelerometer
	//DUMMYDATA RAW DATA GOES HERE
	sensorValueArray[0] = gyro_data[0];
	sensorValueArray[1] = acc_data[0];
	//sensorValueArray[2] = accelz;
	//sensorValueArray[3] = 1;
}

//sends the values from the sensor over serial to 
//BT module with proper formatting
void sendValuesToMobile()
 {
	  //Adds a '$' before the string for the Mobile Application to know
	  //Where the string starts incase of corrupt data
	  Serial.print('$');
	  
	  //loop through the array of values gathered from the sensor
	  //this will place them into proper formatting to send to 
	  //the Mobile Application
	  //'i' represents the four sensor sources being gathered
	  for(int i=0; i<2; i++)
	  {
		//Prints out the value of the data with a space to seperate the 
		//string, the space char will be the delimeter in the Mobile Application
		Serial.print(voltageValue[i]);
		Serial.print(' ');
	  }
	  //'!' represents end of transmission of the string, for the Mobile Application
	  //To know when the string ends, this is also a method of getting the string
	  //length
	  Serial.print('!');
	  //Prints new line for formatting
	  Serial.println();

	  //Delayed to reduce any missed transmissions
	  delay(10);
}

void getVoltageFromSensorValue()
{
	  //Loops through all 4 sensor values and processes each
	  for (int x = 0; x < 4; x++)
	  {
		//Processing goes here
		voltageValue[x] = sensorValueArray[x];
	  }
}
