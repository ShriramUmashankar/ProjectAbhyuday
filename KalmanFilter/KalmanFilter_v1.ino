#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BME280 bme;

#define SEALEVELPRESSURE_HPA (1013.25)
//GPS
#define GPS_TX 16
#define GPS_RX 17
SoftwareSerial FC_GPS(GPS_RX,GPS_TX);
TinyGPSPlus gps;


float height;
float initial_bme = 0;
float initial_gps = 0;
float t = 0.1;

int Sensor;
// 1 -- barometer
// 2 - gps
// 3 - accelrometer
//x = {{position},{velocity},{acceleration}}^(T)

BLA::Matrix<3,1> x; BLA::Matrix<3,1> x_inter; BLA::Matrix<3,1> x_prev = {0,0,0};
BLA::Matrix<3,3> P; BLA::Matrix<3,3> P_inter; BLA::Matrix<3,3> P_prev = {5,0,0,0,5,0,0,0,5};
BLA::Matrix<3,3> A = {1,t,t*t/2,0,1,t,0,0,1}; BLA::Matrix<3,3> A_T = {1,0,0,t,1,0,t*t/2,t,1};
BLA::Matrix<3,3> Q={5,0,0,0,5,0,0,0,2};BLA::Matrix<3,1> K;
BLA::Matrix<1,3> H; BLA::Matrix<3,1> H_T; 
BLA::Matrix<1,1> R;BLA::Matrix<1,1> z;

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do { 
    //Encode data read from GPS while data is available on serial port
    while (FC_GPS.available())
      gps.encode(FC_GPS.read());
  //Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it 
  } while (millis() - start < ms);
}

void init_bme(){
  for(int i=0;i<10;i++){
    initial_bme = initial_bme + bme.readAltitude(SEALEVELPRESSURE_HPA);
  }
  initial_bme = initial_bme/10;
}

void init_gps(){
  Serial.println("looking for sattelites");
  delay(10000);
  smartDelay(100);

  if(gps.satellites.value()>2){
    for(int i=0;i<10;i++){
      smartDelay(100);
      initial_gps = initial_gps + gps.altitude.meters();
    }
  initial_gps = initial_gps/10;
  }
  else{
    initial_gps = 13;
  }
  Serial.println(initial_gps);
  delay(2000);
}


void predict(){
  x_inter = A*x_prev ;
  P_inter = A*P_prev*A_T + Q ;
}

void KalmanGain(int Sensor){
  if (Sensor == 1){
    H = {1,0,0};
    H_T= {1,0,0};
    R = {0.3};
  }

  if (Sensor == 2){
    H = {1,0,0};
    H_T= {1,0,0};
    R = {0.9};
  }

  if (Sensor == 3){
    H = {0,0,1};
    H_T= {0,0,1};
    R = {0.9};
  }
  BLA::Matrix<1,1> intermediate = H*P_inter*H_T + R ;
  K = P_inter*H_T*Inverse(intermediate);
  
}

void estimate_update(int Sensor){
  if(Sensor == 1){
    z = {bme.readAltitude(SEALEVELPRESSURE_HPA)-initial_bme};
  }

  if(Sensor == 2){
    smartDelay(100);
    z = {gps.altitude.meters() - initial_gps};
  }

  if(Sensor == 3){
    z = {-9.81};
  }

  x = x_inter + K*(z - H*x_inter);
  P = P_inter - K*H*P_inter ;

  x_prev = x;
  P_prev = P;

  height = x(0,0);
}


void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
  FC_GPS.begin(9600);
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

  init_bme();
  init_gps();

}

void loop(void) 
{
  smartDelay(100);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  int Sensor = 1;
  predict();
  KalmanGain(Sensor);
  estimate_update(Sensor);

  if (gps.satellites.value() >2 ){
    int Sensor = 2;
    predict();
    KalmanGain(Sensor);
    estimate_update(Sensor);
  }

  if (abs(accel.z())<0.5){
    int Sensor = 3;
    predict();
    KalmanGain(Sensor);
    estimate_update(Sensor);
  }

  Serial.println("The height of the rocket is")
  Serial.println(height);
  Serial.println("---------------------------------");
}
