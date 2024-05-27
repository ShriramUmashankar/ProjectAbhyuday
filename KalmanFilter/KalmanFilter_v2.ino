#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <BNO055_support.h>
#include <math.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

Adafruit_BME280 bme;
struct bno055_t myBNO;
struct bno055_accel myaccel;
struct bno055_mag mymag;
struct bno055_gyro mygyro;

//GPS
#define GPS_TX 16
#define GPS_RX 17
SoftwareSerial FC_GPS(GPS_RX,GPS_TX);
TinyGPSPlus gps;

uint64_t fc_time =0;
uint64_t init_time = esp_timer_get_time();
float t_prev = fc_time;
float initial_vel = 0;
float vel;

float height;
float initial_gps = 0;

int Sensor;
// 1 -- barometer
// 2 - gps
// 3 - accelrometer
//x = {{position},{velocity},{acceleration}}^(T)

BLA::Matrix<3,1> x; BLA::Matrix<3,1> x_inter; BLA::Matrix<3,1> x_prev = {0,0,0};
BLA::Matrix<3,3> P; BLA::Matrix<3,3> P_inter; BLA::Matrix<3,3> P_prev = {5,0,0,0,5,0,0,0,5};
BLA::Matrix<3,3> Q={10,0,0,0,8,0,0,0,2};BLA::Matrix<3,1> K;
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

#define PRESS_WIN 5
float press_arr[PRESS_WIN];
int press_index=0;
float press_avg=0;
float press_initial=0;

//Temp Moving avg
#define TEMP_WIN 20
float temp_arr[TEMP_WIN];
int temp_index=0;
float temp_avg=0;

float temp_initial=0;
float altitude=0;

float getAltitude(float pressure, float base_press, float base_temp) {
  return (float) (base_temp+273.15)/0.0065 * (1.0 - pow(pressure/base_press,0.1903));
}

void press_win_update(){
  float val=bme.readPressure();
  press_avg=press_avg+(val-press_arr[press_index])/PRESS_WIN;
  press_arr[press_index]=val;
  press_index=(press_index+1)%PRESS_WIN;
}
void temp_win_update(){
  float val=bme.readTemperature();
  temp_avg=temp_avg+(val-temp_arr[temp_index])/TEMP_WIN;
  temp_arr[temp_index]=val;
  temp_index=(temp_index+1)%TEMP_WIN;

}

void data_collector(){
  press_win_update();
  temp_win_update();
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


void predict(float dt){
  BLA::Matrix<3,3> A = {1,dt,dt*dt/2,0,1,dt,0,0,1}; BLA::Matrix<3,3> A_T = {1,0,0,dt,1,0,dt*dt/2,dt,1};
  x_inter = A*x_prev ;
  P_inter = A*P_prev*A_T + Q ;
}

void KalmanGain(int Sensor){
  if (Sensor == 1){
    H = {1,0,0};
    H_T= {1,0,0};
    R = {2.0602};
  }

  if (Sensor == 2){
    H = {1,0,0};
    H_T= {1,0,0};
    R = {10};
  }

  if (Sensor == 3){
    H = {0,0,1};
    H_T= {0,0,1};
    R = {0.0029};
  }
  BLA::Matrix<1,1> intermediate = H*P_inter*H_T + R ;
  K = P_inter*H_T*Inverse(intermediate);
  
}

void estimate_update(int Sensor){
  if(Sensor == 1){
    data_collector();
    float Altitude = getAltitude(press_avg, press_initial,temp_initial);
    z = {Altitude};
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
  Wire.begin();
  
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_CONFIG);
  delay(30);
  bno055_set_accel_range(ACCEL_RANGE_16G);
  delay(30);  
  bno055_set_operation_mode(OPERATION_MODE_AMG);
  
   while (!bme.begin(0x76)){
    Serial.println("BME not found");
    delay(1000);
  }
  bme.setSampling(bme.MODE_NORMAL,bme.SAMPLING_X16,bme.SAMPLING_X16,bme.SAMPLING_NONE,bme.FILTER_X16,bme.STANDBY_MS_0_5);

  //Initialising moving avg
  for(int i=0;i<2*PRESS_WIN;i++) press_win_update();
  press_initial=press_avg;
  
  for(int i=0;i<2*TEMP_WIN;i++) temp_win_update();
  temp_initial=temp_avg;

  FC_GPS.begin(9600);

  delay(1000);
  init_gps();

}

void loop(void) 
{
  fc_time = esp_timer_get_time() - init_time;
  float dt = float(fc_time-t_prev)/1000000 ;

  smartDelay(100);
  bno055_read_accel_xyz(&myaccel);

  int Sensor = 1;
  predict(dt);
  KalmanGain(Sensor);
  estimate_update(Sensor);

  if (gps.satellites.value() >2 ){
    int Sensor = 2;
    predict(dt);
    KalmanGain(Sensor);
    estimate_update(Sensor);
  }

  if (abs(float(myaccel.z)/100)<0.5){
    int Sensor = 3;
    predict(dt);
    KalmanGain(Sensor);
    estimate_update(Sensor);
  }

  t_prev = fc_time;

  vel = initial_vel + (float(myaccel.z)/100 - 9.81)*dt;
  initial_vel = vel;
  Serial.printf("The height and velocity of the rocket is %f,%f\n",height,vel);
  Serial.println("---------------------------------");
}
