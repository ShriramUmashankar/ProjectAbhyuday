#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

Adafruit_BME280 bme;

//Pressure Moving avg
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

void setup(){
  Serial.begin(115200);

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
  
}

void loop()
{
  data_collector();
  float Altitude = getAltitude(press_avg, press_initial,temp_initial);
  Serial.printf("Altitude is: %f\n",Altitude);
  delay(50);
}
