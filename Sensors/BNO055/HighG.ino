#include <BNO055_support.h>
#include <Wire.h>

struct bno055_t myBNO;
struct bno055_accel myaccel;
struct bno055_mag mymag;
struct bno055_gyro mygyro;

void setup() {
  Wire.begin();
  
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_CONFIG);
  delay(30);
  bno055_set_accel_range(ACCEL_RANGE_16G);
  delay(30);  
  bno055_set_operation_mode(OPERATION_MODE_AMG);

  Serial.begin(115200);

}

void loop() {
  bno055_read_accel_xyz(&myaccel);
  bno055_read_mag_xyz(&mymag);
  bno055_read_gyro_xyz(&mygyro);

  Serial.printf("Accelerometer: %f %f %f\n",float(myaccel.x)/100,float(myaccel.y)/100,float(myaccel.z)/100);
  Serial.printf("Magnetometer: %f %f %f\n",float(mymag.x),float(mymag.y),float(mymag.z));
  Serial.printf("Gyroscope: %f %f %f\n",float(mygyro.x),float(mygyro.y),float(mygyro.z));
  Serial.println();
  delay(100);
}
