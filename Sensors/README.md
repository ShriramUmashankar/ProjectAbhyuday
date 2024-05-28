## BME280
The [final](https://github.com/ShriramUmashankar/ProjectAbhyuday/blob/main/Sensors/BME280/Altitude.ino) code file is an upgrade to the basic [code](https://github.com/ShriramUmashankar/ProjectAbhyuday/blob/main/Sensors/BME280/basic.ino) file in the following senses
1. The height calculated by the library uses preset sea level pressure and temperature data which is not the base at every location hence the new code takes in initial pressure and temperature readings and then calculates the height.
2. The sampling rate of humidity is set to 0 as we do not need it.
3. The sampling rate for pressure and temperature is set to maximum possible value.
4. The inbuilt IIR filter is being applied to the data.

## BNO055
For the BNO055 Sensor . The capability is set to a max of 4g's . The rocket having an initial acceleration of around 8g's would need the sensor to operate at 16g's mode. Thus the [final](https://github.com/ShriramUmashankar/ProjectAbhyuday/blob/main/Sensors/BNO055/HighG.ino) code file takes care of it.

## Lora E32

