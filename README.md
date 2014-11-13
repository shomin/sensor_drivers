BoschSensortec sensor support by API files
===================

| Sensor names  | API files to use | 
| :------------: |:---------------:|
| BMA220,280,255,250E,222E  | BMA2x2 |
|BMG160   |   BMG160|
|BMI055   |   BMG160 and BMA2x2|
|BMX055  |   BMG160 and BMA2x2 and BMM050|
|BMC150, 156  | BMM050 and BMA2x2|
|BMM150   |   BMM050|
|BMP180   |   BMP180|
|BMP280   |   BMP280|
|BME280   |   BME280|
|BNO055   |   BNO055|

Please refer the following points while using our BST sensor drivers.
-----
Integration of BST drivers:
###
	- Integrate sensor.h and sensor.c file in to your project.
	- Don't integrate sensor_support.c file since it contains only examples for API use cases 

sensor.h file:
###
	- The header file have the register address definition, constant definitions
	- The header file also have data type definition and function declarations.
	- You can find the required sensor driver calls for your application. 

sensor.c file:
###
	- This file contains the implementation for all the sensor driver APIs.
	
sensor_support.c
###
	- This file shall be used as an user guidance, here you can find samples of
		- Initialize the sensor with I2C/SPI communication
			- Add your code to the SPI and/or I2C bus read and bus write functions.
				- Return value can be chosen by yourself
				- API just passes that value to your application code
			- Add your code to the delay function
			- Change I2C address accordingly in sensor.h
		- Power mode configuration of the sensor
		- Get and set functions usage
		- Reading the sensor read out data

