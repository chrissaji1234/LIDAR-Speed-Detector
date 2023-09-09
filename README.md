# LIDAR-Speed-Detector

Created By: Christopher Saji

## Project Description

This project uses a Benewake TF03 LIDAR Range Sensor in conjunction with a STM32 NUCLEO-L4R5ZI to read the distance measurements of an object and to process the measurements into speed. This device is useful for measuring the speed of moving objects such as a vehicle.

#### Disclaimer: Use of LIDAR Speed Measurement Device

This project is primarily intended for experimental purposes and provides a proof of concept for using LIDAR to measure the speed of moving objects. While this device demonstrates reasonable accuracy in speed measurement, it is essential to note that this project is not certified for legal use by law enforcement for determining speed in the court of law.

## LIDAR Sensor

The Benewake TF03 LIDAR Sensor is a single point LIDAR range sensor that measures distance based on the Pulse Time of Flight Principle. To measure distance, the sensor emits a narrow pulse laser, which is then reflected by the object back into the receiving portion of the sensor. The time between the transmitted signal and the received signal is measured and the distance from the object is calculated using the measurement and the speed of light. By taking and comparing different distance samples, the sensor along with a microcontroller can calculate how fast a target is moving.

To read distance measurements from the TF03, the sensor utilizes a UART or CAN communication interface. This sensor has a maximum range of 180m and has built in algorithms for minimizing distortion in suboptimal conditions. For this project, the sensor communicates with UART, using a Baud Rate of 115200. This parameter and many more can be configured to read at different rates. The TF03 outputs a data frame of 9 Bytes, with the first two bytes of the data frame representing the frame header and contains the hex values of 0x59. The next two bytes contain distance data and the terminating byte containing a checksum to verify the integrity of the data.

A USB to TTL Serial Adapter was used in the initial testing of the sensor with Benewake's proprietary software. This software allows the user to easily change certain parameters of the sensor by transmitting different hex codes to the sensor via UART.

Since the sensor didn't come with a connect to interface with the L4R5ZI, I had to purchase one from Molex and solder Dupont connectors to the ends in order to interface with the L4R5ZI.

## Design

To program this project, I used STM's Cube IDE and CubeMX code generation software.

### Pin Initialization

Using CubeMX, I initialized pin PA2 as UART_TX and PA3 as UART_RX for the LIDAR sensor. I configured the baud rate to be 115200 bits, a word length of 8 bits, no parity, and 1 stop bit. These configurations match the settings of the LIDAR sensor from the documentation. I also initialized PG8 and PG7 for STLINK_RX and STLINK_TX for UART. This is for debugging purposes with a terminal. For the LCD, I configured PB9 and PB8 as SDA and SCL lines for I2C usage. Lastly, I setup pins PC0 and PC3 for the green LED and red LED respectively. 

### readLidarSensor

This function reads data directly from the LIDAR sensor over UART. First, the function initializes the variable dist, which stores the received distance value from the sensor as an integer. Next, the buffer that stores the data from the previous UART received transmission is cleared. Using the STM's HAL function, the data from the sensor is transferred over UART. The function checks if the data received is correct by verifying if the first two bytes of the frame header is 0x59. If it is, the third and fourth bytes are combined and shifted to form a 16-bit distance value. The function also checks if the distance is greater than 18000 cm. If it is the red LED will be illuminated and the green LED will turn off by the HAL functions, and the distance value will be replaced with a 0. Otherwise, the green LED will only be illuminated and the distance value will remain the same. 

### speedCalculate

This function calculates the speed of the moving object. The function begins by calling readLidarSensor and storing the returned distance value into distOne. After a delay of 100ms, the function calls redaLidarSensor again and stores the returned distance into distTwo. The function then subtracts the two distances and computes the absolute value of the result to prevent any negative values. The speed of the object is then calculated by dividing the calculated distance by 0.1, which the time elapsed in seconds between the distance readings. That value is divided by a factor of 44.704 to convert the result into miles per hour. Finally, the function stores the speed and distance values into a string buffer for the LCD.

## Project Functionality

This system constantly takes distance measurements of the object the sensor is facing towards. To achieve the most accurate results, make sure the sensor is not used in heavily sunlight or in foggy/rainy conditions. In addition to this, it is integral that the sensor is placed on a stable surface on not held in the hands. This is because the TF03 has a sensitivity of +/- 1 cm, and the slightest movement can result in erroneous results. To help with object tracking, the system has two LEDs that light red when the object is out of range or not in frame and green when the sensor is tracking something. On the first row of the LCD, the speed of the object is reported in miles per hour. The second row displays the distance measurements. When aiming the LIDAR sensor, make sure the sensor hits the most reflective part of the object to achieve the most accurate results.


## Photos

### Object Out of Range (Sensor Pointing to Sky)

![Out of Range](https://github.com/chrissaji1234/LIDAR-Speed-Detector/blob/master/Photos/Out-of-Range.jpg)

### Tracking Moving Vehicle (Moving Away)

![Tracking Moving Vehicle](https://github.com/chrissaji1234/LIDAR-Speed-Detector/blob/master/Photos/Tracking-Car.jpg)

## References

### [NHTSA LIDAR Documentation](https://www.nhtsa.gov/sites/nhtsa.gov/files/documents/lidar_participant_manual-smd-2018.pdf)

### [STM32 L4R5ZI Pin Outs](https://www.st.com/resource/en/user_manual/um2179-stm32-nucleo144-boards-mb1312-stmicroelectronics.pdf#page=35)


### [STM32 L4 UART Documentation](https://www.st.com/resource/en/product_training/stm32l4_peripheral_usart.pdf)


### [STM32 L4 MCU Documentation](https://www.st.com/resource/en/reference_manual/dm00310109-stm32l4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

  

### [STM32 L4 HAL Documentation](https://www.st.com/resource/en/user_manual/um1884-description-of-stm32l4l4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)

  

### [Benewake TF03 LIDAR Range Sensor Documentation](https://www.3dxr.co.uk/downloads/1639567195TF03_UART-CAN_User_Manual_V1.4.6.pdf)

  

### [1602 LCD Display Documentation](https://controllerstech.com/i2c-lcd-in-stm32/)