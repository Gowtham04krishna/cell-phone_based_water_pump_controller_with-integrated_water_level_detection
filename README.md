# cell-phone_based_water_pump_controller_with-integrated_water_level_detection
cell-phone_based_water_pump_controller_with integrated_water_level_detection

This project is a water level detection system using STM32F401 and an ultrasonic sensor.
The setup includes an ultrasonic sensor, LEDs, a Bluetooth module (HC-05), and a water pump.
Water levels are displayed using LEDs: two yellow, two green, and one red.
As water reaches the first level, the first yellow LED lights up.
Each LED lights up sequentially as the water level increases.
Upon reaching the fifth level, the red LED blinks, signaling full capacity.
The HC-05 Bluetooth module connects to a phone, displaying water levels in real-time.
The phone is used to remotely control the water pump.
The water pump automatically stops when the final level is reached, indicated by the blinking red LED.
This system effectively combines automatic and remote control features for water level monitoring.
