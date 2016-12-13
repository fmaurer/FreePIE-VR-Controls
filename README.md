# FreePIE-VR-Controls

Emulates HTC Vive Controllers with Arduino's, IMUs, and the Leap Motion sensor. Using the Leap Motion sensor for positional tracking of hands and orientation from an Arduino, HTC Vive controller data is spoofed by using FreePIE to generate Razer Hydra data. To simplify, the Arduino is flashed as a USB joystick and passes orientation data through the 16-bit joystick axis values, along with button presses.

Requirements:

- Tracked VR headset (Oculus DK2)
- FreePIE     http://andersmalmgren.github.io/FreePIE/
- Leap Motion sensor, Orion installed     https://api.leapmotion.com/orion
- Steam VR Razer Hydra Driver     http://store.steampowered.com/app/491380/
- Sixsense SDK     http://sixense.com/windowssdkdownload
- "... copy the sixense_fake.dll (located in the root folder of your FreePIE installation) over the sixense.dll in the folder of the application you want use this plugin with. "
- 2 x USB joysticks (i.e. Arduino's simulating joystick HID devices)
- recommended: xbox 360 controller for testing

Arduino:
- Firmware is written for Arduino Pro Micro. Different Arduinos may use different pins
- copy the libraries into your Arduino folder (default is ~/Documents/Arduino/libraries)
- Some can be obtained through the library manager in the Arduino IDE

(More install help: http://www.mtbs3d.com/phpBB/viewtopic.php?f=139&t=19753 )

How to Use:

- Run Steam VR
- Open FreePIE, run script

Support:

- Sixsense SDK has testing app under "Samples" folder of download
- The script assumes the Arduino-joysticks feeding the orientation data are index 0 and 1. Most problems can be addressed by change these indexes near Line 147: "joyLi" and "joyRi"d
- To calibrate, point the Arduino's IMU/orientation sensor forward and press left-thumbstick down on the 360 controller (change line 163 "if ((xbox360[i].leftThumb) ):" for different key)
- Still having issues? Have you tried Right-click > "Run as Administrator"?
- If all else fails, test if you can make it work with just a 360 controller: http://www.mtbs3d.com/phpbb/viewtopic.php?f=139&t=19270