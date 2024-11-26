
#Underwater ROV Project
This project implements an underwater remotely operated vehicle (ROV) with various features including motor control, sensor readings, and video streaming.

---

Features
1. Motor Control:
   - Horizontal and vertical motor control for movement in all directions.
   - Joystick-controlled movement with speed and direction adjustments.
   - ESC calibration for motor control precision.

2. Sensor Integration:
   - pH and turbidity sensor readings using MCP3008 ADC.
   - Real-time sensor data processing and display.

3. Video Streaming:
   - Live video feed from a connected USB webcam, streamed through a Flask web server.
   - Overlaid sensor data (pH and turbidity) on the video feed.

4. OLED Display:
   - Displays runtime and sensor data on an OLED screen.
   - Provides visual feedback on motor calibration and operation.

5. Servo Control:
   - Servo motor adjustments based on potentiometer input.

---

Prerequisites
- Raspberry Pi with Raspbian OS installed.
- Python libraries:
  - Flask
  - pigpio
  - Adafruit SSD1306
  - OpenCV
  - spidev
  - PIL (Pillow)
- Hardware components:
  - MCP3008 ADC
  - Joysticks
  - ESCs (Electronic Speed Controllers)
  - Motors
  - Sensors (pH and turbidity)
  - OLED display
  - USB webcam

---

Setup Instructions
1. Hardware Wiring:
   - Connect MCP3008 ADC to Raspberry Pi's SPI interface.
   - Wire the ESCs to GPIO pins as defined in the code.
   - Connect sensors and joystick to the ADC channels.

2. Install Required Libraries:
   
   pip install flask pigpio adafruit-circuitpython-ssd1306 opencv-python pillow spidev
   

3. Run the Application:
   - Start the Flask server:
    
     python your_script_name.py
     
   - Open the live video feed in your browser at `http://<raspberry_pi_ip>:5000/video_feed`.

4. Motor Calibration:
   - Ensure joysticks are at the zero position before starting.
   - Follow on-screen OLED instructions for ESC calibration.

5. Control the ROV:
   - Use the joysticks to control the ROV's movement.
   - Adjust servo angle with the potentiometer.


How It Works
- Video Feed: Captures frames from the webcam and overlays real-time sensor readings.
- Sensor Data: Reads analog data from the sensors via MCP3008, processes it into pH and turbidity values.
- Motor Control: Maps joystick ADC values to motor speed and direction.
- OLED Display: Shows sensor data and runtime information.



Note
- Ensure proper waterproofing of all electronics.
- Test the ROV in a controlled environment before deployment.
- for controller we have design a custum pcb to control the rov
---

Let me know if you still need help styling this or making further edits!"# rpi-based-underwater-rov" 
