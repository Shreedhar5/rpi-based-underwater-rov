from flask import Flask, Response
import cv2
import time
import spidev
import threading
import pigpio
import board
import digitalio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
# Initialize Flask app
app = Flask(name)

# Initialize webcam using OpenCV
webcam = cv2.VideoCapture(0)  # 0 is the default index for the first connected USB camera

# Initialize the pigpio library
pi = pigpio.pi()

# Initialize OLED display
oled_reset = digitalio.DigitalInOut(board.D4)
i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3C)

# GPIO pins connected to the ESCs and servo
SERVO_PIN = 23
ESC_UP_DOWN_LEFT = 17   # GPIO pin for vertical motor left
ESC_UP_DOWN_RIGHT = 18  # GPIO pin for vertical motor right
ESC_FORWARD_BACKWARD_LEFT = 27  # GPIO pin for horizontal motor left
ESC_FORWARD_BACKWARD_RIGHT = 22  # GPIO pin for horizontal motor right

# MCP3008 Setup
class MCP3008:
    def init(self, spi_channel=0, spi_bus=0, max_speed_hz=1350000):
        self.spi_channel = spi_channel
        self.spi_bus = spi_bus
        self.spi = spidev.SpiDev()
        self.spi.open(self.spi_bus, self.spi_channel)
        self.spi.max_speed_hz = max_speed_hz

    def read_channel(self, channel):
        """Read data from the specified ADC channel."""
        if channel < 0 or channel > 7:
            raise ValueError("Channel must be between 0 and 7.")
        adc = self.spi.xfer2([1, (8 + channel) << 4, 0])
        data = ((adc[1] & 3) << 8) + adc[2]
        return data

    def close(self):
        """Close the SPI connection."""
        self.spi.close()


# Function to convert ADC value to voltage
def convert_to_voltage(adc_value, vref=3.3):
    return (adc_value * vref) / 1023.0

# Function to calculate pH from voltage
def get_ph(voltage):
    voltage_at_ph7 = 1.95  # Voltage corresponding to pH 7.0
    voltage_slope = (2.20 - 0.95) / (7 - 4)  # Adjust based on your sensor's calibration
    ph = 7.0 + ((voltage - voltage_at_ph7) / voltage_slope)
    return ph

# Function to calculate turbidity level based on voltage
def get_turbidity(voltage):
    if voltage >= 3.2:
        return "Clear"
    elif 2.0 <= voltage < 3.2:
        return "Moderately Clear"
    elif 0.2 <= voltage < 2.0:
        return "Low Clarity"
    else:
        return "Very Cloudy"

# Initialize MCP3008 for sensors
mcp3008 = MCP3008()

def read_sensors():
    """Read both pH and turbidity sensors."""
    # Reading pH sensor (Channel 0)
    adc_value_ph = mcp3008.read_channel(4)
    time.sleep(0.1)
    voltage_ph = convert_to_voltage(adc_value_ph)
    ph_value = get_ph(voltage_ph)
    
    # Reading turbidity sensor (Channel 1)
    adc_value_turbidity = mcp3008.read_channel(5)
    time.sleep(0.1)
    voltage_turbidity = convert_to_voltage(adc_value_turbidity)
    turbidity_level = get_turbidity(voltage_turbidity)
    
    return ph_value, turbidity_level

def generate_frames():
    while True:
        # Capture frame-by-frame from the USB webcam
        ret, frame = webcam.read()
        if not ret:
            break  # Exit if the webcam is not capturing frames

        # Get sensor readings
        ph_value, turbidity_level = read_sensors()

        # Overlay pH and turbidity values on the video frame
        text_ph = f"pH: {ph_value:.2f}"
        text_turbidity = f"Turbidity: {turbidity_level}"
        
        # Define text position and font
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, text_ph, (10, 30), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, text_turbidity, (10, 60), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

        # Encode the frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        # Yield the frame to Flask for live streaming
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "Video streaming: <a href='/video_feed'>Click here to see the stream</a>"

# Motor control functions start here

# oled start here

def display_text(lines):
    width = oled.width
    height = oled.height
    image = Image.new("1", (width, height))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    oled.fill(0)
    oled.show()

    for line in lines:
        draw.rectangle((0, 0, width, height), outline=0, fill=0)
        for i, subline in enumerate(line.split('\n')):
            (font_width, font_height) = font.getsize(subline)
            draw.text((width // 2 - font_width // 2, (height // 4) + i * font_height), subline, font=font, fill=255)
        oled.image(image)
        oled.show()
        time.sleep(2)

    oled.fill(0)
    oled.show()

# Calibration Function with Timer
calibration_start_time = 0

def display_rov_instructions():
    # Display first set of instructions
    display_text([
        "Instructions\nLeft Joystick:\nVertical motion"
    ])
    
    # Display second set of instructions
    display_text([
        "Right Joystick:\nForward &\nL/R motion"
    ])


def calibrate_all_escs():
    print("Starting calibration of all ESCs")
    global calibration_start_time
    display_text(["Calibration Start"])
    # Step 1: Set all ESCs to minimum throttle
    pi.set_servo_pulsewidth(ESC_UP_DOWN_LEFT, 1000)
    pi.set_servo_pulsewidth(ESC_UP_DOWN_RIGHT, 1000)
    pi.set_servo_pulsewidth(ESC_FORWARD_BACKWARD_LEFT, 1000)
    pi.set_servo_pulsewidth(ESC_FORWARD_BACKWARD_RIGHT, 1000)
    time.sleep(1.5)

    # Step 2: Set all ESCs to maximum throttle
    pi.set_servo_pulsewidth(ESC_UP_DOWN_LEFT, 2000)
    pi.set_servo_pulsewidth(ESC_UP_DOWN_RIGHT, 2000)
    pi.set_servo_pulsewidth(ESC_FORWARD_BACKWARD_LEFT, 2000)
    pi.set_servo_pulsewidth(ESC_FORWARD_BACKWARD_RIGHT, 2000)
    time.sleep(1.5)

    # Step 3: Return all ESCs to minimum throttle
    pi.set_servo_pulsewidth(ESC_UP_DOWN_LEFT, 1000)
    pi.set_servo_pulsewidth(ESC_UP_DOWN_RIGHT, 1000)
    pi.set_servo_pulsewidth(ESC_FORWARD_BACKWARD_LEFT, 1000)
    pi.set_servo_pulsewidth(ESC_FORWARD_BACKWARD_RIGHT, 1000)
    time.sleep(1.5)

    # Step 4: Stop all ESCs
    pi.set_servo_pulsewidth(ESC_UP_DOWN_LEFT, 0)
    pi.set_servo_pulsewidth(ESC_UP_DOWN_RIGHT, 0)
    pi.set_servo_pulsewidth(ESC_FORWARD_BACKWARD_LEFT, 0)
    pi.set_servo_pulsewidth(ESC_FORWARD_BACKWARD_RIGHT, 0)
    time.sleep(1)
    display_text(["Calibration End"])
    calibration_start_time = time.time() 
    print("All ESCs calibrated.")
    display_rov_instructions()
# Function to calculate and return runtime
def get_runtime():
    return time.time() - calibration_start_time


def update_oled(ph_value, turbidity_level, runtime):
    lines = [
        f"pH: {ph_value:.2f}",
        f"Turbidity: {turbidity_level}",
        f"Runtime: {runtime:.2f} sec"
    ]
    display_text(lines)


def set_motor_speed(gpio_pin, speed):
    """Set the motor speed using the servo pulse width."""
    pulsewidth = 1000 + (speed * 10)
    pulsewidth = max(1000, min(2000, pulsewidth))  # Clamping to valid range for ESCs
    pi.set_servo_pulsewidth(gpio_pin, pulsewidth)
    print(f"Motor on GPIO pin {gpio_pin} set to pulse width {pulsewidth} (Speed: {speed:.2f}%)")

def stop_all_motors():
    """Stop all motors by setting pulse width to 0."""
    pi.set_servo_pulsewidth(ESC_UP_DOWN_LEFT, 0)
    pi.set_servo_pulsewidth(ESC_UP_DOWN_RIGHT, 0)
    pi.set_servo_pulsewidth(ESC_FORWARD_BACKWARD_LEFT, 0)
    pi.set_servo_pulsewidth(ESC_FORWARD_BACKWARD_RIGHT, 0)
    print("All motors stopped.")

def convert_to_speed(adc_value, max_adc=1023, max_speed=100):
    """Convert ADC value to motor speed from 0 to 100."""
    return (adc_value / max_adc) * max_speed

def convert_vertical_speed(adc_value, max_adc=1023, max_speed=100):
    """Convert ADC value to vertical motor speed from 0 to 100."""
    return (adc_value / max_adc) * max_speed

def apply_underwaterrov_like_control(forward_backward_speed, turn_speed_adc, dead_zone=500):
    """Adjust horizontal motor speeds for underwater ROV-like control with consistent logic."""
    center_value = 512

    # Normalize turn speed to be within the range of -1 to 1
    if abs(turn_speed_adc - center_value) < dead_zone:
        turn_speed = 0
    else:
        turn_speed_normalized = (turn_speed_adc - center_value) / (1023 - center_value)
        turn_speed = turn_speed_normalized * 100

    # Calculate turn effect
    turn_effect = abs(turn_speed)

    # Adjust motor speeds based on turn direction
    if turn_speed == 0:
        left_motor_speed = forward_backward_speed
        right_motor_speed = forward_backward_speed
    else:
        if turn_speed > 0:
            # Right Turn
            left_motor_speed = forward_backward_speed + turn_effect
            right_motor_speed = forward_backward_speed - (turn_effect * 0.2)
        else:
            # Left Turn
            left_motor_speed = forward_backward_speed - turn_effect
            right_motor_speed = forward_backward_speed + (turn_effect * 0.2)

    # Clamp speeds to the range [0, 100]
    left_motor_speed = max(0, min(100, left_motor_speed))
    right_motor_speed = max(0, min(100, right_motor_speed))

    # Set the speed for horizontal motors
    set_motor_speed(ESC_FORWARD_BACKWARD_LEFT, left_motor_speed)
    set_motor_speed(ESC_FORWARD_BACKWARD_RIGHT, right_motor_speed)

def convert_to_angle(adc_value, max_adc=1023, max_angle=180):
    """Convert ADC value to servo angle from 0 to 180."""
    return (adc_value / max_adc) * max_angle

def set_servo_angle(gpio_pin, angle):
    """Set the servo angle using the pulse width."""
    pulsewidth = 1000 + (angle / 180) * 1000  # Convert angle to pulse width
    pi.set_servo_pulsewidth(gpio_pin, pulsewidth)
    print(f"Servo on GPIO pin {gpio_pin} set to angle {angle:.2f}°")  

def check_joystick_initially_zero():
    while True:
        joystick1_x = mcp3008.read_channel(0)  # VRx for turn
        joystick2_y = mcp3008.read_channel(3)  # VRy for vertical control

        if joystick1_x < 100 or joystick2_y < 100:
            print("Joysticks are at zero. Proceeding with calibration.")
            return
        else:
            print("Joysticks are not at zero. Waiting...")
            time.sleep(0.5)  # Check every 0.5 seconds

def joystick_control():
    try:
        # MCP3008 is already initialized at the global scope
        check_joystick_initially_zero()
        calibrate_all_escs()  # Calibrate ESCs

        while True:
            # Read joystick values
            joystick1_x = mcp3008.read_channel(0) 
            time.sleep(0.1) # VRx for turn
            joystick1_y = mcp3008.read_channel(1) 
            time.sleep(0.1) # VRy for forward/backward
            joystick2_y = mcp3008.read_channel(2)
            time.sleep(0.1)  # VRy for vertical control

            # Convert joystick values to motor speeds
            forward_backward_speed = convert_to_speed(joystick1_y)  # Forward/Backward speed
            turn_speed_adc = joystick1_x  # Turn speed
            vertical_speed = convert_vertical_speed(joystick2_y)  # Vertical speed

            # Apply underwater ROV-like control logic
            apply_underwaterrov_like_control(forward_backward_speed, turn_speed_adc)

            # Set vertical motor speeds
            set_motor_speed(ESC_UP_DOWN_LEFT, vertical_speed)
            set_motor_speed(ESC_UP_DOWN_RIGHT, vertical_speed)

            # Read the potentiometer value from channel 7 to control the servo angle
            pot_value = mcp3008.read_channel(3) 
            time.sleep(0.1) # Read the potentiometer value from channel 7
            servo_angle = convert_to_angle(pot_value)  # Convert the potentiometer value to an angle
            set_servo_angle(SERVO_PIN, servo_angle)  # Set the servo angle

            time.sleep(0.1)  # Small delay for stability
            runtime = get_runtime()
            update_oled(ph_value, turbidity_level, runtime)

    except KeyboardInterrupt:
        pass
    finally:
        stop_all_motors()
        pi.stop()  # Stop pigpio
        mcp3008.close()  # Close SPI connection

if name == 'main':
    joystick_thread = threading.Thread(target=joystick_control)
    joystick_thread.start()  # Start the joystick control in a separate thread

    try:
        app.run(host='0.0.0.0', port=5000)
    finally:
        webcam.release()  # Release the webcam resource