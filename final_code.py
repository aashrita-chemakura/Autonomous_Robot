import RPi.GPIO as gpio
import time
import numpy as np
import serial as ser
import cv2
import imutils
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
# For Email method
import os
from datetime import datetime
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from tkinter.messagebox import NO

import imaplib

class Robot:
    def __init__(self, monitor_encoders=False, monitor_imu=False, debug_mode=False,
                 start_x=0.2096, start_y=0.2096, monitor_pose=False,
                 serial_port='/dev/ttyUSB0', baud_rate=9600):
        
        # Encoder monitoring
        self.is_encoder_monitoring_enabled = monitor_encoders
        
        # Debug mode
        self.is_debug_mode_enabled = debug_mode
        
        # Pose monitoring
        self.is_pose_monitoring_enabled = monitor_pose
        
        # Motor pins
        self.motor_frequency = 50
        self.motor_duty_cycle = 50  # Controls speed
        self.turning_duty_cycle = 45  # Controls turn speed
        self.left_back_motor_pin = 31
        self.left_front_motor_pin = 33
        self.right_back_motor_pin = 35
        self.right_front_motor_pin = 37

        # Encoder pins
        self.left_encoder_pin = 7
        self.right_encoder_pin = 12

        # Gripper pins
        self.servo_frequency = 50
        self.open_servo_duty_cycle = 3
        self.close_servo_duty_cycle = 10.25
        self.servo_pin = 36

        # Distance sensor pins
        self.trig_pin = 16
        self.echo_pin = 18

        # PWM signals
        self.left_pwm_reverse = 0
        self.right_pwm_reverse = 0
        self.left_pwm = 0
        self.right_pwm = 0
        self.gripper_pwm = 0

        self.initialize_gpio()

        # IMU sensor
        self.serial_port = serial.Serial(serial_port, baud_rate)
        self.is_imu_monitoring_enabled = monitor_imu
        self.imu_angle = 0
        self.imu_margin = 3

        # Localization
        self.start_x = start_x  # Meters
        self.start_y = start_y  # Meters
        self.current_x = self.start_x
        self.current_y = self.start_y
        self.goal_x = 1.50  # Meters
        self.goal_y = 0.2  # Meters
        self.position_history = [(self.start_x, self.start_y)]

        # PID terms
        self.time_step = 0.1
        self.previous_error = 0
        self.integral_error = 0
        self.min_output_value = 0
        self.max_output_value = 2

        self.proportional_gain = 0.5
        self.integral_gain = 0.0
        self.derivative_gain = 0.0

        # Robot properties
        self.gear_ratio = 120 / 1  # 1:120
        self.wheel_diameter = 0.065  # Meters
        self.wheel_base = 0.1397  # Meters
        self.encoder_ticks_per_rev = 20
        self.drive_constant = self.encoder_ticks_per_rev \
            / (2 * np.pi * (self.wheel_diameter / 2))
        self.turning_perimeter = 2 * np.pi * self.wheel_base

def initialize_gpio(self):
    gpio.setmode(gpio.BOARD)

    # Left Motor pins
    lb_motor_pin = self.lb_motor_pin
    lf_motor_pin = self.lf_motor_pin
    gpio.setup(lb_motor_pin, gpio.OUT)  # IN1
    gpio.setup(lf_motor_pin, gpio.OUT)  # IN2

    # Right Motor pins
    rb_motor_pin = self.rb_motor_pin
    rf_motor_pin = self.rf_motor_pin
    gpio.setup(rb_motor_pin, gpio.OUT)  # IN3
    gpio.setup(rf_motor_pin, gpio.OUT)  # IN4

    # Encoder pins
    left_encoder_pin = self.left_encoder_pin
    right_encoder_pin = self.right_encoder_pin
    gpio.setup(left_encoder_pin, gpio.IN, pull_up_down=gpio.PUD_UP)  # (Left) Setup pin 7 encoder
    gpio.setup(right_encoder_pin, gpio.IN, pull_up_down=gpio.PUD_UP)  # (Right) Setup pin 12 encoder

    # Servo pins
    servo_pin = self.servo_pin
    gpio.setup(servo_pin, gpio.OUT)
    gpwm = gpio.PWM(servo_pin, self.servo_frequency)
    gpwm.start(self.open_servo_duty_cycle)

    # Distance sensor pins
    trig_pin = self.trig
    echo_pin = self.echo
    gpio.setup(trig_pin, gpio.OUT)
    gpio.setup(echo_pin, gpio.IN)
    gpio.output(trig_pin, False)

def game_over(self):
    lb_motor_pin = self.lb_motor_pin
    lf_motor_pin = self.lf_motor_pin
    rb_motor_pin = self.rb_motor_pin
    rf_motor_pin = self.rf_motor_pin
    servo_pin = self.servo_pin
    trig_pin = self.trig

    gpio.output(lb_motor_pin, False)
    gpio.output(lf_motor_pin, False)
    gpio.output(rb_motor_pin, False)
    gpio.output(rf_motor_pin, False)
    gpio.output(servo_pin, False)
    gpio.output(trig_pin, False)

def monitor_encoders(self, action, state_br, state_fl):
    filename1 = action + "_BR_encoder_states.txt"
    br = open(filename1, mode="a")
    filename2 = action + "_FL_encoder_states.txt"
    fl = open(filename2, mode="a")
    br_out_string = str(state_br) + '\n'
    fl_out_string = str(state_fl) + '\n'
    br.write(br_out_string)
    fl.write(fl_out_string)

def monitor_imu(self, updated_angle):
    filename = "imu_data.txt"
    file = open(filename, 'a')
    out_string = str(updated_angle) + '\n'
    file.write(out_string)

def distance(self):
    trig_pin = self.trig
    echo_pin = self.echo

    # Ensure output has no value
    gpio.output(trig_pin, False)
    time.sleep(0.01)

    # Generate trigger pulse
    gpio.output(trig_pin, True)
    time.sleep(0.00001)
    gpio.output(trig_pin, False)

    # Generate echo time signal
    while gpio.input(echo_pin) == 0:
        pulse_start = time.time()

    while gpio.input(echo_pin) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    # Convert time to distance in [cm] using speed of sound
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    print('Distance Sensor Reading:', distance)
    return distance

def read_imu(self, buffer=False):
    count = 0

    while True:
        if ser.in_waiting > 0:
            count += 1
            # Read serial stream
            line = ser.readline()

            # Avoid first n-lines of serial information
            if count > 0:
                # Strip serial stream of extra characters
                line = line.strip()
                line = line.decode('utf-8')
                line = line.strip("X: ")

                try:
                    line = float(line)
                    if buffer:
                        return line
                    else:
                        return line, count

                except ValueError:
                    pass
                    break

def calculate_distance_from_width(width):
    known_width = 0.0381  # Width of known bounding box and known_dist Meters
    box_width = width  # In pixels
    known_pix_width = 20  # pixels
    known_dist = 1  # Meter
    focal_length = (known_pix_width * known_dist) / known_width  # Meters

    distance = (known_width * focal_length) / box_width
    distance += 0.0762  # Meters
    return distance

def close_gripper(gpwm, close_servo_duty_cycle):
    gpwm.ChangeDutyCycle(close_servo_duty_cycle)
    time.sleep(1.5)

def open_gripper(gpwm, open_servo_duty_cycle):
    gpwm.ChangeDutyCycle(open_servo_duty_cycle)
    time.sleep(1.5)

def stop_driving(self):
    self.lpwm.ChangeDutyCycle(0)
    self.rpwm.ChangeDutyCycle(0)
    self.lpwmrev.ChangeDutyCycle(0)
    self.rpwmrev.ChangeDutyCycle(0)

    motor_pins = [self.lb_motor_pin, self.lf_motor_pin, self.rb_motor_pin, self.rf_motor_pin]
    for pin in motor_pins:
        gpio.output(pin, False)

def forward(self, distance):
    encoder_ticks = int(self.drive_constant * distance)
    proportional_gain = 21

    self.activate_left_wheel()
    self.activate_right_wheel()

    counter_right = np.uint64(0)
    counter_left = np.uint64(0)

    button_right = int(0)
    button_left = int(0)

    print('Duty Cycle:', self.motor_dut_cycle)

    while True:
        state_right = gpio.input(self.right_encoder_pin)
        state_left = gpio.input(self.left_encoder_pin)

        if self.monitor_encoders:
            self.monitor_encoders('Forward', state_right, state_left)

        if int(state_right) != int(button_right):
            button_right = int(state_right)
            counter_right += 1

        if int(state_left) != int(button_left):
            button_left = int(state_left)
            counter_left += 1

        if counter_right >= encoder_ticks:
            self.rpwm.ChangeDutyCycle(0)

        if counter_left >= encoder_ticks:
            self.lpwm.ChangeDutyCycle(0)

        error = counter_right - counter_left
        speed_update_l = self.motor_dut_cycle + error * proportional_gain
        speed_update_r = self.motor_dut_cycle + error * proportional_gain
        speed_update_l = max(10, min(60, speed_update_l))
        speed_update_r = max(10, min(60, speed_update_r))

        try:
            self.lpwm.ChangeDutyCycle(speed_update_l)
            self.rpwm.ChangeDutyCycle(speed_update_r)
        except:
            pass

        if counter_right == counter_left:
            self.lpwm.ChangeDutyCycle(speed_update_l)
            self.rpwm.ChangeDutyCycle(speed_update_r)

        if self.debug_mode:
            print('Goal:', encoder_ticks, 'R:', counter_right, 'L:', counter_left)

        if counter_right >= encoder_ticks and counter_left >= encoder_ticks:
            self.stop_driving()
            self.collect_pos(distance)
            break

def reverse(self, distance):
    encoder_ticks = int(self.drive_constant * distance)
    self.lpwm_rev.ChangeDutyCycle(self.motor_duty_cycle)
    self.rpwm_rev.ChangeDutyCycle(self.motor_duty_cycle)

    counter_right = np.uint64(0)
    counter_left = np.uint64(0)

    button_right = int(0)
    button_left = int(0)

    print('Duty Cycle:', self.motor_duty_cycle)

    while True:
        state_right = gpio.input(self.right_encoder_pin)
        state_left = gpio.input(self.left_encoder_pin)

        if self.monitor_encoders:
            self.monitor_encoders('Reverse', state_right, state_left)

        if int(state_right) != int(button_right):
            button_right = int(state_right)
            counter_right += 1

        if int(state_left) != int(button_left):
            button_left = int(state_left)
            counter_left += 1

        if counter_right >= encoder_ticks:
            self.rpwm_rev.ChangeDutyCycle(0)

        if counter_left >= encoder_ticks:
            self.lpwm_rev.ChangeDutyCycle(0)

        if counter_right > counter_left:
            speed_update = min(self.motor_duty_cycle * 2, 100)
            self.lpwm_rev.ChangeDutyCycle(speed_update)

        if counter_left > counter_right:
            speed_update = min(self.motor_duty_cycle * 2, 100)
            self.rpwm_rev.ChangeDutyCycle(speed_update)

        if counter_right == counter_left:
            self.lpwm_rev.ChangeDutyCycle(self.motor_duty_cycle)
            self.rpwm_rev.ChangeDutyCycle(self.motor_duty_cycle)

        if self.debug_mode:
            print('Goal:', encoder_ticks, 'R:', counter_right, 'L:', counter_left)

        if counter_right >= encoder_ticks and counter_left >= encoder_ticks:
            self.StopDriving()
            print('Stopped Driving!')
            self.CollectPos(-distance)
            print('Pose Collected!')
            break

def pivot(self, angle, direction):
    # Get initial IMU angle reading
    init_angle = self.imu_angle

    if init_angle < 0:
        init_angle += 360

    if direction == 'left':
        goal_angle = init_angle - angle
    elif direction == 'right':
        goal_angle = init_angle + angle

    flip = False
    cross = False
    if direction == 'left' and goal_angle > init_angle:
        cross = True
    elif direction == 'right' and goal_angle < init_angle:
        cross = True

    if direction == 'left' and goal_angle < 0:
        goal_angle += 360
    elif direction == 'right' and goal_angle > 360:
        goal_angle -= 360

    if init_angle - self.imu_margin < goal_angle < init_angle + self.imu_margin:
        return

    counterBR = np.uint64(0)
    counterFL = np.uint64(0)

    buttonBR = int(0)
    buttonFL = int(0)

    count = 0
    while True:
        updated_angle, count = self.ReadIMU()

        if direction == 'left':
            self.rpwm.ChangeDutyCycle(self.turn_dc)
            self.lpwmrev.ChangeDutyCycle(self.turn_dc)
        elif direction == 'right':
            self.rpwmrev.ChangeDutyCycle(self.turn_dc)
            self.lpwm.ChangeDutyCycle(self.turn_dc)

        stateBR = gpio.input(self.right_encoder_pin)
        stateFL = gpio.input(self.left_encoder_pin)

        # Save encoder states to txt files
        if self.monitor_encoders is True:
            self.MonitorEncoders('pivot', stateBR, stateFL)

        if self.monitor_imu is True:
            self.MonitorIMU(updated_angle)

        if int(stateBR) != int(buttonBR):
            buttonBR = int(stateBR)
            counterBR += 1

        if int(stateFL) != int(buttonFL):
            buttonFL = int(stateFL)
            counterFL += 1

        if self.debug_mode:
            print('Goal:', goal_angle ,'Angle:', updated_angle, 'Initial:', init_angle, 'Dutycycle:', self.turn_dc)

        low_thresh = goal_angle - self.imu_margin
        high_thresh = goal_angle + self.imu_margin

        # Break when the current angle is within a threshold of the goal angle
        if low_thresh < updated_angle < high_thresh:
            self.StopDriving()

            self.imu_angle = updated_angle
            break

def calculate_PID(self, target, present):
    error = target - present
    p_term = self.kp * error
    self.integral += (error * self.dt)
    i_term = self.ki * self.integral
    d_term = self.kd * (error - self.prev_error) / self.dt
    output = p_term + i_term + d_term
    self.prev_error = error
    output = max(self.min_value, min(self.max_value, output))
    return output

def colour_thresh(self, color='green'):
    if color == 'red':
        low_red = np.array([170, 105, 85])
        high_red = np.array([255, 255, 255])
        return low_red, high_red
    elif color == 'green':
        low_green = np.array([30, 52, 72])
        high_green = np.array([80, 255, 255])
        return low_green, high_green
    elif color == 'blue':
        low_blue = np.array([96, 161, 64])
        high_blue = np.array([255, 255, 255])
        return low_blue, high_blue

def find_block(self, color='green'):
    print('Finding Block', color)

    midline = 640 / 2
    pix_per_deg = 38.88 / 640

    x_center = 0
    distance = 0
    width = 0

    image = self.capture()
    image = cv2.flip(image, -1)

    # Converting to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    low_thresh, high_thresh = self.color_range(color)
    mask = cv2.inRange(hsv_image, low_thresh, high_thresh)

    # Perform Closing (morphology) to filter noise
    kernel = np.ones((15, 15), np.uint8)
    mask = cv2.dilate(mask, kernel)
    mask = cv2.erode(mask, kernel)

    # Find the contours
    cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if len(cnts) > 0:
        # find the biggest contour (c) by the area
        c = max(cnts, key=cv2.contourArea)
        rect = cv2.boundingRect(c)
        x, y, width, height = rect

        if width > 5:
            print('Found', color, 'block')
            x_center = int(x + width / 2)
            y_center = int(y + height / 2)
            center = (x_center, y_center)

            cv2.rectangle(image, (x, y), (x + width, y + height), color=(0, 255, 255), thickness=2)
            cv2.circle(image, center, 1, color=(0, 0, 255), thickness=4)

            distance = pix_per_deg * abs(x_center - midline)

    if width == 0:
        cv2.destroyAllWindows()
        print('Scanning for', color, 'block')
        time.sleep(4)
        self.left_piv(45)
        print('Turned 45')
        return self.find_block(color)

    if self.debug_mode:
        print('Bounding Box Width:', width)
        print('Distance:', distance)
        cv2.imshow('frame', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return x_center, distance, width

def capture_and_send_email(self):
    # Capture image
    capture_device = cv2.VideoCapture(0)
    fps = capture_device.get(cv2.CAP_PROP_FPS)
    timestamps = [capture_device.get(cv2.CAP_PROP_POS_MSEC)]
    ret, image = capture_device.read()
    capture_device.release()

    # Save image
    image_filename = datetime.now().strftime('%Y%m%d%H%M%S') + '.jpg'
    cv2.imwrite(image_filename, image)

    # Email information
    smtp_user = 'aashritarc@gmail.com'
    smtp_pass = 'bdqqdepfbfijuapd'

    # Destination email information
    to_add = ['ENPM809TS19@gmail.com']
    from_add = smtp_user
    subject = 'Image recorded at ' + datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    message = 'Image recorded at ' + datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    # Create email
    msg = MIMEMultipart()
    msg['Subject'] = subject
    msg['From'] = from_add
    msg['To'] = ', '.join(to_add)
    msg.preamble = subject

    # Email text
    body = MIMEText(message)
    msg.attach(body)

    # Attach image
    with open(image_filename, 'rb') as fp:
        img = MIMEImage(fp.read())
    msg.attach(img)

    # Send email
    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.ehlo()
    server.starttls()
    server.ehlo()
    server.login(smtp_user, smtp_pass)
    server.sendmail(from_add, to_add, msg.as_string())
    server.quit()

    print("Email delivered!")

def teleop(self):
    while True:
        key_press = input("Select Action: 'w' - Forward \n \
            's' - Backward \n \
            'a' - Pivot Left \n \
            'd' - Pivot Right \n \
            'o' - Open Gripper \n \
            'c' - Close Gripper \n \
            'p' - Exit Program \n")

        if key_press.lower() == 'p':
            break

        value = input("Distance to travel/Angle to turn: ")

        print('Key:', key_press)
        print('Distance/Angle:', value)

        distance = self.distance()

        if key_press.lower() == 'w':
            self.forward(float(value))
        elif key_press == 's':
            self.reverse(float(value))
        elif key_press == 'a':
            self.left_piv(float(value))
        elif key_press == 'd':
            self.right_piv(float(value))
        elif key_press == 'o':
            self.open_gripper()
        elif key_press == 'c':
            self.close_gripper()
        else:
            print('Invalid key pressed!!')

        self.game_over()

def navigate(self):
    commands = []
    while True:
        cmd = input('Type command in the format (direction, distance/angle) "q" to finish: ').split(', ')
        if cmd[0] == 'q' or cmd[1] == 'q':
            break
        cmd[1] = float(cmd[1])
        commands.append(cmd)

    for direction, value in commands:
        self.key_input(direction, value)
        time.sleep(1.5)


def g_localize(self):
    delta_x = self.goal_x - self.cur_x
    delta_y = self.goal_y - self.cur_y
    magnitude = np.sqrt((delta_x ** 2) + (delta_y ** 2))

    angle = np.arccos(delta_x / magnitude)
    angle = np.rad2deg(angle)

    angle += self.imu_angle

    if angle > 359:
        angle -= 360

    if self.debug_mode:
        print('Pos History:', self.pos_history)
        print('Move to Goal- Turn:', angle, 'Drive:', magnitude)

    return magnitude, angle


def collect_pos(self, distance):
    prev_x, prev_y = self.pos_history[-1]
    angle_rad = np.deg2rad(self.imu_angle)
    new_x = prev_x + (distance * np.cos(angle_rad))
    new_y = prev_y + (distance * np.sin(angle_rad))

    self.pos_history.append((new_x, new_y))
    self.cur_x = new_x
    self.cur_y = new_y

    if self.monitor_pose:
        print('New Pose: (x, y):', (new_x, new_y))
        # Save Pose data to txt file
        with open('pos_data.txt', 'a') as file, open('xpos_data.txt', 'a') as xfile, open('ypos_data.txt', 'a') as yfile:
            outstring = f"Drive {distance} pose: {new_x}, {new_y} orientation: {np.rad2deg(angle_rad)}\n"
            xoutstring = f"{new_x}\n"
            youtstring = f"{new_y}\n"
            file.write(outstring)
            xfile.write(xoutstring)
            yfile.write(youtstring)

def grand_challenge(robot, target_color, target_index):
    midline = 640 / 2

    while True:
        x_center, dist, box_width = robot.find_block(target_color[target_index])

        delta = midline - x_center
        print('Delta =', delta)

        if abs(delta) > 90:  # Object center is greater than 90 pixels
            direction, angle = robot.dist_from_center(x_center)
            print('Angle:', angle)
            if direction == 'Right':
                robot.right_pivot(angle)
            else:
                robot.left_pivot(angle)

        if dist > 0.2032:  # 8 inches
            robot.forward(dist / 4)
            continue
        else:
            robot.forward(dist)

        x_center, dist, box_width = robot.find_block(target_color[target_index])
        print('x_center, distance, box_width', x_center, dist, box_width)
        if box_width > 220:
            print('Block gripped!')
            robot.close_gripper()
            robot.capture_and_send_email()
            break

    goal_distance, goal_angle = robot.global_localize()
    time.sleep(4)
    robot.left_pivot(goal_angle)
    robot.forward(goal_distance)
    
if __name__ == '__main__':

    ser = ser.Serial('/dev/ttyUSB0', 9600)
    ser.reset_input_buffer()

    robot = Robot(monitor_encoders=False,
                  monitor_imu=False,
                  debug_mode=False,
                  monitor_pose=True,
                  ser=ser)

    lpwm = gpio.PWM(robot.lb_motor_pin,
                    robot.motor_frequency)

    rpwm = gpio.PWM(robot.rf_motor_pin,
                    robot.motor_frequency)

    lpwm.start(0)
    rpwm.start(0)

    lpwmrev = gpio.PWM(robot.lf_motor_pin,
                       robot.motor_frequency)

    rpwmrev = gpio.PWM(robot.rb_motor_pin,
                       robot.motor_frequency)

    lpwmrev.start(0)
    rpwmrev.start(0)

    robot.rpwm = rpwm
    robot.lpwm = lpwm
    robot.rpwmrev = rpwmrev
    robot.lpwmrev = lpwmrev

    robot.BufferIMU()

    repeat = 0
    color = ['red', 'green', 'blue']
    idx = 0

    robot.RightPiv(45)

    while repeat < 3:
        grand_challenge(robot, color, idx)
        open_gripper()
        reverse(0.25)
        pivot(90)
        robot.goal_x += 0.1524  

        idx += 1
        if idx == 3:
            repeat += 1
            idx = 0
    
    teleop()
    game_over()
    gpio.cleanup()

    lpwm.stop()
    rpwm.stop()
    robot.gpwm.stop()
    print('Grand Challenge Complete! Congrats on finishing your Robotics Masters Degree!')