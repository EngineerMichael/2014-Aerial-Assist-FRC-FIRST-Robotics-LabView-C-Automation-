2014-Aerial-Assist-FRC-FIRST-Robotics-LabView-C-Automation



This repository contains source code for the 2014 FRC Aerial Assist robot, which uses LabVIEW for control and C++ for autonomous behavior, along with integration for a 360-degree camera and various sensors.



LabVIEW Code



1. Robot Main.vi (Main Control Loop)



This VI (Virtual Instrument) serves as the central control for the robot, where it communicates with motors, sensors, and vision systems.



[Robot Main.vi]

-------------------------------------------------

| Start |   Teleop Mode  | Autonomous Mode | Stop |

-------------------------------------------------



- Initializes robot hardware (motors, sensors).

- Runs control loops for both teleoperated and autonomous modes.



Code snippet:

• Motor Control (Teleop):

• In the Teleop Mode, motors are controlled based on joystick input.



[Joystick Input (via Driver Station)] -> [Motor Control (via CAN or PWM)]



Example of motor control (for teleop):



Joystick X-Axis -> Drive Motor 1

Joystick Y-Axis -> Drive Motor 2





• Sensor Integration (Gyro, Encoders):

• Gyros provide the robot’s orientation.

• Encoders track wheel rotation for precise movement.



[Gyro Sensor] -> [Angle Calculation] -> [Drive Motor Control (compensating for orientation)]







2. Camera Interface.vi



This VI handles the connection to a 360-degree camera, processing visual data.



Camera Integration Example:

• LabVIEW interfaces with a camera to get image frames and process them for field awareness. The camera’s field of view would be split into quadrants, with different logic for detecting obstacles and objects.



[Camera Capture] -> [Image Processing] -> [Object Detection (ball, goal, obstacles)] -> [Control Feedback]



This VI processes the camera input, identifies objects, and feeds that data to the control loop to adjust the robot’s actions (e.g., move toward a detected object or avoid obstacles).



C++ Code



1. Autonomous.cpp (Autonomous Mode Logic)



The autonomous mode in C++ uses sensors, camera input, and predefined commands to control the robot without human input.



#include <WPILib.h>

#include <cmath>



// Global variables for motor controllers and sensors

TalonSRX leftMotor(0);

TalonSRX rightMotor(1);

Gyro gyro(0);  // Gyroscope to track orientation

CameraServer* camera = CameraServer::GetInstance();



// Example of autonomous routine

void AutonomousInit() {

    gyro.Reset();  // Reset gyro before starting

}



void AutonomousPeriodic() {

    double currentAngle = gyro.GetAngle();  // Get current robot orientation



    // Simple autonomous move forward routine

    if (currentAngle < 10) {  // Move forward if not too far from the start orientation

        leftMotor.Set(0.5);  // 50% power to left motor

        rightMotor.Set(0.5);  // 50% power to right motor

    } else {

        leftMotor.Set(0);  // Stop if orientation is incorrect

        rightMotor.Set(0);

    }

}



Autonomous Movement Logic:

• The robot moves forward until the gyro detects it has deviated too much from the starting orientation.

• A simple PID control loop could be added here to fine-tune movements and correct the robot’s path during autonomous mode.



2. BallDetection.cpp (Vision Processing for Ball Detection)



In FRC, vision processing is crucial for detecting game elements like balls, goals, or obstacles. This C++ code uses a camera feed to detect balls on the field.



#include <opencv2/opencv.hpp>

#include <WPILib.h>



// Initialize Camera

CameraServer* camera = CameraServer::GetInstance();

cv::VideoCapture cap(0);  // Camera stream (0 is default)



// Ball detection function

bool DetectBall() {

    cv::Mat frame;

    cap.read(frame);  // Capture frame from camera



    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);  // Convert to grayscale



    // Thresholding to detect bright objects (like the ball)

    cv::Mat thresholded;

    cv::threshold(frame, thresholded, 100, 255, cv::THRESH_BINARY);



    // Find contours (ball should appear as a distinct shape)

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(thresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);



    // If contours are found, ball detected

    if (contours.size() > 0) {

        return true;  // Ball detected

    } else {

        return false;  // No ball detected

    }

}



void AutonomousPeriodic() {

    if (DetectBall()) {

        // If the ball is detected, adjust robot's path or shoot

        leftMotor.Set(0.5);

        rightMotor.Set(0.5);

    } else {

        // If no ball detected, adjust robot position

        leftMotor.Set(-0.5);

        rightMotor.Set(-0.5);

    }

}



• This example uses OpenCV for image processing. It detects bright objects (such as a ball) by converting the image to grayscale and then thresholding it to detect contours.

• The robot adjusts its movement based on whether a ball is detected.



3. Drive.cpp (Motor Control and Movement)



The following code controls the robot’s movement in autonomous mode, using encoders or vision feedback to navigate.



#include <WPILib.h>



TalonSRX leftMotor(0);

TalonSRX rightMotor(1);

Encoder leftEncoder(0, 1);

Encoder rightEncoder(2, 3);



void DriveForward(double distance) {

    double initialPosition = leftEncoder.Get();

    double targetPosition = initialPosition + distance;  // Target position based on encoder counts



    // Move robot forward until the target position is reached

    while (leftEncoder.Get() < targetPosition) {

        leftMotor.Set(0.5);  // Move forward at 50% speed

        rightMotor.Set(0.5);

    }

    leftMotor.Set(0);  // Stop motors once the target is reached

    rightMotor.Set(0);

}



This C++ snippet drives the robot forward by using the encoder values to track the distance the robot has traveled. When the robot reaches the target distance, it stops.



License



This project is licensed under the MIT License - see the LICENSE file for details.



Contributing



If you’d like to contribute to this project, feel free to fork it and submit pull requests with enhancements or fixes.



For further questions or issues, feel free to create an issue on the GitHub repository
