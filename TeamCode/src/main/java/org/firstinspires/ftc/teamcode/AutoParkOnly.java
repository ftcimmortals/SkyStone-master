/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
/*
this autonomous mode just parks for a total of 5 points
 */

@Autonomous(name = "Park-Only", group = "Concept")
//@Disabled
public class AutoParkOnly extends CommonMethods {

    // The IMU sensor object
    BNO055IMU imu;
    //init runtime
    private ElapsedTime runtime = new ElapsedTime();

    //angles for imu
    Orientation angles;
    Acceleration gravity;


    @Override
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap);//init hardware

        //set mode of touch sensors
        hardware.armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        hardware.armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);
        //set direction of the motors
        hardware.frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.armRotateMotor.setDirection(DcMotor.Direction.REVERSE);
        //set servo position at init
        hardware.capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);
        hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
        hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
        hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
        hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
        hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_UP);
        hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_UP);
        hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
        hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);
        hardware.clawFingersServo.setPosition(FINGERS_OPEN);
        hardware.clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);
        //init imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        hardware.imu.initialize(parameters);
        //wait till start
        waitForStart();
        runtime.reset();
        //get angles and gravity values for imu
        angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = hardware.imu.getGravity();

        // Start the logging of measured acceleration
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // Loop and update the dashboard
        if (opModeIsActive()) {
            double angleStart = getAngle(hardware);//set starting angle as reference angle
            //park
            PIDstraightInches(GAIN_P, GAIN_I,GAIN_D, 0.2, -1, 16, angleStart, hardware);
        }
    }

   }