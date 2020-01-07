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


@Autonomous(name = "DN-Close", group = "Concept")
//@Disabled                            // Comment this out to add to the opmode list
public class AutoDoNothingClose extends CommonMethods {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    WebcamName webcamName = null;

    // The IMU sensor object
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap);

        hardware.armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        hardware.armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);
        hardware.frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.armRotateMotor.setDirection(DcMotor.Direction.REVERSE);


        hardware.capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);
        hardware.stoneServoBlue.setPosition(STONE_PICKER_CLOSED_BLUE);
        hardware.stoneServoRed.setPosition(STONE_PICKER_CLOSED_RED);
        hardware.foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
        hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
        hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        hardware.clawFingersServo.setPosition(FINGERS_OPEN);
        hardware.clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);



        hardware.imu.initialize(parameters);


        waitForStart();
        runtime.reset();


        angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = hardware.imu.getGravity();

        // Start the logging of measured acceleration
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // Loop and update the dashboard
        if (opModeIsActive()) {
            double angleStart = getAngle(hardware);
            PIDstraightInches(0.02, 0.00045,0.00009, 0.5, -1, 72, angleStart, hardware);
        }
    }

   }