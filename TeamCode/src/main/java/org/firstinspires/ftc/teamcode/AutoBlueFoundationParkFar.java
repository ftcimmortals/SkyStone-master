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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * {@link AutoBlueFoundationParkFar} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "B-F-Far", group = "Concept")
//@Disabled                            // Comment this out to add to the opmode list
public class AutoBlueFoundationParkFar extends CommonMethods {

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
    public void runOpMode() {
        // game controller #1
        Hardware hardware = new Hardware(hardwareMap);
        
        hardware.armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        hardware.armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
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



        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        hardware.clawFingersServo.setPosition(FINGERS_OPEN);
        hardware.clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        hardware.imu.initialize(parameters);

        // Wait until we're told to go
        waitForStart();
        runtime.reset();

        angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = hardware.imu.getGravity();

        // Start the logging of measured acceleration
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        if (opModeIsActive()) {
            double startAngle = getAngle(hardware);

            moveForwardInches(0.3,false,13, hardware);
            moveSideInches(0.3,false,23, hardware);
            moveSideInches(0.2,false,10, hardware);
            sleep(1000);
            hardware.foundationGrabberServo.setPosition(FOUNDATION_GRABBER_DOWN);
            sleep(1000);
            PIDsideInches(0.02, 0.00045, 0.00009, 0.2, 1, 60, startAngle, hardware);
            sleep(1000);
            hardware.foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
            sleep(1000);
            moveForwardInches(0.3,true,39, hardware);
            moveSideInches(0.2,false,18, hardware);
            moveForwardInches(0.2,false,18, hardware);
            sleep(500);
            moveForwardInches(0.3,true,9, hardware);
            moveSideInches(0.3, false,9, hardware);
            moveForwardInches(0.3,true,22, hardware);
        }

    }

}