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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
/*
This blue autonomous mode turns the foundation into the building site and parks for a total of 15 points
 */
@Autonomous(name= "Blue-F-Turn", group="Sky autonomous")
//@Disabled
public class AutoBlueFoundationTurn extends CommonMethods {
    //init runtime
    private ElapsedTime runtime = new ElapsedTime();

    //set angles for imu
    Orientation angles;
    Acceleration gravity;

    //variable for distance from foundation
    double foundationDistance;

    @Override
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap);//init all the hardware
        //set motor directions
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
        hardware.clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);
        hardware.clawFingersServo.setPosition(FINGERS_OPEN);
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

        //get imu angles and gravity measurement
        angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = hardware.imu.getGravity();

        // Start the logging of measured acceleration
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        if (opModeIsActive()) {
            double angleStart = getAngle(hardware);//get reference angle

            //move forward and align with foundation
            PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 12, angleStart, hardware);
            PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.5, -1, 30, angleStart, hardware);
            foundationDistance = hardware.sensorLeft.getDistance(DistanceUnit.INCH);
            PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.15, -1, foundationDistance + 2, angleStart, hardware);
            //pull and turn foundation
            hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_DOWN);
            hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_DOWN);
            sleep(1000);
            PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.5, 1, 8, angleStart, hardware);
            moveArchDegrees(0.5, 45, 20, 1, -1, hardware);
            moveForwardInches(0.5, true, 10, hardware);
            moveSideInches(0.5, false, 20, hardware);
            moveArchDegrees(0.5, 45, 20, -1 , 1, hardware);
            hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_UP);
            hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_UP);
            //park
            moveForwardInches(0.5, false, 15, hardware);
            PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.5, 1, 25, angleStart + 90, hardware);
            PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.5, 1, 44, angleStart + 90, hardware);
        }
    }

}