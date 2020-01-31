package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/*
This red autonomous mode moves the foundation without turning it and parks for a total of 15 points
 */
@Autonomous(name= "Red-F-NoTurn", group="Sky autonomous")
//@Disabled
public class AutoRedFoundationNoTurn extends CommonMethods {
    //init runtime
    private ElapsedTime runtime = new ElapsedTime();

    //angles for imu
    Orientation angles;
    Acceleration gravity;

    //distance from the foundation
    double foundationDistance;

    @Override
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap);//init the hardware

        //set motor directions
        hardware.frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.armRotateMotor.setDirection(DcMotor.Direction.REVERSE);

        //set servo positions at init
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

        //get angles and gravity values from the imu
        angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = hardware.imu.getGravity();

        // Start the logging of measured acceleration
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        if (opModeIsActive()) {
            double angleStart = getAngle(hardware);//get start angle as reference

            //align with foundation
            PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.3, -1, 10, angleStart, hardware);
            PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.5, -1, 30, angleStart, hardware);
            foundationDistance = hardware.sensorLeft.getDistance(DistanceUnit.INCH);
            PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.15, -1, foundationDistance + 2, angleStart, hardware);
            //grab and move foundation close to wall
            hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_DOWN);
            hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_DOWN);
            sleep(1000);
            moveArchDegrees(0.5, 30, 20, 1, -1, hardware);
            moveArchDegrees(0.5, 30, 20, 1 , 1, hardware);
            PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.5, 1, 20, angleStart, hardware);
            hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_UP);
            hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_UP);
            //park
            moveForwardInches(0.5, false, 2, hardware);
            sleep(500);
            PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.5, 1, 56, angleStart, hardware);
        }
    }

}