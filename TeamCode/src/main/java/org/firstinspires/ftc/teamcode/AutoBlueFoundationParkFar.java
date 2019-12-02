package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link TestPID} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "B-F-Far", group = "Skystone")
//@Disabled                            // Comment this out to add to the opmode list

public class AutoBlueFoundationParkFar extends LinearOpMode {
    final private double FOUNDATION_GRABBER_DOWN = 0.40;    // grabber down
    final private double FOUNDATION_GRABBER_UP = 1;      // grabber up
    final private double FINGERS_OPEN = 0.05;               // open claw
    final private double FINGERS_CLOSED = 0.5;              //close claw
    final private double CAPSTONE_NOT_DROPPED = 1;
    final private double CAPSTONE_DROPPED = 0;
    private DcMotor frontLeftDriveMotor = null;
    private DcMotor frontRightDriveMotor = null;
    private DcMotor backLeftDriveMotor = null;
    private DcMotor backRightDriveMotor = null;
    private DcMotor armRotateMotor = null;
    private DcMotor armExtendMotor = null;
    private Servo stoneServo = null;                    // place holder
    private Servo clawWristServo = null;
    private Servo clawFingersServo = null;
    private Servo foundationGrabberServo = null;
    private DigitalChannel armLimitTouchFront = null;
    private DigitalChannel armLimitTouchBack = null;
    private Servo capstoneServo = null;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode (){
        // game controller #1
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDriveMotor = hardwareMap.get(DcMotor.class, "back_right_drive");

        // game controller #2
        armRotateMotor = hardwareMap.get(DcMotor.class, "arm_rotate_motor");
        armExtendMotor = hardwareMap.get(DcMotor.class, "arm_extend_motor");
        stoneServo = hardwareMap.get(Servo.class, "stone_picker");
        clawWristServo = hardwareMap.get(Servo.class, "claw_wrist");
        clawFingersServo = hardwareMap.get(Servo.class, "claw_fingers");
        foundationGrabberServo = hardwareMap.get(Servo.class, "foundation_grabber");

        armLimitTouchFront = hardwareMap.get(DigitalChannel.class, "arm_limit_touch_front");
        armLimitTouchBack = hardwareMap.get(DigitalChannel.class, "arm_limit_touch_back");
        capstoneServo = hardwareMap.get(Servo.class, "capstone_servo");
        armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);

        frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        armRotateMotor.setDirection(DcMotor.Direction.REVERSE);
        foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
        clawFingersServo.setPosition(FINGERS_OPEN);
        capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);

        waitForStart();
        runtime.reset();

        double voltage = this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        double timeMultiple = (-0.1*voltage) + 2.25;
        moveForwardTime(0.5, false, 0.7*timeMultiple);
        moveSideTime(0.5, false, 3*timeMultiple);
        moveStop();
        sleep(500);
        foundationGrabberServo.setPosition(FOUNDATION_GRABBER_DOWN);
        sleep(1000);
        moveSideTime(0.5, true, 1*timeMultiple);
        moveSideTime(1, true, 2.2);
        sleep(500);
        foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
        moveForwardTime(0.5, true, 2*timeMultiple);
        moveSideTime(0.5, false, 0.5*timeMultiple);
        moveForwardTime(0.5, false, 1.25*timeMultiple);
        moveStop();
        moveForwardTime(0.5, true, 0.5*timeMultiple);
        moveSideTime(0.5, false, 1.9*timeMultiple);

        int ticsPerDegree = (int) ((1425.2 *24)/360);
        int degrees = 132;

        armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotateMotor.setTargetPosition(ticsPerDegree * degrees);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor.setPower(1);
        sleep(5000);
        armRotateMotor.setPower(0.0);
        moveForwardTime(0.5, true, 1.2*timeMultiple);
    }

    public double moveForwardTime(double wheelPower, boolean direction, double movetime) {
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // direction true => forward
        // direction false => backward
        double timeNow = getRuntime();
        // use a factor for left wheel because the robot gets pulled to the left due to weight balance
        frontRightDriveMotor.setPower(0);
        frontLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);

        if (direction) {
            timeNow = getRuntime();
            while ((getRuntime() < timeNow + movetime) && (opModeIsActive())) {

                frontLeftDriveMotor.setPower(wheelPower);
                frontRightDriveMotor.setPower(wheelPower);
                backLeftDriveMotor.setPower(wheelPower);
                backRightDriveMotor.setPower(wheelPower);
            }

            frontLeftDriveMotor.setPower(0);
            frontRightDriveMotor.setPower(0);
            backLeftDriveMotor.setPower(0);
            backRightDriveMotor.setPower(0);
        } else {
            timeNow = getRuntime();
            while ((getRuntime() < timeNow + movetime) && (opModeIsActive())) {

                frontLeftDriveMotor.setPower(-wheelPower);
                frontRightDriveMotor.setPower(-wheelPower);
                backLeftDriveMotor.setPower(-wheelPower);
                backRightDriveMotor.setPower(-wheelPower);
            }

            frontLeftDriveMotor.setPower(0);
            frontRightDriveMotor.setPower(0);
            backLeftDriveMotor.setPower(0);
            backRightDriveMotor.setPower(0);
        }
        return (0);
    }
    public double moveSideTime(double wheelPower, boolean direction, double movetime) {
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // direction true => right
        // direction false => left
        double timeNow = getRuntime();
        // use a factor for left wheel because the robot gets pulled to the left due to weight balance
        frontRightDriveMotor.setPower(0);
        frontLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);

        if (direction) {
            timeNow = getRuntime();
            while ((getRuntime() < timeNow + movetime) && (opModeIsActive())) {

                frontLeftDriveMotor.setPower(wheelPower);
                frontRightDriveMotor.setPower(-wheelPower);
                backLeftDriveMotor.setPower(-wheelPower);
                backRightDriveMotor.setPower(wheelPower);
            }

            frontLeftDriveMotor.setPower(0);
            frontRightDriveMotor.setPower(0);
            backLeftDriveMotor.setPower(0);
            backRightDriveMotor.setPower(0);
        } else {
            timeNow = getRuntime();
            while ((getRuntime() < timeNow + movetime) && (opModeIsActive())) {

                frontLeftDriveMotor.setPower(-wheelPower);
                frontRightDriveMotor.setPower(wheelPower);
                backLeftDriveMotor.setPower(wheelPower);
                backRightDriveMotor.setPower(-wheelPower);
            }

            frontLeftDriveMotor.setPower(0);
            frontRightDriveMotor.setPower(0);
            backLeftDriveMotor.setPower(0);
            backRightDriveMotor.setPower(0);
        }
        return (0);
    }
    public double moveStop(){
        frontLeftDriveMotor.setPower(0);
        frontRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);

        return (0);
    }
    public double absolute(double inputval) {
        if (inputval > 0) {
            return inputval;
        } else {
            return -1 * inputval;
        }
    }
}
