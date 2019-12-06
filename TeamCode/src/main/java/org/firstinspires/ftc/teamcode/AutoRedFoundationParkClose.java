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
@Autonomous(name = "R-F-Close", group = "Skystone")
//@Disabled                            // Comment this out to add to the opmode list

public class AutoRedFoundationParkClose extends LinearOpMode {
    final private double FOUNDATION_GRABBER_DOWN = 0.40;    // grabber down
    final private double FOUNDATION_GRABBER_UP = 1;      // grabber up
    final private double CAPSTONE_NOT_DROPPED = 1;
    final private double CAPSTONE_DROPPED = 0;
    final private double FINGERS_OPEN = 0.05;               // open claw
    final private double FINGERS_CLOSED = 0.5;              //close claw
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

    public void runOpMode () {

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

        frontLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        armRotateMotor.setDirection(DcMotor.Direction.REVERSE);
        foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
        clawFingersServo.setPosition(FINGERS_OPEN);
        capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);

        waitForStart();
        runtime.reset();

        double voltage = this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        double timeMultiple = (-0.1*voltage) + 2.25;
        moveForwardInches(0.5, true, 10);
        moveSideInches(0.5, false, 32);
        sleep(500);
        foundationGrabberServo.setPosition(FOUNDATION_GRABBER_DOWN);
        sleep(1000);
        moveSideInches(0.5, true, 56);
        sleep(500);
        foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
        moveForwardInches(0.5, false, 26);
        moveSideInches(0.5, false, 18);
        moveForwardInches(0.5, true, 10);
        moveForwardInches(0.5, false, 3);
        moveSideInches(0.5, true, 20);
        moveForwardInches(0.5, false, 12);
        armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticsPerDegree = (int) ((1425.2 *24)/360);
        int degrees = 132;

        armRotateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotateMotor.setTargetPosition(ticsPerDegree * degrees);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor.setPower(1);
        sleep(4000);

        moveForwardInches(0.5, false, 16);
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
    public double moveForwardInches(double wheelPower, boolean direction, double inches) {

        // direction true => forward
        // direction false => backward
        int ticsPerMotor = 1120;
        double circumference = 12.125;
        double ticsPerInch = ticsPerMotor / circumference;
        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 50;
        double poweruse;
        int starttics = frontLeftDriveMotor.getCurrentPosition();

        if (direction) {
            FLtarget = (int)(ticsPerInch * inches + frontLeftDriveMotor.getCurrentPosition());
            FRtarget =(int)(ticsPerInch *  inches + frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int)(ticsPerInch * inches + backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int)(ticsPerInch * inches + backRightDriveMotor.getCurrentPosition());

            frontLeftDriveMotor.setTargetPosition(FLtarget);
            frontRightDriveMotor.setTargetPosition(FRtarget);
            backLeftDriveMotor.setTargetPosition(BLtarget);
            backRightDriveMotor.setTargetPosition(BRtarget);

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            FLtarget = (int)(-ticsPerInch * inches + frontLeftDriveMotor.getCurrentPosition());
            FRtarget =(int)(-ticsPerInch * inches + frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int)(-ticsPerInch * inches + backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int)(-ticsPerInch * inches + backRightDriveMotor.getCurrentPosition());

            frontLeftDriveMotor.setTargetPosition(FLtarget);
            frontRightDriveMotor.setTargetPosition(FRtarget);
            backLeftDriveMotor.setTargetPosition(BLtarget);
            backRightDriveMotor.setTargetPosition(BRtarget);

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        while ((absolute(frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol ) && (absolute(frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())) {
            poweruse = wheelPower + (((wheelPower - 0.5)/(starttics-FLtarget))* ((frontLeftDriveMotor.getCurrentPosition()-starttics)));

            frontLeftDriveMotor.setPower(poweruse);
            frontRightDriveMotor.setPower(poweruse);
            backLeftDriveMotor.setPower(poweruse);
            backRightDriveMotor.setPower(poweruse);
            sleep(50);
        }
        sleep(100);

        return (0);
    }
    public double moveSideInches(double wheelPower, boolean direction, double inches) {

        // direction true => right
        // direction false => left
        double ticsPerMotor = 1120;
        double circumference = 12.125;
        double ticsPerInch = ticsPerMotor / circumference;

        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 50;
        double poweruse;
        int starttics = frontLeftDriveMotor.getCurrentPosition();

        if (direction) {
            double sideMultiple = 1.2;
            FLtarget = (int)(ticsPerInch * sideMultiple * inches + frontLeftDriveMotor.getCurrentPosition());
            FRtarget =(int)(-ticsPerInch * sideMultiple * inches + frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int)(-ticsPerInch * sideMultiple * inches + backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int)(ticsPerInch * sideMultiple * inches + backRightDriveMotor.getCurrentPosition());

            frontLeftDriveMotor.setTargetPosition(FLtarget);
            frontRightDriveMotor.setTargetPosition(FRtarget);
            backLeftDriveMotor.setTargetPosition(BLtarget);
            backRightDriveMotor.setTargetPosition(BRtarget);

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            double sideMultiple = 1.2;

            FLtarget = (int)(-ticsPerInch * sideMultiple * inches + frontLeftDriveMotor.getCurrentPosition());
            FRtarget =(int)(ticsPerInch * sideMultiple * inches + frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int)(ticsPerInch * sideMultiple * inches + backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int)(-ticsPerInch * sideMultiple * inches + backRightDriveMotor.getCurrentPosition());

            frontLeftDriveMotor.setTargetPosition(FLtarget);
            frontRightDriveMotor.setTargetPosition(FRtarget);
            backLeftDriveMotor.setTargetPosition(BLtarget);
            backRightDriveMotor.setTargetPosition(BRtarget);

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        while ((absolute(frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol ) && (absolute(frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())) {
            poweruse = wheelPower + (((wheelPower - 0.5)/(starttics-FLtarget))* ((frontLeftDriveMotor.getCurrentPosition()-starttics)));

            frontLeftDriveMotor.setPower(poweruse);
            frontRightDriveMotor.setPower(poweruse);
            backLeftDriveMotor.setPower(poweruse);
            backRightDriveMotor.setPower(poweruse);
            sleep(50);
        }
        sleep(100);

        return (0);
    }
    public double moveTurnDegrees(double wheelPower, boolean direction, double degrees) {

        // direction true => right
        // direction false => left

        double ticsPerMotor = 1120;
        double degreesPerRotation = 48;
        double ticsToMove = (degrees * ticsPerMotor) / degreesPerRotation;
        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 50;

        if (direction) {
            FLtarget = (int)(-ticsToMove) + frontLeftDriveMotor.getCurrentPosition();
            FRtarget =(int)(ticsToMove)+ frontRightDriveMotor.getCurrentPosition();
            BLtarget = (int)(-ticsToMove) + backLeftDriveMotor.getCurrentPosition();
            BRtarget = (int)(ticsToMove)+ backRightDriveMotor.getCurrentPosition();

            frontLeftDriveMotor.setTargetPosition(FLtarget);
            frontRightDriveMotor.setTargetPosition(FRtarget);
            backLeftDriveMotor.setTargetPosition(BLtarget);
            backRightDriveMotor.setTargetPosition(BRtarget);

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            FLtarget = (int)(ticsToMove) + frontLeftDriveMotor.getCurrentPosition();
            FRtarget =(int)(-ticsToMove)+ frontRightDriveMotor.getCurrentPosition();
            BLtarget = (int)(ticsToMove) + backLeftDriveMotor.getCurrentPosition();
            BRtarget = (int)(-ticsToMove)+ backRightDriveMotor.getCurrentPosition();

            frontLeftDriveMotor.setTargetPosition(FLtarget);
            frontRightDriveMotor.setTargetPosition(FRtarget);
            backLeftDriveMotor.setTargetPosition(BLtarget);
            backRightDriveMotor.setTargetPosition(BRtarget);

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        while ((absolute(frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol ) && (absolute(frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())){
            frontLeftDriveMotor.setPower(wheelPower);
            frontRightDriveMotor.setPower(wheelPower);
            backLeftDriveMotor.setPower(wheelPower);
            backRightDriveMotor.setPower(wheelPower);
            sleep(50);
        }
        sleep(100);

        return (0);
    }
}
