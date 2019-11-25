package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * {@link TestPID} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "B-F-Close", group = "Skystone")
//@Disabled                            // Comment this out to add to the opmode list

public class CommonMethods extends LinearOpMode {
    private DcMotor frontLeftDriveMotor = null;
    private DcMotor frontRightDriveMotor = null;
    private DcMotor backLeftDriveMotor = null;
    private DcMotor backRightDriveMotor = null;

    private ElapsedTime runtime = new ElapsedTime();

    public CommonMethods(DcMotor frontLeftDriveMotor, DcMotor frontRightDriveMotor, DcMotor backLeftDriveMotor, DcMotor backRightDriveMotor) {
        this.frontRightDriveMotor = frontRightDriveMotor;
        this.frontLeftDriveMotor = frontLeftDriveMotor;
        this.backRightDriveMotor = backRightDriveMotor;
        this.backLeftDriveMotor = backLeftDriveMotor;
    }

    public void runOpMode (){
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
