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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
@Autonomous(name = "(test)PID", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class TestPID extends LinearOpMode {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDriveMotor = null;
    private DcMotor frontRightDriveMotor = null;
    private DcMotor backLeftDriveMotor = null;
    private DcMotor backRightDriveMotor = null;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDriveMotor = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

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

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();
        runtime.reset();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        if (opModeIsActive()) {
            double startAngle = angles.firstAngle;
            sleep(1000);
            //0.01, 0.00045, 0.000045
            PIDstraightInches(0.01, 0.0, 0.0, 0.2, -1, 18, startAngle);
            moveTurnDegrees(0.2, false, 90);
            double newangle = startAngle+90.0;
            sleep(1000);
            PIDstraightInches(0.01, 0.0, 0.0, 0.2, -1, 2000, newangle);


            //  stop();
        }


    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public void PIDstraightTime(double gainP, double gainI, double gainD, double initSpeed, double direction, double targetTime) {
        double timeLast = 0;
        double reference1 = angles.firstAngle;
        double Ilast = 0;
        double errorlast = 0;
        double P;
        double I;
        double D;
        double dT;
        double output;
        double Kp = gainP;
        double Ki = gainI;
        double Kd = gainD;
        double timeNow;
        double poL;
        double poR;
        double error;
        double anglenow;
        double refTime = getRuntime();
        double elapsedTime = 0;
        long TIMESLEEP = 100;

        while ((elapsedTime < targetTime) && opModeIsActive()) {

            sleep(TIMESLEEP);

            // calculate time
            timeNow = getRuntime();
            elapsedTime = timeNow - refTime;
            dT = timeNow - timeLast;

            // P
            anglenow = angles.firstAngle;
            error = anglenow - reference1;
            P = error;
            // I
            I = Ilast + (error * dT);
            // D
            D = (error - errorlast) / dT;

            output = ((Kp * P) + (Ki * I) + (Kd * D));

            if (output < 0) {
                poL = -output;
                poR = output;
                if (poR < -0.8) {
                    poR = -0.8;
                }
                if (poL > 0.8) {
                    poL = 0.8;
                }
            } else {
                poL = -output;
                poR = output;
                if (poL < -0.8) {
                    poL = -0.8;
                }
                if (poR > 0.8) {
                    poR = 0.8;
                }
            }

            frontLeftDriveMotor.setPower(-initSpeed * direction + poL);
            frontRightDriveMotor.setPower(-initSpeed * direction + poR);
            backLeftDriveMotor.setPower(-initSpeed * direction + poL);
            backRightDriveMotor.setPower(-initSpeed * direction + poR);

            Ilast = I;
            errorlast = error;
            timeLast = timeNow;

            telemetry.addData("PoL: ", poL);
            telemetry.addData("PoR: ", poR);
            telemetry.addData("error: ", error);
            telemetry.addData("output: ", output);
            telemetry.addData("P: ", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            telemetry.addData("Time that's passed: ", elapsedTime);
            telemetry.update();

        }

    }

    public void PIDturn(double gainP, double gainI, double gainD, double reference1, double maxpower) {
        double timeLast = 0;
        double Ilast = 0;
        double errorlast = 0;
        double P;
        double I;
        double D;
        double dT;
        double outputPID;
        double outputPD;
        double output;
        double Kp = gainP;
        double Ki = gainI;
        double Kd = gainD;
        double timeNow;
        double poL;
        double poR;
        double error = reference1 - angles.firstAngle;
        double anglenow;
        double refTime = getRuntime();
        double elapsedTime = 0;
        long TIMESLEEP = 100;


        while ((absolute(error) > 0.5) && opModeIsActive()) {

            sleep(TIMESLEEP);

            // calculate time
            timeNow = getRuntime();
            elapsedTime = timeNow - refTime;
            dT = timeNow - timeLast;

            // P
            anglenow = angles.firstAngle;
            error = anglenow - reference1;
            P = error;
            // I
            I = Ilast + (error * dT);
            // D
            D = (error - errorlast) / dT;

            outputPID = ((Kp * P) + (Ki * I) + (Kd * D));
            outputPD = ((Kp * P) + (Kd * D));
            if (absolute(error) < 6.0){
                output = outputPID;
            }else{
                output = outputPD;
            }

            if (output < 0) {
                poL = -output;
                poR = output;
                if (poR < -maxpower) {
                    poR = -maxpower;
                }
                if (poL > maxpower) {
                    poL = maxpower;
                }
            } else {
                poL = -output;
                poR = output;
                if (poL < -maxpower) {
                    poL = -maxpower;
                }
                if (poR > maxpower) {
                    poR = maxpower;
                }
            }

            frontLeftDriveMotor.setPower(poL);
            frontRightDriveMotor.setPower(poR);
            backLeftDriveMotor.setPower(poL);
            backRightDriveMotor.setPower(poR);

            Ilast = I;
            errorlast = error;
            timeLast = timeNow;

            telemetry.addData("PoL: ", poL);
            telemetry.addData("PoR: ", poR);
            telemetry.addData("error: ", error);
            telemetry.addData("output: ", output);
            telemetry.addData("P: ", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            telemetry.addData("Time that's passed: ", elapsedTime);
            telemetry.update();

        }

    }
    public void PIDsideTime(double gainP, double gainI, double gainD, double initSpeed, double direction, double targetTime) {
        double timeLast = 0;
        double reference1 = angles.firstAngle;
        double Ilast = 0;
        double errorlast = 0;
        double P;
        double I;
        double D;
        double dT;
        double output;
        double Kp = gainP;
        double Ki = gainI;
        double Kd = gainD;
        double timeNow;
        double poL;
        double poR;
        double error;
        double anglenow;
        double refTime = getRuntime();
        double elapsedTime = 0;
        long TIMESLEEP = 100;
        while ((elapsedTime < targetTime) && opModeIsActive()) {

            sleep(TIMESLEEP);

            // calculate time
            timeNow = getRuntime();
            elapsedTime = timeNow - refTime;
            dT = timeNow - timeLast;

            // P
            anglenow = angles.firstAngle;
            error = anglenow - reference1;
            P = error;
            // I
            I = Ilast + (error * dT);
            // D
            D = (error - errorlast) / dT;

            output = ((Kp * P) + (Ki * I) + (Kd * D));

            if (output < 0) {
                poL = -output;
                poR = output;
                if (poR < -0.8) {
                    poR = -0.8;
                }
                if (poL > 0.8) {
                    poL = 0.8;
                }
            } else {
                poL = -output;
                poR = output;
                if (poL < -0.8) {
                    poL = -0.8;
                }
                if (poR > 0.8) {
                    poR = 0.8;
                }
            }

            frontLeftDriveMotor.setPower(-initSpeed * direction + poL);
            frontRightDriveMotor.setPower(initSpeed * direction + poR);
            backLeftDriveMotor.setPower(initSpeed * direction + poL);
            backRightDriveMotor.setPower(-initSpeed * direction + poR);

            Ilast = I;
            errorlast = error;
            timeLast = timeNow;

            telemetry.addData("PoL: ", poL);
            telemetry.addData("PoR: ", poR);
            telemetry.addData("error: ", error);
            telemetry.addData("output: ", output);
            telemetry.addData("P: ", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            telemetry.addData("Time that's passed: ", elapsedTime);
            telemetry.update();

        }
    }
    public void PIDstraightInches(double gainP, double gainI, double gainD, double initSpeed, int direction, double targetInches, double reference1) {
        double timeLast = 0;
        double Ilast = 0;
        double errorlast = 0;
        double P;
        double I;
        double D;
        double dT;
        double output;
        double Kp = gainP;
        double Ki = gainI;
        double Kd = gainD;
        double timeNow;
        double poL;
        double poR;
        double error;
        double anglenow;
        anglenow = angles.firstAngle;
        double refTime = getRuntime();
        double elapsedTime = 0;
        long TIMESLEEP = 100;
        int ticsPerMotor = (1120);
        double circumference = 12.125;
        double ticsPerInch = (ticsPerMotor / circumference) / 2;
        double startPos = frontRightDriveMotor.getCurrentPosition();

        int target = ((int)(targetInches * ticsPerInch));

        frontRightDriveMotor.setTargetPosition(direction *(target)+ frontRightDriveMotor.getCurrentPosition());
        frontLeftDriveMotor.setTargetPosition(direction *(target)+ frontLeftDriveMotor.getCurrentPosition());
        backRightDriveMotor.setTargetPosition(direction *(target)+ backRightDriveMotor.getCurrentPosition());
        backLeftDriveMotor.setTargetPosition(direction *(target)+ backLeftDriveMotor.getCurrentPosition());

        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((absolute((frontRightDriveMotor.getCurrentPosition() - startPos)) < ((target)) - 50) && opModeIsActive()) {

            sleep(TIMESLEEP);

            // calculate time
            timeNow = getRuntime();
            elapsedTime = timeNow - refTime;
            dT = timeNow - timeLast;

            // P
            anglenow = angles.firstAngle;
            error = anglenow - reference1;
            P = error;
            // I
            I = Ilast + (error * dT);
            // D
            D = (error - errorlast) / dT;

            output = ((Kp * P) + (Ki * I) + (Kd * D));

            poL = -output * direction;
            poR = output * direction;
            if (poR < -0.8) {
                poR = -0.8;
            }
            if (poL > 0.8) {
                poL = 0.8;
            }
            if (poL < -0.8) {
                poL = -0.8;
            }
            if (poR > 0.8) {
                poR = 0.8;
            }

            frontLeftDriveMotor.setPower(initSpeed + poL);
            frontRightDriveMotor.setPower(initSpeed + poR);
            backLeftDriveMotor.setPower(initSpeed + poL);
            backRightDriveMotor.setPower(initSpeed + poR);

            Ilast = I;
            errorlast = error;
            timeLast = timeNow;

            telemetry.addData("PoL: ", poL);
            telemetry.addData("PoR: ", poR);
            telemetry.addData("angleNOW",anglenow);
            telemetry.addData("reference1", reference1);
            telemetry.addData("error: ", error);
            telemetry.addData("output: ", output);
            telemetry.addData("P: ", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            telemetry.addData("Time that's passed: ", elapsedTime);
            telemetry.addData("dT", dT);
            telemetry.update();

        }

    }
    public void PIDsideInches(double gainP, double gainI, double gainD, double initSpeed, int direction, double targetInches, double reference1) {
        double timeLast = 0;
        double Ilast = 0;
        double errorlast = 0;
        double P;
        double I;
        double D;
        double dT;
        double output;
        double Kp = gainP;
        double Ki = gainI;
        double Kd = gainD;
        double timeNow;
        double poB;
        double poF;
        double error;
        double anglenow;
        double refTime = getRuntime();
        double elapsedTime = 0;
        long TIMESLEEP = 100;
        int ticsPerMotor = (1120);
        double circumference = 12.125;
        double ticsMultiple = 1.2;
        double ticsPerInch = ((ticsPerMotor / circumference) * ticsMultiple) / 2;
        double startPos = frontRightDriveMotor.getCurrentPosition();

        int target = ((int)(targetInches * ticsPerInch));

        frontRightDriveMotor.setTargetPosition(-direction *(target)+ frontRightDriveMotor.getCurrentPosition());
        frontLeftDriveMotor.setTargetPosition(direction *(target)+ frontLeftDriveMotor.getCurrentPosition());
        backRightDriveMotor.setTargetPosition(direction *(target)+ backRightDriveMotor.getCurrentPosition());
        backLeftDriveMotor.setTargetPosition(-direction *(target)+ backLeftDriveMotor.getCurrentPosition());

        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((absolute((frontRightDriveMotor.getCurrentPosition() - startPos)) < ((target)) - 50) && opModeIsActive()) {

            sleep(TIMESLEEP);

            // calculate time
            timeNow = getRuntime();
            elapsedTime = timeNow - refTime;
            dT = timeNow - timeLast;

            // P
            anglenow = angles.firstAngle;
            error = anglenow - reference1;
            P = error;
            // I
            I = Ilast + (error * dT);
            // D
            D = (error - errorlast) / dT;

            output = ((Kp * P) + (Ki * I) + (Kd * D));

            poB = output * direction;
            poF = -output * direction;
            if (poF < -0.8) {
                poF = -0.8;
            }
            if (poB > 0.8) {
                poB = 0.8;
            }
            if (poB < -0.8) {
                poB = -0.8;
            }
            if (poF > 0.8) {
                poF = 0.8;
            }

            frontLeftDriveMotor.setPower(initSpeed + poF);
            frontRightDriveMotor.setPower(initSpeed + poF);
            backLeftDriveMotor.setPower(initSpeed + poB);
            backRightDriveMotor.setPower(initSpeed + poB);

            Ilast = I;
            errorlast = error;
            timeLast = timeNow;

            telemetry.addData("PoB: ", poB);
            telemetry.addData("PoF: ", poF);
            telemetry.addData("error: ", error);
            telemetry.addData("output: ", output);
            telemetry.addData("P: ", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            telemetry.addData("Time that's passed: ", elapsedTime);
            telemetry.update();

        }

    }

    public double moveTurnDegrees(double wheelPower, boolean direction, double degrees) {

        // direction true => right
        // direction false => left

        double ticsPerMotor = 1120;
        double degreesPerRotation = 48;
        double ticsToMove = ((degrees * ticsPerMotor) / degreesPerRotation) / 2;
        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 25;

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
            sleep(25);
        }
        sleep(100);

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