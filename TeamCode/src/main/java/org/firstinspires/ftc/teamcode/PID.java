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
 * {@link PID} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "PID", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class PID extends LinearOpMode {
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
        while (opModeIsActive()) {
           //PIDstraight(0.02, 0.0009, 0.00009, 0.2, 1, 10);
           //PIDstraight(0.02, 0.0009, 0.00009, 0.2, -1, 10);
           PIDturn(0.02, 0.0008, 0.007, -90,0.4);
           //PIDside(0.02, 0.0009, 0.00009, 0.2, 1, 10);
           //PIDside(0.02, 0.0009, 0.00009, 0.2, -1, 10);
           stop();
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


    public void PIDstraight(double gainP, double gainI, double gainD, double initSpeed, double direction, double targetTime) {
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

    public void PIDturn(double gainP, double gainI, double gainD, double turnAngle, double maxpower) {
        turnAngle = -1 * turnAngle;
        double timeLast = 0;
        double reference1;
        reference1 = angles.firstAngle + turnAngle;
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
        double error = turnAngle;
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
    public void PIDside(double gainP, double gainI, double gainD, double initSpeed, double direction, double targetTime) {
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

    public double absolute(double inputval) {
       if (inputval > 0) {
           return inputval;
       } else {
            return -1 * inputval;
       }
    }
}