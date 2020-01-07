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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * {@link CommonMethods} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
abstract public class CommonMethods extends LinearOpMode {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    BNO055IMU imu;

    final public double FINGERS_OPEN = 0.05;               // open claw
    final public double FINGERS_CLOSED = 0.5;              //close claw
    final public double GAIN_P = 0.02;
    final public double GAIN_I = 0.00045;
    final public double GAIN_D = 0.00009;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    // The IMU sensor object

    public ElapsedTime runtime = new ElapsedTime();

    final public double SLOW_DRIVE_MULTIPLE_FORWARD = 0.3;
    final public double SLOW_DRIVE_MULTIPLE_SIDE = 0.5;
    final public double MID_DRIVE_MULTIPLE_FORWARD = 0.5;
    final public double MID_DRIVE_MULTIPLE_SIDE = 0.65;
    final public double FAST_DRIVE_MULTIPLE_FORWARD = 1;
    final public double FAST_DRIVE_MULTIPLE_SIDE = 1;
    final public double WRIST_TURN_MULTIPLE = 0.005;       // multiple for wrist turn fine tuning
    final public double ARM_EXTEND_MULTIPLE = 0.5;         // multiple for arm extend

    final public double WRIST_TURN_HORIZONTAL = 0.55;       // wrist turn for horizontal block
    final public double WRIST_TURN_VERTICAL = 0.05;         // wrist turn for vertical block


    final public double FOUNDATION_GRABBER_DOWN = 0.40;    // grabber down
    final public double FOUNDATION_GRABBER_UP = 1;         // grabber up


    final public double DELIVERY_LEFT_OPEN_FULLY = 0.31;
    final public double DELIVERY_RIGHT_OPEN_FULLY = 0.65;
    final public double DELIVERY_SERVO_IN_LEFT = 0.9;
    final public double DELIVERY_SERVO_IN_RIGHT = 0.05;
    final public double DELIVERY_SERVO_LEFT_IN_A_LITTLE = 0.55;
    final public double DELIVERY_SERVO_RIGHT_IN_A_LITTLE = 0.45;

    final public double STONE_PICKER_CLOSED_RED = 1;
    final public double STONE_PICKER_CLOSED_BLUE = 0.1;
    final public double STONE_PICKER_OPEN_RED = 0;
    final public double STONE_PICKER_OPEN_BLUE = 1;

    final public double CAPSTONE_NOT_DROPPED = 1;
    final public double CAPSTONE_DROPPED = 0;

    final public double BASE_ARM_LENGTH = 13;
    final public double SPINDLE_CIRCUMFERENCE = (2.5 * (Math.PI));
    final public double TICS_PER_ROTATION_ROTATION_MOTOR = 1425.2;
    final public double TICS_PER_ROTATION_EXTENTION_MOTOR = 537.6;
    final public double TICS_PER_DEGREE = (TICS_PER_ROTATION_ROTATION_MOTOR * 24) / 360;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------
    public void StonePickDeliver(int RedBlue, int stoneCase, double reference1, Hardware hardware){
        // Blue = 1 and Red = -1
        // Run this after the robot is aligned to pick up the stone before checking the distance
        // This method will pick up and deliver two stones depending on the stone cases
        double deltaD;
        double blockDistance;
        if (stoneCase == 2){
            deltaD = 8;
        }else if (stoneCase == 3){
            deltaD = 16;
        }else{
            deltaD = 0;
        }
        if (RedBlue > 0) {
            blockDistance = hardware.sensorBlue.getDistance(DistanceUnit.INCH);
        }else{
            blockDistance = hardware.sensorRed.getDistance(DistanceUnit.INCH);
        }

        PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.2, -1, blockDistance, reference1, hardware);

        sleep(500);
        if (RedBlue > 0) {
            hardware.stoneServoBlue.setPosition(STONE_PICKER_OPEN_BLUE);
        }else{
            hardware.stoneServoRed.setPosition(STONE_PICKER_OPEN_RED);
        }
        sleep(500);

        PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 6, reference1, hardware);
        moveTurnDegrees(0.3, -1 * RedBlue, 90, hardware);
        PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.5, -1, 42 + deltaD, reference1 + (90 * RedBlue), hardware);

        sleep(500);
        if (RedBlue > 0) {
            hardware.stoneServoBlue.setPosition(STONE_PICKER_CLOSED_BLUE);
        }else{
            hardware.stoneServoRed.setPosition(STONE_PICKER_CLOSED_RED);
        }
        sleep(500);

        PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.5, 1, 70, reference1 + (90*RedBlue), hardware);
        moveTurnDegrees(0.3, 1 * RedBlue, 90, hardware);
        PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.3, -1 * RedBlue, 17, reference1, hardware);
        PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.3, 1 * RedBlue, 16.5 - deltaD, reference1, hardware);
        if (RedBlue > 0) {
            blockDistance = hardware.sensorBlue.getDistance(DistanceUnit.INCH);
        }else{
            blockDistance = hardware.sensorRed.getDistance(DistanceUnit.INCH);
        }
        PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.2, -1, blockDistance, reference1, hardware);

        sleep(500);
        if (RedBlue > 0) {
            hardware.stoneServoBlue.setPosition(STONE_PICKER_OPEN_BLUE);
        }else{
            hardware.stoneServoRed.setPosition(STONE_PICKER_OPEN_RED);
        }
        sleep(500);

        PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 6, reference1, hardware);
        PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.3, 1 * RedBlue, 6, reference1, hardware);
        moveTurnDegrees(0.3, -1 * RedBlue, 90, hardware);
        PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.5, -1, 56 + deltaD, reference1 + (90*RedBlue), hardware);

        sleep(500);
        if (RedBlue > 0) {
            hardware.stoneServoBlue.setPosition(STONE_PICKER_CLOSED_BLUE);
        }else{
            hardware.stoneServoRed.setPosition(STONE_PICKER_CLOSED_RED);
        }
        sleep(500);

        PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.5, 1, 12, reference1 + (90*RedBlue), hardware);
    }

    public void PIDturn(double gainP, double gainI, double gainD, double reference1, double maxpower, Hardware hardware) {
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
        double anglenow = getAngle(hardware);
        double error = reference1 - anglenow;
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
            anglenow = getAngle(hardware);
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

            hardware.frontLeftDriveMotor.setPower(poL);
            hardware.frontRightDriveMotor.setPower(poR);
            hardware.backLeftDriveMotor.setPower(poL);
            hardware.backRightDriveMotor.setPower(poR);

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
        sleep(200);

    }

    public void PIDstraightInches(double gainP, double gainI, double gainD, double maxPower, int direction, double targetInches, double reference1, Hardware hardware) {
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
        double initSpeedUse;
        double distanceTraveled;
        double minPower = 0.05;
        anglenow = getAngle(hardware);
        double refTime = getRuntime();
        double elapsedTime = 0;
        long TIMESLEEP = 100;
        int ticsPerMotor = (1120);
        double circumference = 12.125;
        double ticsPerInch = (ticsPerMotor / circumference) / 2;
        double startPos = hardware.frontRightDriveMotor.getCurrentPosition();

        int target = ((int)(targetInches * ticsPerInch));

        hardware.frontRightDriveMotor.setTargetPosition(direction *(target)+ hardware.frontRightDriveMotor.getCurrentPosition());
        hardware.frontLeftDriveMotor.setTargetPosition(direction *(target)+ hardware.frontLeftDriveMotor.getCurrentPosition());
        hardware.backRightDriveMotor.setTargetPosition(direction *(target)+ hardware.backRightDriveMotor.getCurrentPosition());
        hardware.backLeftDriveMotor.setTargetPosition(direction *(target)+ hardware.backLeftDriveMotor.getCurrentPosition());

        hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((absolute((hardware.frontRightDriveMotor.getCurrentPosition() - startPos)) < (target) - 50) && opModeIsActive()) {

            sleep(TIMESLEEP);

            // calculate time
            timeNow = getRuntime();
            elapsedTime = timeNow - refTime;
            dT = timeNow - timeLast;

            // P
            anglenow = getAngle(hardware);
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
            double rampdistance = 6;

            distanceTraveled = absolute((hardware.frontRightDriveMotor.getCurrentPosition() - startPos) / ticsPerInch);
            if (targetInches > 2 * rampdistance){
                if(distanceTraveled < rampdistance){
                    initSpeedUse = (((maxPower - minPower) / rampdistance) * (distanceTraveled) + minPower);
                }else if((distanceTraveled > rampdistance) && (distanceTraveled < (targetInches - rampdistance))){
                    initSpeedUse = maxPower;
                }else{
                    initSpeedUse = ((-(maxPower - minPower) / rampdistance) * (distanceTraveled - targetInches) + minPower);
                }
            }else{
                initSpeedUse = maxPower;
            }
            hardware.frontLeftDriveMotor.setPower(initSpeedUse + poL);
            hardware.frontRightDriveMotor.setPower(initSpeedUse + poR);
            hardware.backLeftDriveMotor.setPower(initSpeedUse + poL);
            hardware.backRightDriveMotor.setPower(initSpeedUse + poR);

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
        sleep(200);

    }
    public void PIDsideInches(double gainP, double gainI, double gainD, double initSpeed, int direction, double targetInches, double reference1, Hardware hardware) {
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
        double startPos = hardware.frontRightDriveMotor.getCurrentPosition();

        int target = ((int)(targetInches * ticsPerInch));

        hardware.frontRightDriveMotor.setTargetPosition(-direction *(target)+ hardware.frontRightDriveMotor.getCurrentPosition());
        hardware.frontLeftDriveMotor.setTargetPosition(direction *(target)+ hardware.frontLeftDriveMotor.getCurrentPosition());
        hardware.backRightDriveMotor.setTargetPosition(direction *(target)+ hardware.backRightDriveMotor.getCurrentPosition());
        hardware.backLeftDriveMotor.setTargetPosition(-direction *(target)+ hardware.backLeftDriveMotor.getCurrentPosition());

        hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((absolute((hardware.frontRightDriveMotor.getCurrentPosition() - startPos)) < ((target)) - 50) && opModeIsActive()) {

            sleep(TIMESLEEP);

            // calculate time
            timeNow = getRuntime();
            elapsedTime = timeNow - refTime;
            dT = timeNow - timeLast;

            // P
            anglenow = getAngle(hardware);
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

            hardware.frontLeftDriveMotor.setPower(initSpeed + poF);
            hardware.frontRightDriveMotor.setPower(initSpeed + poF);
            hardware.backLeftDriveMotor.setPower(initSpeed + poB);
            hardware.backRightDriveMotor.setPower(initSpeed + poB);

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
        sleep(200);

    }

    public double moveTurnDegrees(double wheelPower, int direction, double degrees, Hardware hardware) {

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

        if (direction > 0) {
            FLtarget = (int)(-ticsToMove) + hardware.frontLeftDriveMotor.getCurrentPosition();
            FRtarget =(int)(ticsToMove)+ hardware.frontRightDriveMotor.getCurrentPosition();
            BLtarget = (int)(-ticsToMove) + hardware.backLeftDriveMotor.getCurrentPosition();
            BRtarget = (int)(ticsToMove)+ hardware.backRightDriveMotor.getCurrentPosition();

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            FLtarget = (int)(ticsToMove) + hardware.frontLeftDriveMotor.getCurrentPosition();
            FRtarget =(int)(-ticsToMove)+ hardware.frontRightDriveMotor.getCurrentPosition();
            BLtarget = (int)(ticsToMove) + hardware.backLeftDriveMotor.getCurrentPosition();
            BRtarget = (int)(-ticsToMove)+ hardware.backRightDriveMotor.getCurrentPosition();

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        while ((absolute(hardware.frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol ) && (absolute(hardware.frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(hardware.backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(hardware.backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())){
            hardware.frontLeftDriveMotor.setPower(wheelPower);
            hardware.frontRightDriveMotor.setPower(wheelPower);
            hardware.backLeftDriveMotor.setPower(wheelPower);
            hardware.backRightDriveMotor.setPower(wheelPower);
            sleep(25);
        }
        sleep(300);

        return (0);
    }
    public double moveForwardInches(double wheelPower, boolean direction, double inches, Hardware hardware) {
        // direction true => forward
        // direction false => backward
        int ticsPerMotor = 1120;
        double circumference = 12.125;
        double ticsPerInch = (ticsPerMotor / circumference) / 2;
        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 25;
        double poweruse;
        int starttics = hardware.frontLeftDriveMotor.getCurrentPosition();

        if (direction) {
            FLtarget = (int) (ticsPerInch * inches + hardware.frontLeftDriveMotor.getCurrentPosition());
            FRtarget = (int) (ticsPerInch * inches + hardware.frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int) (ticsPerInch * inches + hardware.backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int) (ticsPerInch * inches + hardware.backRightDriveMotor.getCurrentPosition());

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            FLtarget = (int) (-ticsPerInch * inches + hardware.frontLeftDriveMotor.getCurrentPosition());
            FRtarget = (int) (-ticsPerInch * inches + hardware.frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int) (-ticsPerInch * inches + hardware.backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int) (-ticsPerInch * inches + hardware.backRightDriveMotor.getCurrentPosition());

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        while ((absolute(hardware.frontLeftDriveMotor.getCurrentPosition() - FLtarget) > ticksTol) && (absolute(hardware.frontRightDriveMotor.getCurrentPosition() - FRtarget) > ticksTol) && (absolute(hardware.backLeftDriveMotor.getCurrentPosition() - BLtarget) > ticksTol) && (absolute(hardware.backRightDriveMotor.getCurrentPosition() - BRtarget) > ticksTol) && (opModeIsActive())) {
            poweruse = wheelPower + (((wheelPower - 0.5) / (starttics - FLtarget)) * ((hardware.frontLeftDriveMotor.getCurrentPosition() - starttics)));

            hardware.frontLeftDriveMotor.setPower(poweruse);
            hardware.frontRightDriveMotor.setPower(poweruse);
            hardware.backLeftDriveMotor.setPower(poweruse);
            hardware.backRightDriveMotor.setPower(poweruse);
            sleep(25);
        }
        sleep(200);
        return (0);
    }
    public double moveSideInches(double wheelPower, boolean direction, double inches, Hardware hardware) {

        // direction true => right
        // direction false => left
        double ticsPerMotor = 1120;
        double circumference = 12.125;
        double ticsPerInch = (ticsPerMotor / circumference) / 2;

        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 25;
        double poweruse;
        int starttics = hardware.frontLeftDriveMotor.getCurrentPosition();

        if (direction) {
            double sideMultiple = 1.2;
            FLtarget = (int) (ticsPerInch * sideMultiple * inches + hardware.frontLeftDriveMotor.getCurrentPosition());
            FRtarget = (int) (-ticsPerInch * sideMultiple * inches + hardware.frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int) (-ticsPerInch * sideMultiple * inches + hardware.backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int) (ticsPerInch * sideMultiple * inches + hardware.backRightDriveMotor.getCurrentPosition());

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            double sideMultiple = 1.2;

            FLtarget = (int) (-ticsPerInch * sideMultiple * inches + hardware.frontLeftDriveMotor.getCurrentPosition());
            FRtarget = (int) (ticsPerInch * sideMultiple * inches + hardware.frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int) (ticsPerInch * sideMultiple * inches + hardware.backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int) (-ticsPerInch * sideMultiple * inches + hardware.backRightDriveMotor.getCurrentPosition());

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        while ((absolute(hardware.frontLeftDriveMotor.getCurrentPosition() - FLtarget) > ticksTol) && (absolute(hardware.frontRightDriveMotor.getCurrentPosition() - FRtarget) > ticksTol) && (absolute(hardware.backLeftDriveMotor.getCurrentPosition() - BLtarget) > ticksTol) && (absolute(hardware.backRightDriveMotor.getCurrentPosition() - BRtarget) > ticksTol) && (opModeIsActive())) {
            poweruse = wheelPower + (((wheelPower - 0.5) / (starttics - FLtarget)) * ((hardware.frontLeftDriveMotor.getCurrentPosition() - starttics)));

            hardware.frontLeftDriveMotor.setPower(poweruse);
            hardware.frontRightDriveMotor.setPower(poweruse);
            hardware.backLeftDriveMotor.setPower(poweruse);
            hardware.backRightDriveMotor.setPower(poweruse);
            sleep(25);
        }
        sleep(200);

        return (0);

    }

    public double getAngle(Hardware hardware)
    {
        Orientation angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double absolute(double inputval) {
        if (inputval > 0) {
            return inputval;
        } else {
            return -1 * inputval;
        }
    }
}