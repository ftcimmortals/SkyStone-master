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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="POV_Mode", group="Linear Opmode")
// @Disabled
public class POVmode extends CommonMethods {

    // define IMU
    private BNO055IMU imu;

    // Constants
    private ElapsedTime runtime = new ElapsedTime();

    // Motors

    boolean currentState = false;
    boolean lastState = false;
    boolean currentRun = false;
    boolean lastRun = false;

    int currentPosition = 0;
    int targetPosition = 0;
    double numberToClear;
    double driveMultipleSide0 = SLOW_DRIVE_MULTIPLE_SIDE;
    double driveMultipleForward0 = SLOW_DRIVE_MULTIPLE_FORWARD;
    double driveMultipleSide = SLOW_DRIVE_MULTIPLE_SIDE;
    double driveMultipleForward = SLOW_DRIVE_MULTIPLE_FORWARD;
    boolean currentDrive = false;
    boolean lastDrive = false;

    float theta = 0.0f;


    @Override
    public void runOpMode() {

        Hardware hardware = new Hardware(hardwareMap);
        /* Initialize the hardware variables. Note that the strings used here as parameters
            to 'get' must correspond to the names assigned during the robot configuration
            step (using the FTC Robot Controller app on the phone). */

        // game controller #1

        hardware.armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        hardware.armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        hardware.frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.armRotateMotor.setDirection(DcMotor.Direction.REVERSE);

        hardware.clawFingersServo.setPosition(FINGERS_OPEN);
        hardware.clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);
        hardware.foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);      //set grabber position at init
        hardware.capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);

        hardware.armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        hardware.armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);

        int armExtendPosition;
        hardware.armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder at init
        hardware.armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);
        hardware.stoneServoBlue.setPosition(STONE_PICKER_CLOSED_BLUE);
        hardware.stoneServoRed.setPosition(STONE_PICKER_CLOSED_RED);
        hardware.foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
        hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
        hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);
        int extendMotorStartPos = hardware.armExtendMotor.getCurrentPosition();
        int rotateMotorStartPos = hardware.armRotateMotor.getCurrentPosition();

        hardware.frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int tester = 0;

        boolean runProgram = false;
        String speedState = "SLOW";

        double lastPosLeft = DELIVERY_SERVO_IN_LEFT;
        double lastPosRight = DELIVERY_SERVO_IN_RIGHT;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Setup a variable for each drive wheel to save power level for telemetry

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double x = hardware.armRotateMotor.getCurrentPosition() / TICS_PER_DEGREE;
            double MultFactor = 1 /*((-((MAX_FACTOR/135)) * x) + MAX_FACTOR)*/;

            // DRIVER 1 : Gamepad 1 controls

            double turn = gamepad1.right_stick_x;
            double mY = gamepad1.left_stick_y;
            double mX = gamepad1.left_stick_x;

            currentDrive = gamepad1.y;

            if((!currentDrive) && (lastDrive)) {
                if(speedState == "SLOW") {
                    driveMultipleSide0 = MID_DRIVE_MULTIPLE_SIDE;
                    driveMultipleForward0 = MID_DRIVE_MULTIPLE_FORWARD;
                    speedState = "MEDIUM";
                }
                else if (speedState == "MEDIUM") {
                    driveMultipleSide0 = FAST_DRIVE_MULTIPLE_SIDE;
                    driveMultipleForward0 = FAST_DRIVE_MULTIPLE_FORWARD;
                    speedState = "FAST";
                }
                else if (speedState == "FAST"){
                    driveMultipleSide0 = SLOW_DRIVE_MULTIPLE_SIDE;
                    driveMultipleForward0 = SLOW_DRIVE_MULTIPLE_FORWARD;
                    speedState = "SLOW";
                }
            }
            if (speedState == "FAST") {
                if (gamepad1.right_trigger > 0) {
                    driveMultipleSide = driveMultipleSide0 * 0.3;
                    driveMultipleForward = driveMultipleForward0 * 0.3;
                }else{
                    driveMultipleSide = driveMultipleSide0;
                    driveMultipleForward = driveMultipleForward0;
                }
            } else {
                if (gamepad1.right_trigger > 0) {
                    driveMultipleSide = driveMultipleSide0 * 0.5;
                    driveMultipleForward = driveMultipleForward0 * 0.5;
                }else{
                    driveMultipleSide = driveMultipleSide0;
                    driveMultipleForward = driveMultipleForward0;
                }
            }

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double powerflD = (mY * -driveMultipleForward);
            double powerfrD = (mY * -driveMultipleForward);
            double powerblD = (mY * -driveMultipleForward);
            double powerbrD = (mY * -driveMultipleForward);

            double powerflS = (mX * driveMultipleSide);
            double powerfrS = (mX * -driveMultipleSide);
            double powerblS = ((mX * -driveMultipleSide) * MultFactor);
            double powerbrS = ((mX * driveMultipleSide) * MultFactor);

            double powerfl = powerflS + powerflD;
            double powerfr = powerfrS + powerfrD;
            double powerbl = powerblS + powerblD;
            double powerbr = powerbrS + powerbrD;

            hardware.frontLeftDriveMotor.setPower(powerfl);
            hardware.frontRightDriveMotor.setPower(powerfr);
            hardware.backLeftDriveMotor.setPower(powerbl);
            hardware.backRightDriveMotor.setPower(powerbr);

            if (turn != 0) {
                hardware.frontLeftDriveMotor.setPower(-turn * driveMultipleSide);
                hardware.backRightDriveMotor.setPower(turn * driveMultipleSide);
                hardware.backLeftDriveMotor.setPower(-turn * driveMultipleSide);
                hardware.frontRightDriveMotor.setPower(turn * driveMultipleSide);
            }

            if (gamepad1.dpad_up) {
                hardware.foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
            }
            if (gamepad1.dpad_down) {
                hardware.foundationGrabberServo.setPosition(FOUNDATION_GRABBER_DOWN);
            }


            if (gamepad1.x) {
                lastPosLeft = DELIVERY_LEFT_OPEN_FULLY;
                lastPosRight = DELIVERY_RIGHT_OPEN_FULLY;
                hardware.deliveryServoLeft.setPosition(lastPosLeft);
                hardware.deliveryServoRight.setPosition(lastPosRight);
            }
            if (gamepad1.b) {
                lastPosLeft = DELIVERY_SERVO_IN_LEFT;
                lastPosRight = DELIVERY_SERVO_IN_RIGHT;
                hardware.deliveryServoLeft.setPosition(lastPosLeft);
                hardware.deliveryServoRight.setPosition(lastPosRight);
            }
            if ((hardware.deliveryServoLeft.getPosition() == DELIVERY_LEFT_OPEN_FULLY) && (gamepad1.left_trigger > 0)){
                lastPosLeft = DELIVERY_SERVO_LEFT_IN_A_LITTLE;
                lastPosRight = DELIVERY_SERVO_RIGHT_IN_A_LITTLE;
                hardware.deliveryServoLeft.setPosition(lastPosLeft);
                hardware.deliveryServoRight.setPosition(lastPosRight);
            }else if ((gamepad1.left_trigger == 0) && (!gamepad1.b) && (lastPosLeft != DELIVERY_SERVO_IN_LEFT)){
                hardware.deliveryServoLeft.setPosition(DELIVERY_LEFT_OPEN_FULLY);
                hardware.deliveryServoRight.setPosition(DELIVERY_RIGHT_OPEN_FULLY);
            }

            /*
            if(gamepad1.left_bumper){
                hardware.deliveryServoLeft.setPosition(DELIVERY_LEFT_OPEN_FULLY);
                hardware.deliveryServoRight.setPosition(DELIVERY_RIGHT_OPEN_FULLY);
            }
            else if ((!gamepad1.left_bumper) && (lastPosLeft == DELIVERY_SERVO_IN_LEFT) && (!gamepad1.x)){
                hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
                hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);
            }*/

            currentRun = gamepad1.right_bumper;
            if((currentRun == false) && (lastRun == true)){
                hardware.deliveryServoLeft.setPosition(DELIVERY_LEFT_OPEN_FULLY);
                hardware.deliveryServoRight.setPosition(DELIVERY_RIGHT_OPEN_FULLY);
                sleep(500);
                hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
                hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);
                sleep(500);
                moveSideInches(0.2, true, 4, hardware);
            }

            //DRIVER 2 : Gamepad 2 controls

            currentState = gamepad2.a;
            if ((!currentState) && (lastState)) {
                targetPosition++;
            }
            if (gamepad2.x) {
                targetPosition = 0;
            }
            if (gamepad2.y) {
                runProgram = true;
            }
            double height;
            double heightsq;
            double basesq;
            double hypotenuse;
            double angleInRadians;
            double angleInDegrees;
            double ticsToMove;
            double ticsPerBlock;

            if (targetPosition == 0) {
                numberToClear = 5.5;
                height = (targetPosition * 4) + numberToClear;
                heightsq = Math.pow(height, 2);
                basesq = Math.pow(BASE_ARM_LENGTH, 2);
                hypotenuse = Math.sqrt(heightsq + basesq);
                angleInRadians = Math.asin(height / hypotenuse);
                angleInDegrees = angleInRadians * (180 / Math.PI);
                ticsToMove = angleInDegrees * TICS_PER_DEGREE;
                currentPosition = 0;
            } else {
                numberToClear = 1.5;
                height = (targetPosition * 4) + numberToClear;
                heightsq = Math.pow(height, 2);
                basesq = Math.pow(BASE_ARM_LENGTH, 2);
                hypotenuse = Math.sqrt(heightsq + basesq);
                angleInRadians = Math.asin(height / hypotenuse);
                angleInDegrees = angleInRadians * (180 / Math.PI);
                ticsToMove = angleInDegrees * TICS_PER_DEGREE;
                ticsPerBlock = (ticsToMove / 4) - 2;
                currentPosition = (int) (hardware.armRotateMotor.getCurrentPosition() / ticsPerBlock);
            }

            if (runProgram) {
                if (hardware.armLimitTouchFront.getState() == false) {
                    hardware.armRotateMotor.setTargetPosition(hardware.armRotateMotor.getCurrentPosition());
                    hardware.armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hardware.armRotateMotor.setPower(1.0);
                }

                double angleStart = (150 * TICS_PER_DEGREE);

                //calculating current position
                if ((hardware.armRotateMotor.getCurrentPosition()) <= ((ticsToMove - angleStart) - 50)) {
                    // move arm up/down because arm is already contracted
                    if (hardware.armExtendMotor.getCurrentPosition() <= (extendMotorStartPos + 50)) {
                        // arm is contracted, move arm
                        hardware.armRotateMotor.setTargetPosition((int)(ticsToMove- angleStart));
                        hardware.armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardware.armRotateMotor.setPower(1.0);
                        tester = 1;
                    } else {
                        // contract arm before moving arm
                        hardware.armExtendMotor.setTargetPosition(extendMotorStartPos);
                        hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardware.armExtendMotor.setPower(0.3);
                        tester = 2;
                    }
                } else if ((hardware.armRotateMotor.getCurrentPosition()) >= ((ticsToMove - angleStart) + 50)) {
                    // move arm up/down because arm is already contracted
                    if (hardware.armExtendMotor.getCurrentPosition() <= (extendMotorStartPos + 50)) {
                        // arm is contracted, move arm
                        hardware.armRotateMotor.setTargetPosition((int)(ticsToMove- angleStart));
                        hardware.armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardware.armRotateMotor.setPower(1.0);
                        tester = 3;
                    } else {
                        // contract arm before moving arm
                        hardware.armExtendMotor.setTargetPosition(extendMotorStartPos);
                        hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardware.armExtendMotor.setPower(0.3);
                        tester = 4;
                    }
                } else {
                    // extend arm
                    double factorExtend;
                    if ((targetPosition > 0) && (targetPosition < 4)){
                        factorExtend = 1.1;
                    }
                    else if (targetPosition == 4){
                        factorExtend = 1.19;
                    }
                    else if (targetPosition == 5){
                        factorExtend = 1.21;
                    }
                    else if (targetPosition == 6){
                        factorExtend = 1.28;
                    }
                    else if (targetPosition == 7){
                        factorExtend = 1.32;
                    }
                    else if (targetPosition == 8){
                        factorExtend = 1.35;
                    }
                    else{
                        factorExtend = 1;
                    }
                    double hypotenuseWithFactor = (hypotenuse * factorExtend);
                    double deltaH = hypotenuseWithFactor - BASE_ARM_LENGTH;
                    double rotationsOfArmExtension = deltaH / SPINDLE_CIRCUMFERENCE;
                    int armExPos = 0;

                    armExPos = (int) ((rotationsOfArmExtension * TICS_PER_ROTATION_EXTENTION_MOTOR));
                    hardware.armExtendMotor.setTargetPosition(armExPos);
                    hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hardware.armExtendMotor.setPower(0.3);
                    tester = 5;
                    if (hardware.armExtendMotor.getCurrentPosition() >= armExPos - 50) {
                        runProgram = false;
                    }
                }
            } else {
                double armRotate = gamepad2.right_stick_y;          //Set multiples for rotating and extending the arm
                double armExtend = gamepad2.left_stick_y;


                if ((armRotate > 0) && (hardware.armLimitTouchBack.getState() == true)) {                                // right stick y to rotate arm up
                    hardware.armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hardware.armRotateMotor.setPower(armRotate);
                } else if ((armRotate < 0) && (hardware.armLimitTouchFront.getState() == true)) {                           // right stick y to rotate arm down
                    hardware.armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hardware.armRotateMotor.setPower(armRotate);
                } else {
                    hardware.armRotateMotor.setPower(0.0);
                }

                if (armExtend > 0) {                                // left stick y to extend arm
                    hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hardware.armExtendMotor.setPower(-armExtend * ARM_EXTEND_MULTIPLE);
                } else if (armExtend < 0) {                           // left stick y to collapse arm
                    hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hardware.armExtendMotor.setPower(-armExtend * ARM_EXTEND_MULTIPLE);
                } else {
                    // set power to the motor to keep it extended
                    armExtendPosition = hardware.armExtendMotor.getCurrentPosition();
                    hardware.armExtendMotor.setTargetPosition(armExtendPosition);
                    hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }

            if (gamepad2.right_bumper) {                        // right bumper to turn wrist right 90 degrees
                hardware.clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);
            }
            if (gamepad2.left_bumper) {                         // left bumper to turn wrist left 90 degrees
                hardware.clawWristServo.setPosition(WRIST_TURN_VERTICAL);
            }
            if (gamepad2.right_trigger > 0) {                    // right trigger turn wrist right
                hardware.clawWristServo.setPosition(hardware.clawWristServo.getPosition() + (gamepad2.right_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad2.left_trigger > 0) {                   // left trigger turn wrist left
                hardware.clawWristServo.setPosition(hardware.clawWristServo.getPosition() - (gamepad2.left_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad2.dpad_up) {                                   // y button on gamepad2 to close claw
                hardware.clawFingersServo.setPosition(FINGERS_CLOSED);
            }
            if (gamepad2.dpad_down) {                                   // a button on gamepad2 to open claw
                hardware.clawFingersServo.setPosition(FINGERS_OPEN);
            }
            if (gamepad2.back) {
                hardware.capstoneServo.setPosition(CAPSTONE_DROPPED);
            }





            // telemetry only below here ...
           /* telemetry.addData("Arm touch front: ", armLimitTouchFront.getState());
            telemetry.addData("Arm touch Back: ", armLimitTouchBack.getState());
            telemetry.addData("Arm extend position", armExtendMotor.getCurrentPosition());
            telemetry.addData("mX: ", mX);
            telemetry.addData("mY: ", mY);

//            String frontLeftDriveMotorValue = frontLeftDriveMotor.getCurrentPosition() + ", " + frontLeftDriveMotor.getDirection() + ", " + frontLeftDriveMotor.getPower();
//            telemetry.addData("frontLeftDriveMotor: ", frontLeftDriveMotorValue);
//            String frontRightDriveMotorValue = frontRightDriveMotor.getCurrentPosition() + ", " + frontRightDriveMotor.getDirection() + ", " + frontRightDriveMotor.getPower();
//            telemetry.addData("frontRightDriveMotor: ", frontRightDriveMotorValue);
//            String backLeftDriveMotorValue = backLeftDriveMotor.getCurrentPosition() + ", " + backLeftDriveMotor.getDirection() + ", " + backLeftDriveMotor.getPower();
//            telemetry.addData("backLeftDriveMotor: ", backLeftDriveMotorValue);
//            String backRightDriveMotorValue = frontRightDriveMotor.getCurrentPosition() + ", " + frontRightDriveMotor.getDirection() + ", " + frontRightDriveMotor.getPower();
//            telemetry.addData("backRightDriveMotor: ", backRightDriveMotorValue);
            String armExtendMotorValue = armExtendMotor.getCurrentPosition() + ", " + armExtendMotor.getMode() + ", " + armExtendMotor.getDirection() + ", " + armExtendMotor.getPower();
            telemetry.addData("armExtendMotor: ", armExtendMotorValue);
            String armRotateMotorValue = armRotateMotor.getCurrentPosition() + ", " + armExtendMotor.getMode() + ", " + armRotateMotor.getDirection() + ", " + armRotateMotor.getPower();
            telemetry.addData("armRotateMotor: ", armRotateMotorValue);
            //  String foundationGrabberServoValue = Double.toString(foundationGrabberServo.getPosition());
            //  telemetry.addData("foundationGrabberServo: ", foundationGrabberServoValue);
            //  telemetry.addData("Battery", this.hardwareMap.voltageSensor.iterator().next().getVoltage());*/
            telemetry.addData("Target Position: ", targetPosition);
            telemetry.addData("Current Speed: ", speedState);
          /*  telemetry.addData("Current: ", currentPosition);
            telemetry.addData("Tester: ", tester);
            telemetry.addData("height", height);
            telemetry.addData("hypotenuse", hypotenuse);
            telemetry.addData("angle", angleInDegrees);
            telemetry.addData("Tics To Move", ticsToMove);*/

            telemetry.update();
            lastState = currentState;
            lastDrive = currentDrive;
            lastRun = currentRun;
        }
    }
}
