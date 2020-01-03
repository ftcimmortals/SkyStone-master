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
public class POVmode extends LinearOpMode {

    // define IMU
    private BNO055IMU imu;

    // Constants
    final private double SLOW_DRIVE_MULTIPLE = 0.5;
    final private double FAST_DRIVE_MULTIPLE = 1;
    final private double WRIST_TURN_MULTIPLE = 0.005;       // multiple for wrist turn fine tuning
    final private double ARM_EXTEND_MULTIPLE = 0.5;         // multiple for arm extend

    final private double WRIST_TURN_HORIZONTAL = 0.55;       // wrist turn for horizontal block
    final private double WRIST_TURN_VERTICAL = 0.05;         // wrist turn for vertical block

    final private double FINGERS_OPEN = 0.05;               // open claw
    final private double FINGERS_CLOSED = 0.5;              // closed claw

    final private double FOUNDATION_GRABBER_DOWN = 0.40;    // grabber down
    final private double FOUNDATION_GRABBER_UP = 1;         // grabber up


    final private double DELIVERY_LEFT_OPEN_FULLY = 0.885;
    final private double DELIVERY_RIGHT_OPEN_FULLY = 0.09;
    final private double DELIVERY_SERVO_IN_LEFT = 0.475;
    final private double DELIVERY_SERVO_IN_RIGHT = 0.475;
    final private double DELIVERY_SERVO_LEFT_IN_A_LITTLE = 0.8;
    final private double DELIVERY_SERVO_RIGHT_IN_A_LITTLE = 0.2;

    final private double CAPSTONE_NOT_DROPPED = 1;
    final private double CAPSTONE_DROPPED = 0;

    final private double BASE_ARM_LENGTH = 13;
    final private double SPINDLE_CIRCUMFERENCE = (2.5 * (Math.PI));
    final private double TICS_PER_ROTATION_ROTATION_MOTOR = 1425.2;
    final private double TICS_PER_ROTATION_EXTENTION_MOTOR = 537.6;
    final private double TICS_PER_DEGREE = (TICS_PER_ROTATION_ROTATION_MOTOR * 24) / 360;



    private ElapsedTime runtime = new ElapsedTime();

    // Motors
    private DcMotor frontLeftDriveMotor = null;
    private DcMotor frontRightDriveMotor = null;
    private DcMotor backLeftDriveMotor = null;
    private DcMotor backRightDriveMotor = null;
    private DcMotor armRotateMotor = null;
    private DcMotor armExtendMotor = null;
    private Servo stoneServoRed = null;                    // place holder
    private Servo stoneServoBlue = null;
    private Servo clawWristServo = null;
    private Servo clawFingersServo = null;
    private Servo foundationGrabberServo = null;
    private DigitalChannel armLimitTouchFront = null;
    private DigitalChannel armLimitTouchBack = null;
    private Servo capstoneServo = null;
    private Servo deliveryServoLeft = null;
    private Servo deliveryServoRight = null;
    final private double STONE_PICKER_CLOSED_RED = 1;
    final private double STONE_PICKER_CLOSED_BLUE = 0.1;
    final private double STONE_PICKER_OPEN_RED = 0;
    final private double STONE_PICKER_OPEN_BLUE = 1;


    boolean currentState = false;
    boolean lastState = false;
    boolean run = false;

    int currentPosition = 0;
    int targetPosition = 0;
    double numberToClear;
    double driveMultiple = SLOW_DRIVE_MULTIPLE;
    boolean currentDrive = false;
    boolean lastDrive = false;

    float theta = 0.0f;


    @Override
    public void runOpMode() {

        /* Initialize the hardware variables. Note that the strings used here as parameters
            to 'get' must correspond to the names assigned during the robot configuration
            step (using the FTC Robot Controller app on the phone). */

        // game controller #1
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDriveMotor = hardwareMap.get(DcMotor.class, "back_right_drive");
        // game controller #2
        armRotateMotor = hardwareMap.get(DcMotor.class, "arm_rotate_motor");
        armExtendMotor = hardwareMap.get(DcMotor.class, "arm_extend_motor");
        stoneServoRed = hardwareMap.get(Servo.class, "stone_picker_red");
        stoneServoBlue = hardwareMap.get(Servo.class, "stone_picker_blue");
        clawWristServo = hardwareMap.get(Servo.class, "claw_wrist");
        clawFingersServo = hardwareMap.get(Servo.class, "claw_fingers");
        foundationGrabberServo = hardwareMap.get(Servo.class, "foundation_grabber");
        deliveryServoLeft = hardwareMap.get(Servo.class, "delivery_servo_left");
        deliveryServoRight = hardwareMap.get(Servo.class, "delivery_servo_right");

        armLimitTouchFront = hardwareMap.get(DigitalChannel.class, "arm_limit_touch_front");
        armLimitTouchBack = hardwareMap.get(DigitalChannel.class, "arm_limit_touch_back");
        capstoneServo = hardwareMap.get(Servo.class, "capstone_servo");


        armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        armRotateMotor.setDirection(DcMotor.Direction.REVERSE);

        clawFingersServo.setPosition(FINGERS_OPEN);
        clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);
        foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);      //set grabber position at init
        capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);

        int armExtendPosition;
        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder at init
        armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);
        stoneServoBlue.setPosition(STONE_PICKER_CLOSED_BLUE);
        stoneServoRed.setPosition(STONE_PICKER_CLOSED_RED);
        foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
        deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
        deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);
        int extendMotorStartPos = armExtendMotor.getCurrentPosition();
        int rotateMotorStartPos = armRotateMotor.getCurrentPosition();
       /* frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        int tester = 0;

        boolean runProgram = false;

        double lastPosLeft = DELIVERY_SERVO_IN_LEFT;
        double lastPosRight = DELIVERY_SERVO_IN_RIGHT;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Setup a variable for each drive wheel to save power level for telemetry

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double x = armRotateMotor.getCurrentPosition() / TICS_PER_DEGREE;
            double MultFactor = 1 /*((-((MAX_FACTOR/135)) * x) + MAX_FACTOR)*/;

            // DRIVER 1 : Gamepad 1 controls

            double turn = gamepad1.right_stick_x;
            double mY = gamepad1.left_stick_y;
            double mX = gamepad1.left_stick_x;

            currentDrive = gamepad1.y;

            if((!currentDrive) && (lastDrive)) {
                if(driveMultiple == SLOW_DRIVE_MULTIPLE) {
                    driveMultiple = FAST_DRIVE_MULTIPLE;
                }
                else if (driveMultiple == FAST_DRIVE_MULTIPLE) {
                    driveMultiple = SLOW_DRIVE_MULTIPLE;
                }
            }

            double powerflD = (mY * -driveMultiple);
            double powerfrD = (mY * -driveMultiple);
            double powerblD = (mY * -driveMultiple);
            double powerbrD = (mY * -driveMultiple);

            double powerflS = (mX * driveMultiple);
            double powerfrS = (mX * -driveMultiple);
            double powerblS = ((mX * -driveMultiple) * MultFactor);
            double powerbrS = ((mX * driveMultiple) * MultFactor);

            double powerfl = powerflS + powerflD;
            double powerfr = powerfrS + powerfrD;
            double powerbl = powerblS + powerblD;
            double powerbr = powerbrS + powerbrD;

            frontLeftDriveMotor.setPower(powerfl);
            frontRightDriveMotor.setPower(powerfr);
            backLeftDriveMotor.setPower(powerbl);
            backRightDriveMotor.setPower(powerbr);

            if (turn != 0) {
                frontLeftDriveMotor.setPower(-turn * driveMultiple);
                backRightDriveMotor.setPower(turn * driveMultiple);
                backLeftDriveMotor.setPower(-turn * driveMultiple);
                frontRightDriveMotor.setPower(turn * driveMultiple);
            }

            if (gamepad1.dpad_up) {
                foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
            }
            if (gamepad1.dpad_down) {
                foundationGrabberServo.setPosition(FOUNDATION_GRABBER_DOWN);
            }


            if (gamepad1.x) {
                lastPosLeft = DELIVERY_LEFT_OPEN_FULLY;
                lastPosRight = DELIVERY_RIGHT_OPEN_FULLY;
                deliveryServoLeft.setPosition(lastPosLeft);
                deliveryServoRight.setPosition(lastPosRight);
            }
            if (gamepad1.b) {
                lastPosLeft = DELIVERY_SERVO_IN_LEFT;
                lastPosRight = DELIVERY_SERVO_IN_RIGHT;
                deliveryServoLeft.setPosition(lastPosLeft);
                deliveryServoRight.setPosition(lastPosRight);
            }
            if ((deliveryServoLeft.getPosition() == DELIVERY_LEFT_OPEN_FULLY) && (gamepad1.left_trigger > 0)){
                lastPosLeft = DELIVERY_SERVO_LEFT_IN_A_LITTLE;
                lastPosRight = DELIVERY_SERVO_RIGHT_IN_A_LITTLE;
                deliveryServoLeft.setPosition(lastPosLeft);
                deliveryServoRight.setPosition(lastPosRight);
            }else if ((gamepad1.left_trigger == 0) && (!gamepad1.b) && (lastPosLeft != DELIVERY_SERVO_IN_LEFT)){
                    deliveryServoLeft.setPosition(DELIVERY_LEFT_OPEN_FULLY);
                    deliveryServoRight.setPosition(DELIVERY_RIGHT_OPEN_FULLY);
            }
            /*if ((deliveryServoLeft.getPosition() == DELIVERY_LEFT_OPEN_FULLY) && (gamepad1.right_trigger > 0)){
                lastPosLeft = DELIVERY_SERVO_IN_LEFT;
                lastPosRight = DELIVERY_SERVO_IN_RIGHT;
                deliveryServoLeft.setPosition(lastPosLeft);
                deliveryServoRight.setPosition(lastPosRight);
            }else if ((gamepad1.right_trigger == 0) && (!gamepad1.x) && (lastPosLeft == DELIVERY_SERVO_IN_LEFT)){
                deliveryServoLeft.setPosition(DELIVERY_LEFT_OPEN_FULLY);
                deliveryServoRight.setPosition(DELIVERY_RIGHT_OPEN_FULLY);
            }*/

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
                currentPosition = (int) (armRotateMotor.getCurrentPosition() / ticsPerBlock);
            }

            if (runProgram) {
                if (armLimitTouchFront.getState() == false) {
                    armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition());
                    armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRotateMotor.setPower(1.0);
                }

                double angleStart = (150 * TICS_PER_DEGREE);

                //calculating current position
                if ((armRotateMotor.getCurrentPosition()) <= ((ticsToMove - angleStart) - 50)) {
                    // move arm up/down because arm is already contracted
                    if (armExtendMotor.getCurrentPosition() <= (extendMotorStartPos + 50)) {
                        // arm is contracted, move arm
                        armRotateMotor.setTargetPosition((int)(ticsToMove- angleStart));
                        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armRotateMotor.setPower(1.0);
                        tester = 1;
                    } else {
                        // contract arm before moving arm
                        armExtendMotor.setTargetPosition(extendMotorStartPos);
                        armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armExtendMotor.setPower(0.3);
                        tester = 2;
                    }
                } else if ((armRotateMotor.getCurrentPosition()) >= ((ticsToMove - angleStart) + 50)) {
                    // move arm up/down because arm is already contracted
                    if (armExtendMotor.getCurrentPosition() <= (extendMotorStartPos + 50)) {
                        // arm is contracted, move arm
                        armRotateMotor.setTargetPosition((int)(ticsToMove- angleStart));
                        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armRotateMotor.setPower(1.0);
                        tester = 3;
                    } else {
                        // contract arm before moving arm
                        armExtendMotor.setTargetPosition(extendMotorStartPos);
                        armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armExtendMotor.setPower(0.3);
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

                    armExPos = (int) (rotationsOfArmExtension * TICS_PER_ROTATION_EXTENTION_MOTOR);
                    armExtendMotor.setTargetPosition(armExPos);
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtendMotor.setPower(0.3);
                    tester = 5;
                    if (armExtendMotor.getCurrentPosition() >= armExPos - 50) {
                        runProgram = false;
                    }
                }
            } else {
                double armRotate = gamepad2.right_stick_y;          //Set multiples for rotating and extending the arm
                double armExtend = gamepad2.left_stick_y;


                if ((armRotate > 0) && (armLimitTouchBack.getState() == true)) {                                // right stick y to rotate arm up
                    armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armRotateMotor.setPower(armRotate);
                } else if ((armRotate < 0) && (armLimitTouchFront.getState() == true)) {                           // right stick y to rotate arm down
                    armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armRotateMotor.setPower(armRotate);
                } else {
                    armRotateMotor.setPower(0.0);
                }

                if (armExtend > 0) {                                // left stick y to extend arm
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armExtendMotor.setPower(-armExtend * ARM_EXTEND_MULTIPLE);
                } else if (armExtend < 0) {                           // left stick y to collapse arm
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armExtendMotor.setPower(-armExtend * ARM_EXTEND_MULTIPLE);
                } else {
                    // set power to the motor to keep it extended
                    armExtendPosition = armExtendMotor.getCurrentPosition();
                    armExtendMotor.setTargetPosition(armExtendPosition);
                    armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }

            if (gamepad2.right_bumper) {                        // right bumper to turn wrist right 90 degrees
                clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);
            }
            if (gamepad2.left_bumper) {                         // left bumper to turn wrist left 90 degrees
                clawWristServo.setPosition(WRIST_TURN_VERTICAL);
            }
            if (gamepad2.right_trigger > 0) {                    // right trigger turn wrist right
                clawWristServo.setPosition(clawWristServo.getPosition() + (gamepad2.right_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad2.left_trigger > 0) {                   // left trigger turn wrist left
                clawWristServo.setPosition(clawWristServo.getPosition() - (gamepad2.left_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad2.dpad_up) {                                   // y button on gamepad2 to close claw
                clawFingersServo.setPosition(FINGERS_CLOSED);
            }
            if (gamepad2.dpad_down) {                                   // a button on gamepad2 to open claw
                clawFingersServo.setPosition(FINGERS_OPEN);
            }
            if (gamepad2.back) {
                capstoneServo.setPosition(CAPSTONE_DROPPED);
            }


            //            }*/
            //
            //
            //            // reset power to all motors
            frontLeftDriveMotor.setPower(0.0);
            frontRightDriveMotor.setPower(0.0);
            backLeftDriveMotor.setPower(0.0);
            backRightDriveMotor.setPower(0.0);


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
          /*  telemetry.addData("Current: ", currentPosition);
            telemetry.addData("Tester: ", tester);
            telemetry.addData("height", height);
            telemetry.addData("hypotenuse", hypotenuse);
            telemetry.addData("angle", angleInDegrees);
            telemetry.addData("Tics To Move", ticsToMove);*/

            telemetry.update();
            lastState = currentState;
            lastDrive = currentDrive;
        }
    }
}
