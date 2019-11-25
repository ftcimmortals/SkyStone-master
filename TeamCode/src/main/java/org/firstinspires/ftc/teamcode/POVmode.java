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
    final private double FAST_DRIVE_MULTIPLE = 1.0;         // multiple for driving
    final private double SLOW_DRIVE_MULTIPLE = 0.5;
    final private double WRIST_TURN_MULTIPLE = 0.005;       // multiple for wrist turn fine tuning
    final private double ARM_EXTEND_MULTIPLE = 0.5;         // multiple for arm extend

    final private double WRIST_TURN_HORIZONTAL = 0.5;       // wrist turn for horizontal block
    final private double WRIST_TURN_VERTICAL = 0.0;         // wrist turn for vertical block

    final private double FINGERS_OPEN = 0.05;               // open claw
    final private double FINGERS_CLOSED = 0.5;              // closed claw

    final private double FOUNDATION_GRABBER_DOWN = 0.40;    // grabber down
    final private double FOUNDATION_GRABBER_UP = 1;      // grabber up

    final private double STONE_PICKER_CLOSED = 1;
    final private double STONE_PICKER_OPEN = 0;

    final private double CAPSTONE_DROPPED = 1;
    final private double CAPSTONE_NOT_DROPPED = 0;

    final private int TICS_PER_BLOCK = 1500;
    final private int ARM_LENGTH = 13;      // inches
    final private float SPINDLE_DIAMETER = 1.8f;
    final private double SPINDLE_CIRCUMFERENCE = SPINDLE_DIAMETER * Math.PI;
    final private double TICS_PER_ROTATION = 1425.2;

    private ElapsedTime runtime = new ElapsedTime();

    // Motors
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
    boolean currentState = false;
    boolean lastState = false;

    int currentPosition = 0;
    int targetPosition = 0;

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
        stoneServo = hardwareMap.get(Servo.class, "stone_picker");
        clawWristServo = hardwareMap.get(Servo.class, "claw_wrist");
        clawFingersServo = hardwareMap.get(Servo.class, "claw_fingers");
        foundationGrabberServo = hardwareMap.get(Servo.class, "foundation_grabber");

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

        clawFingersServo.setPosition(FINGERS_OPEN);                   //set fingers position at init
        foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);      //set grabber position at init
        capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);

        int armExtendPosition;
        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder at init
        armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int extendMotorStartPos = armExtendMotor.getCurrentPosition();
        int rotateMotorStartPos = armRotateMotor.getCurrentPosition();

        int tester = 0;

        boolean runProgram = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Setup a variable for each drive wheel to save power level for telemetry

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // DRIVER 1 : Gamepad 1 controls

            double turn = gamepad1.right_stick_x;
            double mY = gamepad1.left_stick_y;
            double mX = gamepad1.left_stick_x;

            double powerflD = (mY * -FAST_DRIVE_MULTIPLE);
            double powerfrD = (mY * -FAST_DRIVE_MULTIPLE);
            double powerblD = (mY * -FAST_DRIVE_MULTIPLE);
            double powerbrD = (mY * -FAST_DRIVE_MULTIPLE);

            double powerflS = (mX * FAST_DRIVE_MULTIPLE);
            double powerfrS = (mX * -FAST_DRIVE_MULTIPLE);
            double powerblS = (mX * -FAST_DRIVE_MULTIPLE);
            double powerbrS = (mX * FAST_DRIVE_MULTIPLE);

            double powerfl = powerflS + powerflD;
            double powerfr = powerfrS + powerfrD;
            double powerbl = powerblS + powerblD;
            double powerbr = powerbrS + powerbrD;

            frontLeftDriveMotor.setPower(powerfl);
            frontRightDriveMotor.setPower(powerfr);
            backLeftDriveMotor.setPower(powerbl);
            backRightDriveMotor.setPower(powerbr);

            if(turn != 0) {
                frontLeftDriveMotor.setPower(-turn * FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(turn * FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(-turn * FAST_DRIVE_MULTIPLE);
                frontRightDriveMotor.setPower(turn * FAST_DRIVE_MULTIPLE);
            }
            if (gamepad1.right_trigger> 0) {                    // right trigger turn wrist right
                clawWristServo.setPosition(clawWristServo.getPosition() + (gamepad2.right_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad1.left_trigger > 0) {                   // left trigger turn wrist left
                clawWristServo.setPosition(clawWristServo.getPosition() - (gamepad2.left_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad1.dpad_up){
                stoneServo.setPosition(STONE_PICKER_OPEN);
            }
            if(gamepad1.dpad_down){
                stoneServo.setPosition(STONE_PICKER_CLOSED);
            }

            /*
                temporary code to automate arm
             */

            // tics = (17.87/360)/1452.2
            // 1452.2 - https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-50-9-1-ratio-117-rpm-3-3-5v-encoder/
            // sin theta = P/H, P = 4inches, H = 13 inches = .307, inverse sin .307 = 17.87 https://www.mathsisfun.com/scientific-calculator.html
            // ABOVE MIGHT BE WRONG
            // assuming 15 degrees movement per rotation ~15 degrees. 360/24

            currentState = gamepad1.a;
            if ((currentState == false) && (lastState == true)){
                targetPosition++;
            }
            if(gamepad1.x){
                targetPosition = 0;
            }
            if (gamepad1.b) {
                runProgram = true;
            }
            if (runProgram) {
                if (armLimitTouchFront.getState() == false) {
                    armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition());
                    armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRotateMotor.setPower(1.0);
                }

                //calculating current position

                currentPosition = (int)(armRotateMotor.getCurrentPosition() / TICS_PER_BLOCK);
                if (currentPosition < targetPosition) {
                    if ((armRotateMotor.getCurrentPosition()) <= ((targetPosition * TICS_PER_BLOCK) - 50)) {
                        // move arm up/down because arm is already contracted
                        if (armExtendMotor.getCurrentPosition() <= (extendMotorStartPos + 50)) {
                            // arm is contracted, move arm
                            armRotateMotor.setTargetPosition(TICS_PER_BLOCK * targetPosition);
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
                    } else {
                        // extend arm
                        double bSquared = Math.pow(ARM_LENGTH, 2);
                        double x = Math.pow(4 + (4 * targetPosition), 2);
                        double newH = Math.sqrt(bSquared + x);
                        double deltaH = newH - ARM_LENGTH;
                        double rotationsOfArmExtension = deltaH / SPINDLE_CIRCUMFERENCE;
                        int armExPos = 0;

                        armExPos = (int) (rotationsOfArmExtension * TICS_PER_ROTATION);
                        armExtendMotor.setTargetPosition(armExPos);
                        armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armExtendMotor.setPower(0.3);
                        tester = 3;

                        if (armExtendMotor.getCurrentPosition() >= Math.abs(armExPos) - 50) {
                            runProgram = false;
                        }
                    }
                }else{
                    if ((armRotateMotor.getCurrentPosition()) >= ((targetPosition * TICS_PER_BLOCK) + 50)) {
                        // move arm up/down because arm is already contracted
                        if (armExtendMotor.getCurrentPosition() <= (extendMotorStartPos + 50)) {
                            // arm is contracted, move arm
                            armRotateMotor.setTargetPosition(TICS_PER_BLOCK * targetPosition);
                            armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armRotateMotor.setPower(1.0);
                            tester = 4;
                        } else {
                            // contract arm before moving arm
                            armExtendMotor.setTargetPosition(extendMotorStartPos);
                            armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armExtendMotor.setPower(0.3);
                            tester = 5;
                        }
                    }else{
                        // extend arm
                        double bSquared = Math.pow(ARM_LENGTH, 2);
                        double x = Math.pow(4 + (4 * targetPosition), 2);
                        double newH = Math.sqrt(bSquared + x);
                        double deltaH = newH - ARM_LENGTH;
                        double rotationsOfArmExtension = deltaH / SPINDLE_CIRCUMFERENCE;
                        int armExPos = 0;

                        armExPos = (int) (rotationsOfArmExtension * TICS_PER_ROTATION);
                        armExtendMotor.setTargetPosition(armExPos);
                        armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armExtendMotor.setPower(0.3);
                        tester = 6;

                        if (armExtendMotor.getCurrentPosition() <= Math.abs(armExPos) + 50) {
                            runProgram = false;
                        }
                    }
                }
            }else{
                //DRIVER 2 : Gamepad 2 controls

                double armRotate = gamepad2.right_stick_y;          //Set multiples for rotating and extending the arm
                double armExtend = gamepad2.left_stick_y;

                if ((armRotate > 0) && (armLimitTouchBack.getState() == true)) {                                // right stick y to rotate arm up
                    armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armRotateMotor.setPower(1);
                } else if ((armRotate < 0) && (armLimitTouchFront.getState() == true)) {                           // right stick y to rotate arm down
                    armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armRotateMotor.setPower(-1);
                }
                else{
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

            if(gamepad2.dpad_up){
                foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
            }
            if (gamepad2.dpad_down) {
                foundationGrabberServo.setPosition(FOUNDATION_GRABBER_DOWN);
            }
            if (gamepad2.right_bumper) {                        // right bumper to turn wrist right 90 degrees
                clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);
            }
            if (gamepad2.left_bumper) {                         // left bumper to turn wrist left 90 degrees
                clawWristServo.setPosition(WRIST_TURN_VERTICAL);
            }
            if (gamepad2.right_trigger> 0) {                    // right trigger turn wrist right
                clawWristServo.setPosition(clawWristServo.getPosition() + (gamepad2.right_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad2.left_trigger > 0) {                   // left trigger turn wrist left
                clawWristServo.setPosition(clawWristServo.getPosition() - (gamepad2.left_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad2.y) {                                   // y button on gamepad2 to close claw
                clawFingersServo.setPosition(FINGERS_CLOSED);
            }
            if (gamepad2.a) {                                   // a button on gamepad2 to open claw
                clawFingersServo.setPosition(FINGERS_OPEN);
            }
            if ((gamepad2.x)&&(gamepad2.dpad_left)){
                capstoneServo.setPosition(CAPSTONE_DROPPED);
            }
            if((gamepad2.b)&&(gamepad2.dpad_left)){
                capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);
            }
           /* if((armRotate == 0) && (armExtend == 0) && (clawFingers.getPosition() == FINGERS_OPEN) && (!gamepad2.left_bumper) && (gamepad2.right_trigger == 0) && (gamepad2.left_trigger == 0)){
                clawWrist.setPosition(WRIST_TURN_HORIZONTAL);
                setting position to always be horizontal unless something is pressed or fingers are closed
            //            }*/
            //
            //
            //            // reset power to all motors
            frontLeftDriveMotor.setPower(0.0);
            frontRightDriveMotor.setPower(0.0);
            backLeftDriveMotor.setPower(0.0);
            backRightDriveMotor.setPower(0.0);


            // telemetry only below here ...
            telemetry.addData("Arm touch front: ", armLimitTouchFront.getState());
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
          //  telemetry.addData("Battery", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("Target Position: ", targetPosition);
            telemetry.addData("Current: ", currentPosition);
            telemetry.addData("Tester: ", tester);

            telemetry.update();
            lastState = currentState;
        }
    }
}