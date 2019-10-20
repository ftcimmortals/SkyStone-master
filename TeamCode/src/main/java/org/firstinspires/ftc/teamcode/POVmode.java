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
    final private float FAST_DRIVE_MULTIPLE = 1f;           // multiple for driving
    final private double WRIST_TURN_MULTIPLE = 0.005;       //multiple for wrist turn fine tuning
    final private double ARM_EXTEND_MULTIPLE = 1;      //multiple for arm extend
    final private double WRIST_TURN_HORIZONTAL = 0.5;       // wrist turn for horizontal block
    final private double WRIST_TURN_VERTICAL = 0.0;         // wrist turn for vertical block
    final private double FINGERS_OPEN = 0.05;               // open claw
    final private double FINGERS_CLOSED = 0.5;             // closed claw
    final private double FOUNDATION_GRABBER_DOWN = 0.25;     //grabber down
    final private double FOUNDATION_GRABBER_UP = 0.85;

    private ElapsedTime runtime = new ElapsedTime();

    // Motors
    private DcMotor frontLeftDriveMotor = null;
    private DcMotor frontRightDriveMotor = null;
    private DcMotor backLeftDriveMotor = null;
    private DcMotor backRightDriveMotor = null;
    private DcMotor armRotateMotor = null;
    private DcMotor armExtendMotor = null;
    private Servo clawElbowServo = null;
    private Servo clawWristServo = null;
    private Servo clawFingersServo = null;
    private Servo foundationGrabberServo = null;
    private DigitalChannel armLimitTouchFront = null;
    private DigitalChannel armLimitTouchBack = null;

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
        armExtendMotor = hardwareMap.get(DcMotor.class, "arm_lift_motor");
        clawElbowServo = hardwareMap.get(Servo.class, "claw_elbow");
        clawWristServo = hardwareMap.get(Servo.class, "claw_wrist");
        clawFingersServo = hardwareMap.get(Servo.class, "claw_fingers");
        foundationGrabberServo = hardwareMap.get(Servo.class, "foundation_grabber");

        armLimitTouchFront = hardwareMap.get(DigitalChannel.class, "arm_limit_touch_front");
        armLimitTouchBack = hardwareMap.get(DigitalChannel.class, "arm_limit_touch_back");
        armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        armRotateMotor.setDirection(DcMotor.Direction.REVERSE);

        clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);//set wrist position at init
        clawFingersServo.setPosition(FINGERS_OPEN);//set fingers position at init
        foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);//set grabber position on init

        int armExtendPosition;
        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset encoder at init
        armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Setup a variable for each drive wheel to save power level for telemetry

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // DRIVER 1 : Gamepad 1 controls

            double multiple = 1;                                            // set multiples for drive and side motions
            double drive = gamepad1.left_stick_y;
            double side = gamepad1.right_stick_x;


            if (drive < 0) {                                                 // use left stick y to move forward
                multiple = -1 * drive;
                frontLeftDriveMotor.setPower(multiple * -FAST_DRIVE_MULTIPLE);
                frontRightDriveMotor.setPower(multiple * -FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(multiple * -FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(multiple * -FAST_DRIVE_MULTIPLE);
            } else if (drive > 0) {                                           // use left stick y to move back
                multiple = drive;
                frontLeftDriveMotor.setPower(multiple * FAST_DRIVE_MULTIPLE);
                frontRightDriveMotor.setPower(multiple * FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(multiple * FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(multiple * FAST_DRIVE_MULTIPLE);
            } else if (side > 0) {                                            // use right stick x to move right
                multiple = side;
                frontRightDriveMotor.setPower(multiple * FAST_DRIVE_MULTIPLE);
                frontLeftDriveMotor.setPower(multiple * -FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(multiple * FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(multiple * -FAST_DRIVE_MULTIPLE);
            } else if (side < 0) {                                            // use right stick x to move left
                multiple = -1 * side;
                frontRightDriveMotor.setPower(multiple * -FAST_DRIVE_MULTIPLE);
                frontLeftDriveMotor.setPower(multiple * FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(multiple * -FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(multiple * FAST_DRIVE_MULTIPLE);
            } else if (gamepad1.left_bumper) {                              // use left bumper for turning left
                frontLeftDriveMotor.setPower(FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(-FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(FAST_DRIVE_MULTIPLE);
                frontRightDriveMotor.setPower(-FAST_DRIVE_MULTIPLE);
            }                                                             // use right bumper for turning right
            else if (gamepad1.right_bumper) {
                frontLeftDriveMotor.setPower(-FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(-FAST_DRIVE_MULTIPLE);
                frontRightDriveMotor.setPower(FAST_DRIVE_MULTIPLE);
            }


            //DRIVER 2 : Gamepad 2 controls

            double armRotate = gamepad2.right_stick_y;          //Set multiples for rotating and extending the arm
            double armExtend = gamepad2.left_stick_y;

            if ((armRotate > 0) && (armLimitTouchBack.getState() == true)) {                                // right stick y to rotate arm up
                armRotateMotor.setPower(1);
            } else if ((armRotate < 0) && (armLimitTouchFront.getState() == true)) {                           // right stick y to rotate arm down
                armRotateMotor.setPower(-1);
            }

            if (armExtend > 0) {                                // left stick y to extend arm
                armExtendMotor.setPower(-armExtend);
            } else if (armExtend < 0) {                           // left stick y to collapse arm
                armExtendMotor.setPower(-armExtend);
            }
            else {
                // set power to the motor to keep it extended
                armExtendPosition = armExtendMotor.getCurrentPosition();
                armExtendMotor.setTargetPosition(armExtendPosition);
                armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            if (gamepad2.left_trigger > 0) {                    // left trigger turn wrist left
                clawWristServo.setPosition(clawWristServo.getPosition() + (gamepad2.left_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad2.right_trigger > 0) {                   // right trigger turn wrist right
                clawWristServo.setPosition(clawWristServo.getPosition() - (gamepad2.right_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad2.y) {                                   // y button on gamepad2 to close claw
                clawFingersServo.setPosition(FINGERS_CLOSED);
            }
            if (gamepad2.a) {                                   // a button on gamepad2 to open claw
                clawFingersServo.setPosition(FINGERS_OPEN);
            }
           /* if((armRotate == 0) && (armExtend == 0) && (clawFingers.getPosition() == FINGERS_OPEN) && (!gamepad2.left_bumper) && (gamepad2.right_trigger == 0) && (gamepad2.left_trigger == 0)){
                clawWrist.setPosition(WRIST_TURN_HORIZONTAL);
                setting position to always be horizontal unless something is pressed or fingers are closed
            }*/


            // reset power to all motors
            frontLeftDriveMotor.setPower(0.0);
            frontRightDriveMotor.setPower(0.0);
            backLeftDriveMotor.setPower(0.0);
            backRightDriveMotor.setPower(0.0);
            armRotateMotor.setPower(0.0);

            // telemetry only below here ...
            telemetry.addData("Arm touch front: ", armLimitTouchFront.getState());
            telemetry.addData("Arm touch Back: ", armLimitTouchBack.getState());
            telemetry.addData("Arm extend position", armExtendMotor.getCurrentPosition());

//            String frontLeftDriveMotorValue = frontLeftDriveMotor.getCurrentPosition() + ", " + frontLeftDriveMotor.getDirection() + ", " + frontLeftDriveMotor.getPower();
//            telemetry.addData("frontLeftDriveMotor: ", frontLeftDriveMotorValue);
//            String frontRightDriveMotorValue = frontRightDriveMotor.getCurrentPosition() + ", " + frontRightDriveMotor.getDirection() + ", " + frontRightDriveMotor.getPower();
//            telemetry.addData("frontRightDriveMotor: ", frontRightDriveMotorValue);
//            String backLeftDriveMotorValue = backLeftDriveMotor.getCurrentPosition() + ", " + backLeftDriveMotor.getDirection() + ", " + backLeftDriveMotor.getPower();
//            telemetry.addData("backLeftDriveMotor: ", backLeftDriveMotorValue);
//            String backRightDriveMotorValue = frontRightDriveMotor.getCurrentPosition() + ", " + frontRightDriveMotor.getDirection() + ", " + frontRightDriveMotor.getPower();
//            telemetry.addData("backRightDriveMotor: ", backRightDriveMotorValue);
            String armExtendMotorValue = armExtendMotor.getCurrentPosition() + ", " + armExtendMotor.getDirection() + ", " + armExtendMotor.getPower();
            telemetry.addData("armExtendMotor: ", armExtendMotorValue);
            String armRotateMotorValue = armRotateMotor.getCurrentPosition() + ", " + armRotateMotor.getDirection() + ", " + armRotateMotor.getPower();
            telemetry.addData("armRotateMotor: ", armRotateMotorValue);
            String foundationGrabberServoValue = Double.toString(foundationGrabberServo.getPosition());
            telemetry.addData("foundationGrabberServo: ", foundationGrabberServoValue);

            telemetry.update();
        }
    }
}