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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="POV_Mode_Test", group="Linear Opmode")
// @Disabled
public class POVmodeTest extends CommonMethods {

    // define IMU
    private BNO055IMU imu;

    // Constants
    private ElapsedTime runtime = new ElapsedTime();

    // Motors

    boolean currentState = false;
    boolean lastState = false;
    boolean currentRun = false;
    boolean currentRun2 = false;
    boolean lastRun = false;
    boolean lastRun2 = false;

    String armReset = "NO";

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
        hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_UP);
        hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_UP);
        hardware.capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);

        hardware.armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        hardware.armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);

        int armExtendPosition;
        hardware.armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder at init
        hardware.armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);
        hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
        hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
        hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
        hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
        hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_UP);
        hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_UP);
        hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
        hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);
        int extendMotorStartPos = hardware.armExtendMotor.getCurrentPosition();
        int rotateMotorStartPos = hardware.armRotateMotor.getCurrentPosition();

        hardware.frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.armRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        if(gamepad1.left_trigger > 0){
            liftOrDropStones(1, 1, hardware);
            liftOrDropStones(-1, 1, hardware);

        }else if(gamepad1.right_trigger > 0){
            liftOrDropStones(1, -1, hardware);
            liftOrDropStones(-1, -1, hardware);

        }
        if(gamepad1.left_bumper){
            hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_DOWN);
            hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN);
        }
        if(gamepad1.right_bumper){
            hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
            hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
        }
        if(gamepad1.a){
            hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
            hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
        }

        }
    }
}
