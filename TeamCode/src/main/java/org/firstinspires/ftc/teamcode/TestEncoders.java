/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="TestEncoders", group ="Concept")
//@Disabled
public class TestEncoders extends LinearOpMode {

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
    final private double STONE_PICKER_CLOSED = 1;
    final private double STONE_PICKER_OPEN = 0;
    final private double CAPSTONE_NOT_DROPPED = 1;
    final private double CAPSTONE_DROPPED = 0;


    @Override public void runOpMode() {
        /*
         * Retrieve the camera we are to use.
         */
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
        frontLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        armRotateMotor.setDirection(DcMotor.Direction.REVERSE);

        capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);

        waitForStart();
        if(opModeIsActive()) {
//            moveForwardInches(0.5, true, 24);
//            moveForwardInches(0.5, false, 24);
            moveSideInches(0.5, true, 60);
//            moveSideInches(0.5, false, 24);
//            moveTurnDegrees(0.5, true, 90);
//            moveTurnDegrees(0.5, false, 90);
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
        int ticksTol = 25;

        if (direction) {
            FLtarget = (int)(ticsPerInch * inches + frontLeftDriveMotor.getCurrentPosition());
            FRtarget =(int)(ticsPerInch * inches + frontRightDriveMotor.getCurrentPosition());
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

            frontLeftDriveMotor.setPower(wheelPower);
            frontRightDriveMotor.setPower(wheelPower);
            backLeftDriveMotor.setPower(wheelPower);
            backRightDriveMotor.setPower(wheelPower);
        }

        else {
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

            frontLeftDriveMotor.setPower(wheelPower);
            frontRightDriveMotor.setPower(wheelPower);
            backLeftDriveMotor.setPower(wheelPower);
            backRightDriveMotor.setPower(wheelPower);
        }
        long timeToSleep = ((long) ((1000)*(inches / 20)));
        while ((absolute(frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol ) && (absolute(frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())){
            sleep(50);
        }
        sleep(500);

        return (0);
    }
    public double moveSideInches(double wheelPower, boolean direction, double inches) {

        // direction true => right
        // direction false => left
        double sideMultiple = 1.2;
        double ticsPerMotor = 1120 * sideMultiple;
        double circumference = 12.125;
        double ticsPerInch = ticsPerMotor / circumference;

        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 25;

        if (direction) {
            FLtarget = (int)(ticsPerInch * inches + frontLeftDriveMotor.getCurrentPosition());
            FRtarget =(int)(-ticsPerInch * inches + frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int)(-ticsPerInch * inches + backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int)(ticsPerInch * inches + backRightDriveMotor.getCurrentPosition());

            frontLeftDriveMotor.setTargetPosition(FLtarget);
            frontRightDriveMotor.setTargetPosition(FRtarget);
            backLeftDriveMotor.setTargetPosition(BLtarget);
            backRightDriveMotor.setTargetPosition(BRtarget);

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeftDriveMotor.setPower(wheelPower);
            frontRightDriveMotor.setPower(wheelPower);
            backLeftDriveMotor.setPower(wheelPower);
            backRightDriveMotor.setPower(wheelPower);
        }

        else {
            FLtarget = (int)(-ticsPerInch * inches + frontLeftDriveMotor.getCurrentPosition());
            FRtarget =(int)(ticsPerInch * inches + frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int)(ticsPerInch * inches + backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int)(-ticsPerInch * inches + backRightDriveMotor.getCurrentPosition());

            frontLeftDriveMotor.setTargetPosition(FLtarget);
            frontRightDriveMotor.setTargetPosition(FRtarget);
            backLeftDriveMotor.setTargetPosition(BLtarget);
            backRightDriveMotor.setTargetPosition(BRtarget);

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeftDriveMotor.setPower(wheelPower);
            frontRightDriveMotor.setPower(wheelPower);
            backLeftDriveMotor.setPower(wheelPower);
            backRightDriveMotor.setPower(wheelPower);
        }
        long timeToSleep = ((long) ((1000)*((inches / 12))));
        while ((absolute(frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol ) && (absolute(frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())){
            sleep(50);
        }
        sleep(500);

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

            frontLeftDriveMotor.setPower(wheelPower);
            frontRightDriveMotor.setPower(wheelPower);
            backLeftDriveMotor.setPower(wheelPower);
            backRightDriveMotor.setPower(wheelPower);
        }

        else {
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

            frontLeftDriveMotor.setPower(wheelPower);
            frontRightDriveMotor.setPower(wheelPower);
            backLeftDriveMotor.setPower(wheelPower);
            backRightDriveMotor.setPower(wheelPower);
        }
        long timeToSleep = ((long)((1000)*((1.5)*(degrees / 90))));
        while ((absolute(frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol ) && (absolute(frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())){
            sleep(50);
        }
        sleep(500);

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