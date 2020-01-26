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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class Hardware {

    BNO055IMU imu = null;
    WebcamName webcamName = null;
    public DistanceSensor sensorLeft = null;
    public DistanceSensor sensorRight = null;
    public DcMotor frontLeftDriveMotor = null;
    public DcMotor frontRightDriveMotor = null;
    public DcMotor backLeftDriveMotor = null;
    public DcMotor backRightDriveMotor = null;
    public DcMotor armRotateMotor = null;
    public DcMotor armExtendMotor = null;
    public Servo stoneServoRight = null;                    // place holder
    public Servo stoneServoLeft = null;
    public Servo smallStoneServoRight = null;                    // place holder
    public Servo smallStoneServoLeft = null;
    public Servo clawWristServo = null;
    public Servo clawFingersServo = null;
    public Servo foundationGrabberServoLeft = null;
    public Servo foundationGrabberServoRight = null;
    public DigitalChannel armLimitTouchFront = null;
    public DigitalChannel armLimitTouchBack = null;
    public Servo capstoneServo = null;
    public Servo deliveryServoLeft = null;
    public Servo deliveryServoRight = null;


    Hardware(HardwareMap hardwareMap){
        webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // game controller #1
        frontLeftDriveMotor = hardwareMap.dcMotor.get("front_left_drive");
        frontRightDriveMotor = hardwareMap.dcMotor.get("front_right_drive");
        backLeftDriveMotor = hardwareMap.dcMotor.get("back_left_drive");
        backRightDriveMotor = hardwareMap.dcMotor.get("back_right_drive");
        // game controller #2
        armRotateMotor = hardwareMap.dcMotor.get("arm_rotate_motor");
        armExtendMotor = hardwareMap.dcMotor.get("arm_extend_motor");
        stoneServoRight = hardwareMap.servo.get( "stone_picker_right");
        stoneServoLeft = hardwareMap.servo.get("stone_picker_left");
        smallStoneServoRight = hardwareMap.servo.get("small_stone_picker_right");
        smallStoneServoLeft = hardwareMap.servo.get("small_stone_picker_left");
        clawWristServo = hardwareMap.servo.get("claw_wrist");
        clawFingersServo = hardwareMap.servo.get("claw_fingers");
        foundationGrabberServoLeft = hardwareMap.servo.get("foundation_servo_left");
        foundationGrabberServoRight = hardwareMap.servo.get("foundation_servo_right");
        deliveryServoLeft = hardwareMap.servo.get("delivery_servo_left");
        deliveryServoRight = hardwareMap.servo.get("delivery_servo_right");
        sensorLeft = hardwareMap.get(DistanceSensor.class, "distance_left");
        sensorRight = hardwareMap.get(DistanceSensor.class, "distance_right");

        armLimitTouchFront = hardwareMap.digitalChannel.get("arm_limit_touch_front");
        armLimitTouchBack = hardwareMap.digitalChannel.get("arm_limit_touch_back");
        capstoneServo = hardwareMap.servo.get("capstone_servo");
    }
}