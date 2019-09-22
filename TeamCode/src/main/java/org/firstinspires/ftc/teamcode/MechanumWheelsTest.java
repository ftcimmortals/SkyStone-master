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


@TeleOp(name="MECH TEST", group="Linear Opmode")
// @Disabled
public class MechanumWheelsTest extends LinearOpMode {

    // define IMU
    private BNO055IMU imu;

    // drive multiple (to set motor power)
    final private float FAST_DRIVE_MULTIPLE = 1f;
    private ElapsedTime runtime = new ElapsedTime();

    // define motors
    private DcMotor frontLeftDriveMotor = null;
    private DcMotor frontRightDriveMotor = null;
    private DcMotor backLeftDriveMotor = null;
    private DcMotor backRightDriveMotor = null;


    @Override
    public void runOpMode() {


        /* Initialize the hardware variables. Note that the strings used here as parameters
        to 'get' must correspond to the names assigned during the robot configuration
         step (using the FTC Robot Controller app on the phone).*/
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDriveMotor = hardwareMap.get(DcMotor.class, "back_right_drive");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Setup a variable for each drive wheel to save power level for telemetry

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
             * DRIVER 1 : Gamepad 1 controls
             */
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            // drive of robot using gamepad1 left and right sticks
            double multiple;
            double drive = gamepad1.left_stick_y;
            double side = gamepad1.right_stick_x;

            telemetry.addData("drive: ", drive);

            if(drive < 0){
                multiple = -1*drive;
                frontLeftDriveMotor.setPower(multiple*-FAST_DRIVE_MULTIPLE);
                frontRightDriveMotor.setPower(multiple*-FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(multiple*-FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(multiple*-FAST_DRIVE_MULTIPLE);
            }

            else if(drive > 0){
                multiple = drive;
                frontLeftDriveMotor.setPower(multiple*FAST_DRIVE_MULTIPLE);
                frontRightDriveMotor.setPower(multiple*FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(multiple*FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(multiple*FAST_DRIVE_MULTIPLE);
            }

            else if(side > 0){
                multiple = side;
                frontRightDriveMotor.setPower(multiple*FAST_DRIVE_MULTIPLE);
                frontLeftDriveMotor.setPower(multiple*-FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(multiple*FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(multiple*-FAST_DRIVE_MULTIPLE);
            }
            else if(side < 0){
                multiple = -1*side;
                frontRightDriveMotor.setPower(multiple*-FAST_DRIVE_MULTIPLE);
                frontLeftDriveMotor.setPower(multiple*FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(multiple*-FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(multiple*FAST_DRIVE_MULTIPLE);
            }
            else if(gamepad1.left_bumper){
                frontLeftDriveMotor.setPower(FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(-FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(FAST_DRIVE_MULTIPLE);
                frontRightDriveMotor.setPower(-FAST_DRIVE_MULTIPLE);
            }
            else if(gamepad1.right_bumper){
                frontLeftDriveMotor.setPower(-FAST_DRIVE_MULTIPLE);
                backRightDriveMotor.setPower(FAST_DRIVE_MULTIPLE);
                backLeftDriveMotor.setPower(-FAST_DRIVE_MULTIPLE);
                frontRightDriveMotor.setPower(FAST_DRIVE_MULTIPLE);
            }

            frontLeftDriveMotor.setPower(0.0);
            frontRightDriveMotor.setPower(0.0);
            backLeftDriveMotor.setPower(0.0);
            backRightDriveMotor.setPower(0.0);
            telemetry.addData("Front/Back Power", drive);
            telemetry.addData("Side to side Power", side);

            telemetry.update();
        }
    }
}