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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="TestEncoders", group ="Concept")
@Disabled
public class TestEncoders extends LinearOpMode {

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQGs+8X/////AAABmUlgVbItAkXMhUiMsKHBxsOOsOSky4xQM7QN/1ugM+DkgNZQYexbfsQkK4+aDQexx9sWXZr+TPwDLQ8aXvJ3cru61Y/17wBrCRs2hGeLOENx0hRyY+sTnH2PJSXN+qaKSggoE67PpO33KHKdUD48x9T/dzeg9Rtg2PVEQBezKKa1SMq5AJGXTLI2YnjsXPJ/Uk+9TNcXfaCqxWAgFXaT9bLsyaXRLdaudyEq+qG6d73EOsV9RI2LY/RJGFPhL34Cs8WoRLtuXl8uo/mfvaLsaZrj0w6mxF+9hiYPEXrKwCXuFxc0pSXomDaWTE+NyetotMlsYNRJOccVhtUerhZ13nXfxUk7mECSNod/YBYiVrSp";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    final private double FINGERS_OPEN = 0.05;               // open claw
    final private double FINGERS_CLOSED = 0.5;              //close claw

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
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
        webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
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

        capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);

        double voltage = this.hardwareMap.voltageSensor.iterator().next().getVoltage();
        double timeMultiple = (-0.1*voltage) + 2.25;

        if(opModeIsActive()) {
            moveForwardTime(0.8, true, 5);
            sleep(5000);
            moveForwardTime(0.8, false,5);
            sleep(5000);
        }

    }

    public double moveForwardInches(double wheelPower, boolean direction, double inches) {

        // direction true => forward
        // direction false => backward
        int ticsPerMotor = 1440;
        double circumference = 12.125;
        double ticsPerInch = ticsPerMotor / circumference;

        if (direction) {
            frontLeftDriveMotor.setTargetPosition((int)(ticsPerInch * inches + frontLeftDriveMotor.getCurrentPosition()));
            frontRightDriveMotor.setTargetPosition((int)(ticsPerInch * inches + frontRightDriveMotor.getCurrentPosition()));
            backLeftDriveMotor.setTargetPosition((int)(ticsPerInch * inches + backLeftDriveMotor.getCurrentPosition()));
            backRightDriveMotor.setTargetPosition((int)(ticsPerInch * inches + backRightDriveMotor.getCurrentPosition()));

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
            frontLeftDriveMotor.setTargetPosition((int)(-ticsPerInch * inches + frontLeftDriveMotor.getCurrentPosition()));
            frontRightDriveMotor.setTargetPosition((int)(-ticsPerInch * inches + frontRightDriveMotor.getCurrentPosition()));
            backLeftDriveMotor.setTargetPosition((int)(-ticsPerInch * inches + backLeftDriveMotor.getCurrentPosition()));
            backRightDriveMotor.setTargetPosition((int)(-ticsPerInch * inches + backRightDriveMotor.getCurrentPosition()));

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeftDriveMotor.setPower(wheelPower);
            frontRightDriveMotor.setPower(wheelPower);
            backLeftDriveMotor.setPower(wheelPower);
            backRightDriveMotor.setPower(wheelPower);
        }
        return (0);
    }
    public double moveForwardTime(double wheelPower, boolean direction, double movetime) {
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // direction true => forward
        // direction false => backward
        double timeNow = getRuntime();
        // use a factor for left wheel because the robot gets pulled to the left due to weight balance
        frontRightDriveMotor.setPower(0);
        frontLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);

        if (direction) {
            timeNow = getRuntime();
            while ((getRuntime() < timeNow + movetime) && (opModeIsActive())) {

                frontLeftDriveMotor.setPower(wheelPower);
                frontRightDriveMotor.setPower(wheelPower);
                backLeftDriveMotor.setPower(wheelPower);
                backRightDriveMotor.setPower(wheelPower);
            }

            frontLeftDriveMotor.setPower(0);
            frontRightDriveMotor.setPower(0);
            backLeftDriveMotor.setPower(0);
            backRightDriveMotor.setPower(0);
        } else {
            timeNow = getRuntime();
            while ((getRuntime() < timeNow + movetime) && (opModeIsActive())) {

                frontLeftDriveMotor.setPower(-wheelPower);
                frontRightDriveMotor.setPower(-wheelPower);
                backLeftDriveMotor.setPower(-wheelPower);
                backRightDriveMotor.setPower(-wheelPower);
            }

            frontLeftDriveMotor.setPower(0);
            frontRightDriveMotor.setPower(0);
            backLeftDriveMotor.setPower(0);
            backRightDriveMotor.setPower(0);
        }
        return (0);
    }
    public double moveSideTime(double wheelPower, boolean direction, double movetime) {
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // direction true => right
        // direction false => left
        double timeNow = getRuntime();
        // use a factor for left wheel because the robot gets pulled to the left due to weight balance
        frontRightDriveMotor.setPower(0);
        frontLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);

        if (direction) {
            timeNow = getRuntime();
            while ((getRuntime() < timeNow + movetime) && (opModeIsActive())) {

                frontLeftDriveMotor.setPower(wheelPower);
                frontRightDriveMotor.setPower(-wheelPower);
                backLeftDriveMotor.setPower(-wheelPower);
                backRightDriveMotor.setPower(wheelPower);
            }

            frontLeftDriveMotor.setPower(0);
            frontRightDriveMotor.setPower(0);
            backLeftDriveMotor.setPower(0);
            backRightDriveMotor.setPower(0);
        } else {
            timeNow = getRuntime();
            while ((getRuntime() < timeNow + movetime) && (opModeIsActive())) {

                frontLeftDriveMotor.setPower(-wheelPower);
                frontRightDriveMotor.setPower(wheelPower);
                backLeftDriveMotor.setPower(wheelPower);
                backRightDriveMotor.setPower(-wheelPower);
            }

            frontLeftDriveMotor.setPower(0);
            frontRightDriveMotor.setPower(0);
            backLeftDriveMotor.setPower(0);
            backRightDriveMotor.setPower(0);
        }
        return (0);
    }
    public double absolute(double inputval) {
        if (inputval > 0) {
            return inputval;
        } else {
            return -1 * inputval;
        }
    }
    public double turnTime(double wheelPower, boolean direction, double turntime) {
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // direction true => right
        // direction false => left
        double timeNow = getRuntime();
        // use a factor for left wheel because the robot gets pulled to the left due to weight balance
        frontRightDriveMotor.setPower(0);
        frontLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);

        if (direction) {
            timeNow = getRuntime();
            while ((getRuntime() < timeNow + turntime) && (opModeIsActive())) {

                frontLeftDriveMotor.setPower(-wheelPower);
                frontRightDriveMotor.setPower(wheelPower);
                backLeftDriveMotor.setPower(-wheelPower);
                backRightDriveMotor.setPower(wheelPower);
            }

            frontLeftDriveMotor.setPower(0);
            frontRightDriveMotor.setPower(0);
            backLeftDriveMotor.setPower(0);
            backRightDriveMotor.setPower(0);
        } else {
            timeNow = getRuntime();
            while ((getRuntime() < timeNow + turntime) && (opModeIsActive())) {

                frontLeftDriveMotor.setPower(wheelPower);
                frontRightDriveMotor.setPower(-wheelPower);
                backLeftDriveMotor.setPower(wheelPower);
                backRightDriveMotor.setPower(-wheelPower);
            }

            frontLeftDriveMotor.setPower(0);
            frontRightDriveMotor.setPower(0);
            backLeftDriveMotor.setPower(0);
            backRightDriveMotor.setPower(0);
        }
        return (0);
    }
}