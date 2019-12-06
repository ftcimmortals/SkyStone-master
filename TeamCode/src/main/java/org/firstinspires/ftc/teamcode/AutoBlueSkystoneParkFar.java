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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name="B-SS-Far", group ="Concept")
//@Disabled
public class AutoBlueSkystoneParkFar extends LinearOpMode {

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
        frontLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        armRotateMotor.setDirection(DcMotor.Direction.REVERSE);

        capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);

        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        //Set the position of the bridge support targets with relation to origin (center of field)

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        clawFingersServo.setPosition(FINGERS_OPEN);

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        waitForStart();
        runtime.reset();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        if(opModeIsActive()) {
            double voltage = this.hardwareMap.voltageSensor.iterator().next().getVoltage();
            double timeMultiple = (-0.1*voltage) + 2.25;
            double forwardPowerSlow = 0.5;
            double sidePowerSlow = 0.5;
            double forwardPowerFast = 0.9;
            double sidePowerFast = 0.6;
            double turnPower = 0.5;

            // check all the trackable targets to see which one (if any) is visible.
            moveForwardInches(forwardPowerSlow, false, 16);
            targetVisible = false;

            int ticsPerDegree = (int) ((1425.2 *24)/360);
            int degrees = 132;

            armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armRotateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armRotateMotor.setTargetPosition(ticsPerDegree * degrees);
            armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRotateMotor.setPower(1);

            for(int ii=0; ii<2; ii++) {
                VuforiaTrackable trackable = allTrackables.get(0);
                sleep(500);
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                }
            }
            telemetry.addData("Visible or not? ", targetVisible);

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                //First stone
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                double Ypos = translation.get(1)/mmPerInch;
                telemetry.addData("Ypos: ",absolute(Ypos));
                telemetry.update();
                moveForwardInches(forwardPowerSlow, false, 17);
                stoneServo.setPosition(STONE_PICKER_OPEN);
                sleep(1000);
                moveForwardInches(forwardPowerSlow, true, 7);
                moveTurnDegrees(turnPower, false, 90);
                moveForwardInches(forwardPowerFast, false, 60);
                stoneServo.setPosition(STONE_PICKER_CLOSED);
                sleep(500);
                moveForwardInches(forwardPowerFast, true, 60);
                moveTurnDegrees(turnPower, true, 90);
                moveSideInches(sidePowerFast, false, 23);
                moveForwardInches(0.4, false, 11);
                stoneServo.setPosition(STONE_PICKER_OPEN);
                sleep(1000);
                moveForwardInches(forwardPowerSlow, true, 11);
                moveSideInches(sidePowerFast, true, 23);
                moveTurnDegrees(turnPower, false, 90);
                moveForwardInches(forwardPowerFast, false, 60);
                stoneServo.setPosition(STONE_PICKER_CLOSED);
                sleep(500);
                moveForwardInches(forwardPowerFast, true, 28);
            }else{
                //Second stone
                moveSideInches(sidePowerSlow, false, 8);
                targetVisible = false;
                for(int ii=0; ii<2; ii++) {
                    VuforiaTrackable trackable = allTrackables.get(0);
                    sleep(500);
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;
                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                    }
                }
                if (targetVisible){
                    moveForwardInches(forwardPowerSlow, false, 17);
                    stoneServo.setPosition(STONE_PICKER_OPEN);
                    sleep(1000);
                    moveForwardInches(forwardPowerSlow, true, 7);
                    moveTurnDegrees(turnPower, false, 90);
                    moveForwardInches(forwardPowerFast, false, 68);
                    stoneServo.setPosition(STONE_PICKER_CLOSED);
                    sleep(500);
                    moveForwardInches(forwardPowerFast, true, 68);
                    moveTurnDegrees(turnPower, true, 90);
                    moveSideInches(sidePowerFast, false, 23);
                    moveForwardInches(0.4, false, 12);
                    stoneServo.setPosition(STONE_PICKER_OPEN);
                    sleep(1000);
                    moveForwardInches(forwardPowerSlow, true, 11);
                    moveSideInches(sidePowerFast, true, 23);
                    moveTurnDegrees(turnPower, false, 90);
                    moveForwardInches(forwardPowerFast, false, 68);
                    stoneServo.setPosition(STONE_PICKER_CLOSED);
                    sleep(500);
                    moveForwardInches(forwardPowerFast, true, 28);
                }else{
                    //Third stone
                    moveSideInches(sidePowerSlow, false, 8);
                    moveForwardInches(0.4, false, 17);
                    stoneServo.setPosition(STONE_PICKER_OPEN);
                    sleep(1000);
                    moveForwardInches(forwardPowerSlow, true, 7);
                    moveTurnDegrees(turnPower, false, 90);
                    moveForwardInches(forwardPowerFast, false, 73);
                    stoneServo.setPosition(STONE_PICKER_CLOSED);
                    sleep(500);
                    moveForwardInches(forwardPowerFast, true, 54);
                    moveTurnDegrees(turnPower, true, 90);
                    moveForwardInches(0.4, false, 11);
                    stoneServo.setPosition(STONE_PICKER_OPEN);
                    sleep(1000);
                    moveForwardInches(forwardPowerSlow, true, 11);
                    moveTurnDegrees(turnPower, false, 90);
                    moveForwardInches(forwardPowerFast, false, 54);
                    stoneServo.setPosition(STONE_PICKER_CLOSED);
                    sleep(500);
                    moveForwardInches(forwardPowerFast, true, 24);
                }
            }

            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
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
        double poweruse;
        int starttics = frontLeftDriveMotor.getCurrentPosition();

        if (direction) {
            FLtarget = (int)(ticsPerInch * inches + frontLeftDriveMotor.getCurrentPosition());
            FRtarget =(int)(ticsPerInch *  inches + frontRightDriveMotor.getCurrentPosition());
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

        } else {
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

        }
        while ((absolute(frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol ) && (absolute(frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())) {
            poweruse = wheelPower + (((wheelPower - 0.5)/(starttics-FLtarget))* ((frontLeftDriveMotor.getCurrentPosition()-starttics)));

            frontLeftDriveMotor.setPower(poweruse);
            frontRightDriveMotor.setPower(poweruse);
            backLeftDriveMotor.setPower(poweruse);
            backRightDriveMotor.setPower(poweruse);
            sleep(25);
        }
        sleep(100);

        return (0);
    }
    public double moveSideInches(double wheelPower, boolean direction, double inches) {

        // direction true => right
        // direction false => left
        double ticsPerMotor = 1120;
        double circumference = 12.125;
        double ticsPerInch = ticsPerMotor / circumference;

        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 25;
        double poweruse;
        int starttics = frontLeftDriveMotor.getCurrentPosition();

        if (direction) {
            double sideMultiple = 1.2;
            FLtarget = (int)(ticsPerInch * sideMultiple * inches + frontLeftDriveMotor.getCurrentPosition());
            FRtarget =(int)(-ticsPerInch * sideMultiple * inches + frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int)(-ticsPerInch * sideMultiple * inches + backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int)(ticsPerInch * sideMultiple * inches + backRightDriveMotor.getCurrentPosition());

            frontLeftDriveMotor.setTargetPosition(FLtarget);
            frontRightDriveMotor.setTargetPosition(FRtarget);
            backLeftDriveMotor.setTargetPosition(BLtarget);
            backRightDriveMotor.setTargetPosition(BRtarget);

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            double sideMultiple = 1.2;

            FLtarget = (int)(-ticsPerInch * sideMultiple * inches + frontLeftDriveMotor.getCurrentPosition());
            FRtarget =(int)(ticsPerInch * sideMultiple * inches + frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int)(ticsPerInch * sideMultiple * inches + backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int)(-ticsPerInch * sideMultiple * inches + backRightDriveMotor.getCurrentPosition());

            frontLeftDriveMotor.setTargetPosition(FLtarget);
            frontRightDriveMotor.setTargetPosition(FRtarget);
            backLeftDriveMotor.setTargetPosition(BLtarget);
            backRightDriveMotor.setTargetPosition(BRtarget);

            frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        while ((absolute(frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol ) && (absolute(frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())) {
            poweruse = wheelPower + (((wheelPower - 0.5)/(starttics-FLtarget))* ((frontLeftDriveMotor.getCurrentPosition()-starttics)));

            frontLeftDriveMotor.setPower(poweruse);
            frontRightDriveMotor.setPower(poweruse);
            backLeftDriveMotor.setPower(poweruse);
            backRightDriveMotor.setPower(poweruse);
            sleep(25);
        }
        sleep(200);

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

        } else {
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

        }
        while ((absolute(frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol ) && (absolute(frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())){
            frontLeftDriveMotor.setPower(wheelPower);
            frontRightDriveMotor.setPower(wheelPower);
            backLeftDriveMotor.setPower(wheelPower);
            backRightDriveMotor.setPower(wheelPower);
            sleep(25);
        }
        sleep(100);

        return (0);
    }
}