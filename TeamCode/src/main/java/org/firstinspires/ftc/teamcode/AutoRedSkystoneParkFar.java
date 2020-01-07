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
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * {@link AutoRedSkystoneParkFar} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "R-SS-Far", group = "Concept")
//@Disabled                            // Comment this out to add to the opmode list
public class AutoRedSkystoneParkFar extends CommonMethods {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------


    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AQGs+8X/////AAABmUlgVbItAkXMhUiMsKHBxsOOsOSky4xQM7QN/1ugM+DkgNZQYexbfsQkK4+aDQexx9sWXZr+TPwDLQ8aXvJ3cru61Y/17wBrCRs2hGeLOENx0hRyY+sTnH2PJSXN+qaKSggoE67PpO33KHKdUD48x9T/dzeg9Rtg2PVEQBezKKa1SMq5AJGXTLI2YnjsXPJ/Uk+9TNcXfaCqxWAgFXaT9bLsyaXRLdaudyEq+qG6d73EOsV9RI2LY/RJGFPhL34Cs8WoRLtuXl8uo/mfvaLsaZrj0w6mxF+9hiYPEXrKwCXuFxc0pSXomDaWTE+NyetotMlsYNRJOccVhtUerhZ13nXfxUk7mECSNod/YBYiVrSp";

    private static final float mmPerInch = 25.4f;


    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;


    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */

    // The IMU sensor object
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
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

        hardware.capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);
        hardware.stoneServoBlue.setPosition(STONE_PICKER_CLOSED_BLUE);
        hardware.stoneServoRed.setPosition(STONE_PICKER_CLOSED_RED);
        hardware.foundationGrabberServo.setPosition(FOUNDATION_GRABBER_UP);
        hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
        hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parametersV.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parametersV.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parametersV);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

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
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parametersV.cameraDirection);
        }
        hardware.clawFingersServo.setPosition(FINGERS_OPEN);
        hardware.clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        hardware.imu.initialize(parameters);
        // Wait until we're told to go
        waitForStart();
        runtime.reset();

        targetsSkyStone.activate();

        angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = hardware.imu.getGravity();

        // Start the logging of measured acceleration
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        int RB = -1;

        // Loop and update the dashboard
        if (opModeIsActive()) {
            double startAngle = getAngle(hardware);
            //0.02, 0.0009, 0.00009
            PIDstraightInches(GAIN_P,GAIN_I,GAIN_D, 0.2, -1, 18, startAngle, hardware);
            sleep(500);
            for (int ii = 0; ii < 2; ii++) {
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
                double Ypos = translation.get(1) / mmPerInch;
                telemetry.addData("Ypos: ", absolute(Ypos));
                telemetry.update();
                PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.2, 1 * RB , 3.5, startAngle, hardware);
                StonePickDeliver(RB, 1, startAngle, hardware);

            }else{
                PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.2, -1 * RB, 8, startAngle, hardware);
                for (int ii = 0; ii < 2; ii++) {
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
                if(targetVisible){
                    //Second Stone
                    PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.2, 1 * RB, 3.5, startAngle, hardware);
                    StonePickDeliver(RB, 2, startAngle, hardware);
                }else{
                    //Third Stone
                    PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.2, -1 * RB, 4.5, startAngle, hardware);
                    StonePickDeliver(RB, 3, startAngle, hardware);
                }
            }

        }

    }

}