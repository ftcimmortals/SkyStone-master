package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;


/*
This blue autonomous mode picks up and delivers two skystones and one stone, and parks for a total of 27 points
 */
@Autonomous(name= "Blue-SS-Deliver", group="Sky autonomous")
//@Disabled
public class AutoBlueSkystoneDelivery extends CommonMethods {
    private ElapsedTime runtime = new ElapsedTime();

    //init the OpenCV vision
    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    //webcam
    OpenCvCamera webcam;
    //variable for block case
    String blockCase;

    //set angles for imu
    Orientation angles;
    Acceleration gravity;

    //variable for distance from block
    double blockDistance;

    @Override
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap);//init hardware

        //get camera view
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //define webcam
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);

        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
        hardware.armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        hardware.armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);

        //set motor direction
        hardware.frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.armRotateMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.tapeMotor.setDirection(DcMotor.Direction.FORWARD);

        //set servo position at init
        hardware.capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);
        hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
        hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
        hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
        hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
        hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_UP);
        hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_UP);
        hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
        hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);
        hardware.clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);
        hardware.clawFingersServo.setPosition(FINGERS_OPEN);

        //init imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        hardware.imu.initialize(parameters);
        //wait till start
        waitForStart();
        runtime.reset();

        //get values for angles and gravity with imu
        angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = hardware.imu.getGravity();

        // Start the logging of measured acceleration
        hardware.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        double ticsPerInch = (TICS_PER_ROTATION_TAPE / TAPE_WHEEL_CIRCUMFERENCE);//tics per inch moved
        double startPos = hardware.frontRightDriveMotor.getCurrentPosition();//start position of the otor

        if (opModeIsActive()) {
            double angleStart = getAngle(hardware); //get reference angle

            //look for block and find case
            for(int ii = 0; ii < 3; ii++) {
                if (valLeft < 10) {
                    blockCase = "LEFT";
                } else if (valRight < 10) {
                    blockCase = "RIGHT";
                } else if (valMid < 10) {
                    blockCase = "CENTER";
                }
                telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
                telemetry.addData("Case", blockCase);
                telemetry.addData("Height", rows);
                telemetry.addData("Width", cols);

                telemetry.update();
                sleep(100);
            }
            //first case
            if (blockCase == "LEFT"){
                //align with first stone and pick it up
                PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 3.5, angleStart, hardware);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 22, angleStart, hardware);
                blockDistance = hardware.sensorRight.getDistance(DistanceUnit.INCH);
                moveForwardInches(0.2, false, blockDistance, hardware);
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_DOWN_FULLY);
                sleep(500);
                //deliver it
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 9, angleStart, hardware);
                moveTurnDegrees(0.3, -1, 90, hardware);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_DOWN);
                sleep(500);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 49, angleStart + 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                //align with second stone and pick it up
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, 1, 65, angleStart + 90, hardware);
                moveTurnDegrees(0.3, 1, 90, hardware);
                blockDistance = hardware.sensorLeft.getDistance(DistanceUnit.INCH);
                moveForwardInches(0.2, false, blockDistance, hardware);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN_FULLY);
                sleep(500);
                //deliver it
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 9, angleStart, hardware);
                moveTurnDegrees(0.3, -1, 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN);
                sleep(500);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 66, angleStart + 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                //align with third stone and pick it up
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, 1, 75, angleStart + 90, hardware);
                moveTurnDegrees(0.3, 1, 90, hardware);
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.2, -1, 11, angleStart, hardware);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN_FULLY);
                sleep(500);
                //deliver it
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 11, angleStart, hardware);
                moveTurnDegrees(0.3, -1, 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN);
                //move right foundation servo and open the tape
                hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_HALF);//move foundation grabber out of the way
                sleep(500);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 75, angleStart + 90, hardware);
                int tapeticstomove = (int) (30 * ticsPerInch);
                hardware.tapeMotor.setTargetPosition(tapeticstomove + hardware.tapeMotor.getCurrentPosition());
                hardware.tapeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.tapeMotor.setPower(1);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                sleep(1000);
            //second case
            } else if (blockCase == "CENTER"){
                //align with first stone and pick it up
                PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 3.5, angleStart, hardware);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 22, angleStart, hardware);
                blockDistance = hardware.sensorLeft.getDistance(DistanceUnit.INCH);
                moveForwardInches(0.2, false, blockDistance, hardware);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN_FULLY);
                sleep(500);
                //deliver it
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 9, angleStart, hardware);
                moveTurnDegrees(0.3, -1, 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN);
                sleep(500);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 50, angleStart + 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                //align with second stone and pick it up
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, 1, 74, angleStart + 90, hardware);
                moveTurnDegrees(0.3, 1, 90, hardware);
                blockDistance = hardware.sensorLeft.getDistance(DistanceUnit.INCH);
                moveForwardInches(0.2, false, blockDistance, hardware);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN_FULLY);
                sleep(500);
                //deliver it
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 9, angleStart, hardware);
                moveTurnDegrees(0.3, -1, 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN);
                sleep(500);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 74, angleStart + 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                //align with third stone and pick it up
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, 1, 56, angleStart + 90, hardware);
                moveTurnDegrees(0.3, 1, 90, hardware);
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.15, -1, 11, angleStart, hardware);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN_FULLY);
                sleep(500);
                //deliver it
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 11, angleStart, hardware);
                moveTurnDegrees(0.3, -1, 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN);
                //move right foundation servo and open the tape
                hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_HALF);//move foundation grabber out of the way
                sleep(500);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 55, angleStart + 90, hardware);
                int tapeticstomove = (int) (30 * ticsPerInch);
                hardware.tapeMotor.setTargetPosition(tapeticstomove + hardware.tapeMotor.getCurrentPosition());
                hardware.tapeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.tapeMotor.setPower(1);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                sleep(1000);
            //Third case
            } else {
                //align with first stone and pick it up
                PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.3, -1, 4.5, angleStart, hardware);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 22, angleStart, hardware);
                blockDistance = hardware.sensorLeft.getDistance(DistanceUnit.INCH);
                moveForwardInches(0.2, false, blockDistance, hardware);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN_FULLY);
                sleep(500);
                //deliver it
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 9, angleStart, hardware);
                moveTurnDegrees(0.3, -1, 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN);
                sleep(500);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 58, angleStart + 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                //align with second stone and pick it up
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, 1, 74, angleStart + 90, hardware);
                moveTurnDegrees(0.3, 1, 90, hardware);
                PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.3, -1, 11, angleStart, hardware);
                blockDistance = hardware.sensorLeft.getDistance(DistanceUnit.INCH);
                moveForwardInches(0.2, false, blockDistance, hardware);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN_FULLY);
                sleep(500);
                //deliver it
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 9, angleStart, hardware);
                PIDsideInches(GAIN_P, GAIN_I, GAIN_D, 0.3, 1,11, angleStart, hardware);
                moveTurnDegrees(0.3, -1, 90, hardware);
                sleep(500);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 67, angleStart + 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                //align with third stone and pick it up
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, 1, 36, angleStart + 90, hardware);
                moveTurnDegrees(0.3, 1, 90, hardware);
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.2, -1, 11, angleStart, hardware);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN_FULLY);
                sleep(500);
                //deliver it
                PIDstraightInchesNoRamp(GAIN_P, GAIN_I, GAIN_D, 0.3, 1, 11, angleStart, hardware);
                moveTurnDegrees(0.3, -1, 90, hardware);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN_FULLY);
                //move right foundation servo and open the tape
                hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_HALF);//move foundation grabber out of the way
                sleep(500);
                PIDstraightInches(GAIN_P, GAIN_I, GAIN_D, 0.6, -1, 38, angleStart + 90, hardware);
                int tapeticstomove = (int) (30 * ticsPerInch);
                hardware.tapeMotor.setTargetPosition(tapeticstomove + hardware.tapeMotor.getCurrentPosition());
                hardware.tapeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.tapeMotor.setPower(1);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                sleep(500);
            }
        }
    }

//Functions for OpenCV below
    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}