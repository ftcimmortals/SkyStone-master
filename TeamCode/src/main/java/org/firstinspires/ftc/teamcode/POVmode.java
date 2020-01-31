package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/*This opmode is for driver controlled period*/

@TeleOp(name="POV_Mode", group="Linear Opmode")
// @Disabled
public class POVmode extends CommonMethods {

    // define IMU
    private BNO055IMU imu;

    //runtime
    private ElapsedTime runtime = new ElapsedTime();

    // Variables for toggling
    boolean currentState = false;
    boolean lastState = false;
    boolean currentRun = false;
    boolean currentRun2 = false;
    boolean lastRun = false;
    boolean lastRun2 = false;
    boolean runProgram = false;
    boolean currentDrive = false;
    boolean lastDrive = false;
    boolean currentMode = false;
    boolean lastMode = false;

    //variables for levels
    int currentPosition = 0;
    int targetPosition = 0;
    double numberToClear;
    int armExtendPosition;

    //speed variables
    double driveMultipleSide0 = MID_DRIVE_MULTIPLE_SIDE;
    double driveMultipleForward0 = MID_DRIVE_MULTIPLE_FORWARD;
    double driveMultipleSide = MID_DRIVE_MULTIPLE_SIDE;
    double driveMultipleForward = MID_DRIVE_MULTIPLE_FORWARD;



    @Override
    public void runOpMode() {

        Hardware hardware = new Hardware(hardwareMap); //For hardware variables

        //Directions for all motors
        hardware.frontLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.frontRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.backLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        hardware.backRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.armRotateMotor.setDirection(DcMotor.Direction.REVERSE);
        hardware.tapeMotor.setDirection(DcMotor.Direction.REVERSE);

        //set servos for init
        hardware.clawFingersServo.setPosition(FINGERS_OPEN);
        hardware.clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);
        hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_UP);
        hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_UP);
        hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_UP);
        hardware.capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);
        hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_UP);
        hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_UP);
        hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
        hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
        hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_UP);
        hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_UP);
        hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
        hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);

        //set mode for encoders and touch sensors
        hardware.armLimitTouchFront.setMode(DigitalChannel.Mode.INPUT);
        hardware.armLimitTouchBack.setMode(DigitalChannel.Mode.INPUT);
        hardware.armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.armRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set start position for extend motor
        int extendMotorStartPos = hardware.armExtendMotor.getCurrentPosition();

        //set motors to break when stopped
        hardware.frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.armRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set strings for telemetry usage
        String speedState = "MEDIUM";
        String armReset = "NO";
        String mode = "TAPE";

        //set last positions for servos
        double lastPosLeft = DELIVERY_SERVO_IN_LEFT;
        double lastPosRight = DELIVERY_SERVO_IN_RIGHT;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //set factor for robot drift
            double MultFactor = 1;

            // DRIVER 1 : Gamepad 1 controls

            //set controls for the gamepads
            double turn = gamepad1.right_stick_x;
            double mY = gamepad1.left_stick_y;
            double mX = gamepad1.left_stick_x;

            //when y is pressed set current drive
            currentDrive = gamepad1.y;

            //if y is let go
            if((!currentDrive) && (lastDrive)) {
                //and it is slow
                if(speedState == "SLOW") {
                    //set to medium speed
                    driveMultipleSide0 = MID_DRIVE_MULTIPLE_SIDE;
                    driveMultipleForward0 = MID_DRIVE_MULTIPLE_FORWARD;
                    speedState = "MEDIUM";
                }
                //if it is medium
                else if (speedState == "MEDIUM") {
                    //set to fast speed
                    driveMultipleSide0 = FAST_DRIVE_MULTIPLE_SIDE;
                    driveMultipleForward0 = FAST_DRIVE_MULTIPLE_FORWARD;
                    speedState = "FAST";
                }
                //if it is fast speed
                else if (speedState == "FAST"){
                    //set to slow speed
                    driveMultipleSide0 = SLOW_DRIVE_MULTIPLE_SIDE;
                    driveMultipleForward0 = SLOW_DRIVE_MULTIPLE_FORWARD;
                    speedState = "SLOW";
                }
            }
            //if it is fast
            if (speedState == "FAST") {
                //and trigger is pressed
                if (gamepad1.right_trigger > 0) {
                    //set to running slower
                    driveMultipleSide = driveMultipleSide0 * 0.3;
                    driveMultipleForward = driveMultipleForward0 * 0.3;
                }else{
                    //set to normal
                    driveMultipleSide = driveMultipleSide0;
                    driveMultipleForward = driveMultipleForward0;
                }
                //if it is slow or medium
            } else {
                //and trigger is pressed
                if (gamepad1.right_trigger > 0) {
                    //set to running slower
                    driveMultipleSide = driveMultipleSide0 * 0.5;
                    driveMultipleForward = driveMultipleForward0 * 0.5;
                }else{
                    //set to normal
                    driveMultipleSide = driveMultipleSide0;
                    driveMultipleForward = driveMultipleForward0;
                }
            }

            //set mode for drive motor encoders to run with power set
            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //drive powers are the gamepad left stick y values times the drive multiple
            double powerflD = (mY * -driveMultipleForward);
            double powerfrD = (mY * -driveMultipleForward);
            double powerblD = (mY * -driveMultipleForward);
            double powerbrD = (mY * -driveMultipleForward);

            //side powers are the gamepad left stick x values times the side drive multiple
            double powerflS = (mX * driveMultipleSide);
            double powerfrS = (mX * -driveMultipleSide);
            double powerblS = ((mX * -driveMultipleSide) * MultFactor); //use multFactor for drift
            double powerbrS = ((mX * driveMultipleSide) * MultFactor); //use multFactor for drift

            //add side and drive powers for diagonal possibility
            double powerfl = powerflS + powerflD;
            double powerfr = powerfrS + powerfrD;
            double powerbl = powerblS + powerblD;
            double powerbr = powerbrS + powerbrD;

            //set powers
            hardware.frontLeftDriveMotor.setPower(powerfl);
            hardware.frontRightDriveMotor.setPower(powerfr);
            hardware.backLeftDriveMotor.setPower(powerbl);
            hardware.backRightDriveMotor.setPower(powerbr);

            //if it is turning set power to right stick y values
            if (turn != 0) {
                hardware.frontLeftDriveMotor.setPower(-turn * driveMultipleSide);
                hardware.backRightDriveMotor.setPower(turn * driveMultipleSide);
                hardware.backLeftDriveMotor.setPower(-turn * driveMultipleSide);
                hardware.frontRightDriveMotor.setPower(turn * driveMultipleSide);
            }

            //set foundation grabber positions
            if (gamepad1.dpad_up) {
                hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_UP);
                hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_UP);
            }
            if (gamepad1.dpad_down) {
                hardware.foundationGrabberServoLeft.setPosition(FOUNDATION_GRABBER_LEFT_DOWN);
                hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_DOWN);
            }

            //set delivery servos to open
            if (gamepad1.x) {
                lastPosLeft = DELIVERY_LEFT_OPEN_FULLY;
                lastPosRight = DELIVERY_RIGHT_OPEN_FULLY;
                hardware.deliveryServoLeft.setPosition(lastPosLeft);
                hardware.deliveryServoRight.setPosition(lastPosRight);
            }

            //set delivery servos to closed
            if (gamepad1.b) {
                lastPosLeft = DELIVERY_SERVO_IN_LEFT;
                lastPosRight = DELIVERY_SERVO_IN_RIGHT;
                hardware.deliveryServoLeft.setPosition(lastPosLeft);
                hardware.deliveryServoRight.setPosition(lastPosRight);
            }

            //if trigger is pressed and it's open, close for pickup
            if ((hardware.deliveryServoLeft.getPosition() == DELIVERY_LEFT_OPEN_FULLY) && (gamepad1.left_trigger > 0)){
                lastPosLeft = DELIVERY_SERVO_LEFT_IN_A_LITTLE;
                lastPosRight = DELIVERY_SERVO_RIGHT_IN_A_LITTLE;
                hardware.deliveryServoLeft.setPosition(lastPosLeft);
                hardware.deliveryServoRight.setPosition(lastPosRight);

             //if trigger is not pressed and it is not in, set to open
            }else if ((gamepad1.left_trigger == 0) && (!gamepad1.b) && (lastPosLeft != DELIVERY_SERVO_IN_LEFT)){
                hardware.deliveryServoLeft.setPosition(DELIVERY_LEFT_OPEN_FULLY);
                hardware.deliveryServoRight.setPosition(DELIVERY_RIGHT_OPEN_FULLY);
            }


            //run alignment ('The dance') if bumper is let go
            currentRun2 = gamepad1.left_bumper;
            if((currentRun2 == false) && (lastRun2 == true)){
                hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_RIGHT_FOR_ALIGNMENT);
                moveSideInches(0.2, false, 3, hardware);
                hardware.deliveryServoRight.setPosition(DELIVERY_RIGHT_OPEN_FULLY);
                moveForwardInches(0.2, false, 6, hardware);
                lastPosLeft = DELIVERY_SERVO_IN_LEFT;
                lastPosRight = DELIVERY_SERVO_IN_RIGHT;
                hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
                hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);
                sleep(100);
                moveSideInches(0.2, true, 6, hardware);
            }

            //close capstone servo when back is pressed
            if (gamepad1.back) {
                hardware.capstoneServo.setPosition(CAPSTONE_NOT_DROPPED);
            }
            //DRIVER 2 : Gamepad 2 controls

            //add to counter when a is pressed
            currentState = gamepad2.a;
            if ((!currentState) && (lastState)) {
                targetPosition++;
            }
            //if x is pressed, reset the level
            if (gamepad2.x) {
                targetPosition = 0;
            }
            //go to level if y button is pressed
            if (gamepad2.y) {
                runProgram = true;
                hardware.deliveryServoRight.setPosition(DELIVERY_SERVO_IN_RIGHT);
                hardware.deliveryServoLeft.setPosition(DELIVERY_SERVO_IN_LEFT);
            }

            double height;
            double heightsq;
            double basesq;
            double hypotenuse;
            double angleInRadians;
            double angleInDegrees;
            double ticsToMove;
            double ticsPerBlock;

            if (targetPosition == 0) {
                //setting level for zero
                numberToClear = 7.5; //number to pass block
                height = (targetPosition * 4) + numberToClear; //height of triangle
                heightsq = Math.pow(height, 2); //square for pythagorean
                basesq = Math.pow(BASE + 1.5, 2); //square base of triangle for pythagorean
                hypotenuse = Math.sqrt(heightsq + basesq); //square root of the values squared to find the hypotenuse
                angleInRadians = Math.asin(height / hypotenuse);//find the angle by using inverse sin
                angleInDegrees = angleInRadians * (180 / Math.PI);//convert to degrees
                ticsToMove = angleInDegrees * TICS_PER_DEGREE;//convert to tics using tics per degree
                currentPosition = 0;//set current level to zero
            } else {
                //setting level for the rest
                numberToClear = 3;//number to pass block
                height = (targetPosition * 4) + numberToClear;//height of triangle
                heightsq = Math.pow(height, 2);//square for pythagorean
                basesq = Math.pow(BASE, 2);//square base of triangle for pythagorean
                hypotenuse = Math.sqrt(heightsq + basesq);//square root of the values squared to find the hypotenuse
                angleInRadians = Math.asin(height / hypotenuse);//find the angle by using the inverse sin
                angleInDegrees = angleInRadians * (180 / Math.PI);//convert to degrees
                ticsToMove = angleInDegrees * TICS_PER_DEGREE;//convert to tics using tics per degree
                ticsPerBlock = (ticsToMove / 4) - numberToClear;//calculate tics per block
                currentPosition = (int) (hardware.armRotateMotor.getCurrentPosition() / ticsPerBlock);//set current level
            }

            //if y is pressed
            if (runProgram) {
                //if touch sensor is pressed
                if (hardware.armLimitTouchFront.getState() == false) {
                    //set position to the current position
                    hardware.armRotateMotor.setTargetPosition(hardware.armRotateMotor.getCurrentPosition());
                    hardware.armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hardware.armRotateMotor.setPower(1.0);
                }


                //calculating current position
                if ((hardware.armRotateMotor.getCurrentPosition()) <= ((ticsToMove - ANGLE_START) - 50)) {
                    // move arm up/down because arm is already contracted
                    if (hardware.armExtendMotor.getCurrentPosition() <= (extendMotorStartPos + 50)) {
                        // arm is contracted, move arm
                        hardware.armRotateMotor.setTargetPosition((int)(ticsToMove - ANGLE_START));
                        hardware.armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardware.armRotateMotor.setPower(1.0);
                    } else {
                        // contract arm before moving arm
                        hardware.armExtendMotor.setTargetPosition(extendMotorStartPos);
                        hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardware.armExtendMotor.setPower(0.3);
                    }
                } else if ((hardware.armRotateMotor.getCurrentPosition()) >= ((ticsToMove - ANGLE_START) + 50)) {
                    // move arm up/down because arm is already contracted
                    if (hardware.armExtendMotor.getCurrentPosition() <= (extendMotorStartPos + 50)) {
                        // arm is contracted, move arm
                        hardware.armRotateMotor.setTargetPosition((int)(ticsToMove- ANGLE_START));
                        hardware.armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardware.armRotateMotor.setPower(1.0);
                    } else {
                        // contract arm before moving arm
                        hardware.armExtendMotor.setTargetPosition(extendMotorStartPos);
                        hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardware.armExtendMotor.setPower(0.3);
                    }
                } else {
                    // extend arm
                    double factorExtend;
                    //calculate factors to add to extention because of spring tension
                    if ((targetPosition > 0) && (targetPosition < 4)){
                        factorExtend = 1.1;
                    }
                    else if (targetPosition == 4){
                        factorExtend = 1.15;
                    }
                    else if (targetPosition == 5){
                        factorExtend = 1.22;
                    }
                    else if (targetPosition == 6){
                        factorExtend = 1.4;
                    }
                    else if (targetPosition == 7){
                        factorExtend = 1.32;
                    }
                    else if (targetPosition == 8){
                        factorExtend = 1.35;
                    }
                    else{
                        factorExtend = 1;
                    }
                    double hypotenuseWithFactor = (hypotenuse * factorExtend);//multiply hypotenuse by the factor
                    double deltaH = hypotenuseWithFactor - ARM_LENGTH;//subtract arm length from hypotenuse to find how much to extend
                    double rotationsOfArmExtension = deltaH / SPINDLE_CIRCUMFERENCE;//how many rotations of the spindle is needed to extend arm that much
                    int armExPos = 0;//define arm extend position

                    armExPos = (int) ((rotationsOfArmExtension * TICS_PER_ROTATION_EXTENTION_MOTOR));//set tics needed to move
                    //go to calculated position
                    hardware.armExtendMotor.setTargetPosition(armExPos);
                    hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hardware.armExtendMotor.setPower(0.3);

                    //stop running program if reached needed position
                    if (absolute(hardware.armExtendMotor.getCurrentPosition()) >= absolute(armExPos) - 50) {
                        runProgram = false;
                    }
                }
                //if run program is false
            } else {
                //set controls for the gamepad
                double armRotate = gamepad2.right_stick_y;
                double armExtend = gamepad2.left_stick_y;


                if ((armRotate > 0) && (hardware.armLimitTouchBack.getState() == true)) {      // right stick y to rotate arm up
                    hardware.armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hardware.armRotateMotor.setPower(armRotate);
                } else if ((armRotate < 0) && (hardware.armLimitTouchFront.getState() == true)) {    // right stick y to rotate arm down
                    hardware.armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hardware.armRotateMotor.setPower(armRotate);
                } else {
                    hardware.armRotateMotor.setPower(0.0);
                }

                currentMode = gamepad2.b;//when b is pressed once, change the mode
                if((currentMode == false) && (lastMode == true)){
                    if(mode == "ARM"){
                        mode = "TAPE";
                    } else if (mode == "TAPE"){
                        mode = "ARM";
                    }
                }
                if (mode == "ARM") {
                    if (armExtend > 0) {                                // left stick y to extend arm
                        hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hardware.armExtendMotor.setPower(-armExtend * ARM_EXTEND_MULTIPLE);
                    } else if ((armExtend < 0) && (!gamepad2.b)) {                           // left stick y to collapse arm
                        hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hardware.armExtendMotor.setPower(-armExtend * ARM_EXTEND_MULTIPLE);
                    } else {
                        // set power to the motor to keep it extended
                        armExtendPosition = hardware.armExtendMotor.getCurrentPosition();
                        hardware.armExtendMotor.setTargetPosition(armExtendPosition);
                        hardware.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                } else if (mode == "TAPE"){
                    if(gamepad2.left_stick_y != 0){ //if the left stick is moved
                        hardware.foundationGrabberServoRight.setPosition(FOUNDATION_GRABBER_RIGHT_HALF);//move foundation grabber out of the way
                        hardware.tapeMotor.setPower(gamepad2.left_stick_y);//set power to the stick value
                    }else{
                        hardware.tapeMotor.setPower(0);//stop motor
                    }
                }
            }


            if (gamepad2.right_bumper) {                        // right bumper to turn wrist right 90 degrees
                hardware.clawWristServo.setPosition(WRIST_TURN_HORIZONTAL);
            }
            if (gamepad2.left_bumper) {                         // left bumper to turn wrist left 90 degrees
                hardware.clawWristServo.setPosition(WRIST_TURN_VERTICAL);
            }
            if (gamepad2.right_trigger > 0) {                    // right trigger to fine tune turn wrist right
                hardware.clawWristServo.setPosition(hardware.clawWristServo.getPosition() + (gamepad2.right_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad2.left_trigger > 0) {                   // left trigger to fine tune turn wrist left
                hardware.clawWristServo.setPosition(hardware.clawWristServo.getPosition() - (gamepad2.left_trigger * WRIST_TURN_MULTIPLE));
            }
            if (gamepad2.dpad_up) {                                   // y button on gamepad2 to close claw
                hardware.clawFingersServo.setPosition(FINGERS_CLOSED);
            }
            if (gamepad2.dpad_down) {                                   // a button on gamepad2 to open claw
                hardware.clawFingersServo.setPosition(FINGERS_OPEN);
            }
            if (gamepad2.back) {                                        //drop capstone if back is pressed
                hardware.capstoneServo.setPosition(CAPSTONE_DROPPED);
            }
            if(hardware.armLimitTouchBack.getState() == false){ //if touch sensor is pressed, change strings for telemetry
                armReset = "YES";
            }else{
                armReset = "NO";
            }

            //set telemetry
            telemetry.addData("Target Position: ", targetPosition);
            telemetry.addData("Current Speed: ", speedState);
            telemetry.addData("Is the arm reset? ", armReset);
            telemetry.addData("Driver B Mode: ", mode);

            telemetry.update();
            //update boolean variables for one press buttons
            lastState = currentState;
            lastDrive = currentDrive;
            lastRun = currentRun;
            lastRun2 = currentRun2;
            lastMode = currentMode;

        }
    }
}