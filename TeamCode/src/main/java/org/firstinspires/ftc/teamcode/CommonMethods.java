package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*This class has all of our functions and all of our constant variables*/

abstract public class CommonMethods extends LinearOpMode {

    // The IMU sensor object
    BNO055IMU imu;

    final public double FINGERS_OPEN = 0.00;// open claw
    final public double FINGERS_CLOSED = 0.5;//close claw
    //gains for PID
    final public double GAIN_P = 0.02;
    final public double GAIN_I = 0.00045;
    final public double GAIN_D = 0.00009;


    //this is our webcam
    WebcamName webcamName = null;

    //runtime
    public ElapsedTime runtime = new ElapsedTime();

    final public double SLOW_DRIVE_MULTIPLE_FORWARD = 0.4;//Slow multiple for driving forward
    final public double SLOW_DRIVE_MULTIPLE_SIDE = 0.6;//Slow multiple for driving sideways
    final public double MID_DRIVE_MULTIPLE_FORWARD = 0.6;//Medium multiple for driving forward
    final public double MID_DRIVE_MULTIPLE_SIDE = 0.7;//Medium multiple for driving sideways
    final public double FAST_DRIVE_MULTIPLE_FORWARD = 1;//Fast multiple for driving forward
    final public double FAST_DRIVE_MULTIPLE_SIDE = 1;//Fast multiple for driving sideways

    final public double WRIST_TURN_MULTIPLE = 0.005;//multiple for wrist turn fine tuning
    final public double WRIST_TURN_HORIZONTAL = 0.55;//wrist turn for horizontal block
    final public double WRIST_TURN_VERTICAL = 0.05;//wrist turn for vertical block


    final public double FOUNDATION_GRABBER_LEFT_DOWN = 0;//grabber left down
    final public double FOUNDATION_GRABBER_LEFT_UP = 1;//grabber left up
    final public double FOUNDATION_GRABBER_RIGHT_UP = 0;//grabber right up
    final public double FOUNDATION_GRABBER_RIGHT_DOWN = 1;//grabber right down
    final public double FOUNDATION_GRABBER_RIGHT_HALF = 0.1;//grabber right halfway for tape extention

    final public double DELIVERY_LEFT_OPEN_FULLY = 0.31;//delivery left open fully
    final public double DELIVERY_RIGHT_OPEN_FULLY = 0.65;//delivery right open fully
    final public double DELIVERY_SERVO_IN_LEFT = 0.9;//delivery left in
    final public double DELIVERY_SERVO_IN_RIGHT = 0.05;//delivery right in
    final public double DELIVERY_SERVO_LEFT_IN_A_LITTLE = 0.65;//delivery left in for pickup
    final public double DELIVERY_SERVO_RIGHT_IN_A_LITTLE = 0.35;//delivery right in for pickup
    final public double DELIVERY_SERVO_RIGHT_FOR_ALIGNMENT = 0.50;//delivery right for straightening block

    final public double WHEEL_DISTANCE = 14.5;//distance from wheels

    final public double STONE_PICKER_LEFT_DOWN = 0.8;//left big stone picker down
    final public double STONE_PICKER_LEFT_UP = 0;//right big stone picker up
    final public double STONE_PICKER_LEFT_PICKUP = 0.4;//big stone picker for pickup
    final public double STONE_PICKER_RIGHT_DOWN = 0.85;//right big stone picker down
    final public double STONE_PICKER_RIGHT_DOWN_FULLY = 1;//right big stone picker down fully
    final public double STONE_PICKER_LEFT_DOWN_FULLY = 1;//left big stone picker down fully
    final public double STONE_PICKER_RIGHT_UP = 0;//right big stone picker up
    final public double STONE_PICKER_RIGHT_PICKUP = 0.4;//big stone picker for pickup

    final public double SMALL_STONE_PICKER_LEFT_DOWN = 0.15;//small stone picker down
    final public double SMALL_STONE_PICKER_RIGHT_DOWN = 0;//small stone picker down
    final public double SMALL_STONE_PICKER_PICKUP = 0.6;//small stone picker for pickup

    final public double CAPSTONE_NOT_DROPPED = 0.95;//capstone servo in
    final public double CAPSTONE_DROPPED = 0;//capstone servo out

    final public double ARM_EXTEND_MULTIPLE = 0.5;// multiple for arm extend
    final public double ARM_LENGTH = 13;//arm length
    final public double BASE = 11.5;//base of the triangle
    final public double SPINDLE_CIRCUMFERENCE = (2.5 * (Math.PI));//circumference of the spindle
    final public double TICS_PER_ROTATION_ROTATION_MOTOR = 1425.2;//tics per rotations of the rotation motor
    final public double TICS_PER_ROTATION_EXTENTION_MOTOR = 537.6;//tics per rotations of the extention motor
    final public double TICS_PER_ROTATION_TAPE = 480; // tics per rotations of the tape motor
    final public double TICS_PER_DEGREE = (TICS_PER_ROTATION_ROTATION_MOTOR * 24) / 360;//tics per degree of rotation
    final public double ANGLE_START = (150 * TICS_PER_DEGREE);//angle of arm when it's all the way back
    final public double TAPE_WHEEL_CIRCUMFERENCE = 13;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /**********************************************Functions****************************************/

    public void PIDstraightInches(double gainP, double gainI, double gainD, double maxPower, int direction, double targetInches, double reference1, Hardware hardware) {
        //Function to move forward with PID
        /*direction + => forward
         direction - => backward*/
        double timeLast = 0; //The last time recorded
        double Ilast = 0;//The last integral value recorded
        double errorlast = 0;//The last error recorded
        double P;//Proportional
        double I;//Integral
        double D;//Derivative
        double dT;//Change in time
        double output;//Output of the controller
        double Kp = gainP;//Gain for P
        double Ki = gainI;//gain for I
        double Kd = gainD;//Gain for D
        double timeNow;//Current time
        double poL;//Left power
        double poR;//Right power
        double error;//Error
        double anglenow;//Current angle
        double initSpeedUse;//Speed to add to
        double distanceTraveled;//Distance traveled till now
        double minPowerUp = 0.3;//Minimum power for ramp up
        double minPowerDown = 0.2;//Minimum power for ramp down
        anglenow = getAngle(hardware);//Current angle
        double refTime = getRuntime();//time at start
        double elapsedTime = 0;//time passed
        long TIMESLEEP = 50;//Time to sleep for loop
        int ticsPerMotor = (1120);//Tics of the drive motors
        double circumference = 12.125;//Wheel circumference
        double ticsPerInch = (ticsPerMotor / circumference) / 2;//tics per inch moved
        double startPos = hardware.frontRightDriveMotor.getCurrentPosition();//start position of the otor

        int target = ((int)(targetInches * ticsPerInch)); //Where to go

        //go to target position for all the motors
        hardware.frontRightDriveMotor.setTargetPosition(direction *(target)+ hardware.frontRightDriveMotor.getCurrentPosition());
        hardware.frontLeftDriveMotor.setTargetPosition(direction *(target)+ hardware.frontLeftDriveMotor.getCurrentPosition());
        hardware.backRightDriveMotor.setTargetPosition(direction *(target)+ hardware.backRightDriveMotor.getCurrentPosition());
        hardware.backLeftDriveMotor.setTargetPosition(direction *(target)+ hardware.backLeftDriveMotor.getCurrentPosition());

        hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //while we are not at position
        while ((absolute((hardware.frontRightDriveMotor.getCurrentPosition() - startPos)) < (target) - 50) && opModeIsActive()) {

            sleep(TIMESLEEP);

            // calculate time
            timeNow = getRuntime();
            elapsedTime = timeNow - refTime;
            dT = timeNow - timeLast;

            // P
            anglenow = getAngle(hardware);
            error = anglenow - reference1;
            P = error;
            // I
            I = Ilast + (error * dT);
            // D
            D = (error - errorlast) / dT;

            //calculate output
            output = ((Kp * P) + (Ki * I) + (Kd * D));

            //power left to move
            poL = -output * direction;
            //power right to move
            poR = output * direction;

            //set max correction limits
            if (poR < -0.8) {
                poR = -0.8;
            }
            if (poL > 0.8) {
                poL = 0.8;
            }
            if (poL < -0.8) {
                poL = -0.8;
            }
            if (poR > 0.8) {
                poR = 0.8;
            }

            //set ramps
            double rampdistanceUp = 10;
            double rampdistanceDown = 5;
            double slopeup = (minPowerUp - maxPower)/(0 - rampdistanceUp);
            double slopedown = (maxPower - minPowerDown)/(0 - rampdistanceDown);

            //calculate distance till now
            distanceTraveled = absolute((hardware.frontRightDriveMotor.getCurrentPosition() - startPos) / ticsPerInch);
            //if the distance you want to move is enough to complete the ramp
            if (targetInches > rampdistanceUp + rampdistanceDown){
                //if it is in the first part of the trapezoid
                if(distanceTraveled < rampdistanceUp){
                    //set power to increase
                    initSpeedUse = ((slopeup * distanceTraveled) + minPowerUp);
                    //if it is in the second part of the trapezoid
                }else if((distanceTraveled >= rampdistanceUp) && (distanceTraveled <= (targetInches - rampdistanceDown))){
                    //set power to constant
                    initSpeedUse = maxPower;
                    //if it is in the third part of the trapezoid
                }else{
                    //set power to decrease
                    initSpeedUse = ((slopedown) * (distanceTraveled - targetInches) + minPowerDown);
                }
                //if the distance cannot fit in the ramp
            }else{
                //Compute triangle power evolution for distances less than rampup + rampdown
                if (distanceTraveled < targetInches/2) {
                    initSpeedUse = (slopeup * distanceTraveled) + minPowerUp;
                }else{
                    initSpeedUse = ((slopedown) * (distanceTraveled - targetInches) + minPowerDown);
                }
            }

            //set powers
            hardware.frontLeftDriveMotor.setPower(initSpeedUse + poL);
            hardware.frontRightDriveMotor.setPower(initSpeedUse + poR);
            hardware.backLeftDriveMotor.setPower(initSpeedUse + poL);
            hardware.backRightDriveMotor.setPower(initSpeedUse + poR);

            //set last powers
            Ilast = I;
            errorlast = error;
            timeLast = timeNow;

            //telemetry for testing
            telemetry.addData("SpeedUse:", initSpeedUse);
            telemetry.addData("PoL: ", poL);
            telemetry.addData("PoR: ", poR);
            telemetry.addData("angleNOW",anglenow);
            telemetry.addData("reference1", reference1);
            telemetry.addData("error: ", error);
            telemetry.addData("output: ", output);
            telemetry.addData("P: ", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            telemetry.update();
        }
        hardware.frontLeftDriveMotor.setPower(0);
        hardware.frontRightDriveMotor.setPower(0);
        hardware.backLeftDriveMotor.setPower(0);
        hardware.backRightDriveMotor.setPower(0);
    }

    public void PIDstraightInchesNoRamp(double gainP, double gainI, double gainD, double maxPower, int direction, double targetInches, double reference1, Hardware hardware) {
        //Function to move forward with PID
        /*direction + => forward
         direction - => backward*/
        double timeLast = 0; //The last time recorded
        double Ilast = 0;//The last integral value recorded
        double errorlast = 0;//The last error recorded
        double P;//Proportional
        double I;//Integral
        double D;//Derivative
        double dT;//Change in time
        double output;//Output of the controller
        double Kp = gainP;//Gain for P
        double Ki = gainI;//gain for I
        double Kd = gainD;//Gain for D
        double timeNow;//Current time
        double poL;//Left power
        double poR;//Right power
        double error;//Error
        double anglenow;//Current angle
        double initSpeedUse;//Speed to add to
        double distanceTraveled;//Distance traveled till now
        double minPowerUp = 0.3;//Minimum power for ramp up
        double minPowerDown = 0.2;//Minimum power for ramp down
        anglenow = getAngle(hardware);//Current angle
        double refTime = getRuntime();//time at start
        double elapsedTime = 0;//time passed
        long TIMESLEEP = 50;//Time to sleep for loop
        int ticsPerMotor = (1120);//Tics of the drive motors
        double circumference = 12.125;//Wheel circumference
        double ticsPerInch = (ticsPerMotor / circumference) / 2;//tics per inch moved
        double startPos = hardware.frontRightDriveMotor.getCurrentPosition();//start position of the otor

        int target = ((int)(targetInches * ticsPerInch)); //Where to go

        //go to target position for all the motors
        hardware.frontRightDriveMotor.setTargetPosition(direction *(target)+ hardware.frontRightDriveMotor.getCurrentPosition());
        hardware.frontLeftDriveMotor.setTargetPosition(direction *(target)+ hardware.frontLeftDriveMotor.getCurrentPosition());
        hardware.backRightDriveMotor.setTargetPosition(direction *(target)+ hardware.backRightDriveMotor.getCurrentPosition());
        hardware.backLeftDriveMotor.setTargetPosition(direction *(target)+ hardware.backLeftDriveMotor.getCurrentPosition());

        hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //while we are not at position
        while ((absolute((hardware.frontRightDriveMotor.getCurrentPosition() - startPos)) < (target) - 50) && opModeIsActive()) {

            sleep(TIMESLEEP);

            // calculate time
            timeNow = getRuntime();
            elapsedTime = timeNow - refTime;
            dT = timeNow - timeLast;

            // P
            anglenow = getAngle(hardware);
            error = anglenow - reference1;
            P = error;
            // I
            I = Ilast + (error * dT);
            // D
            D = (error - errorlast) / dT;

            //calculate output
            output = ((Kp * P) + (Ki * I) + (Kd * D));

            //power left to move
            poL = -output * direction;
            //power right to move
            poR = output * direction;

            //set max correction limits
            if (poR < -0.8) {
                poR = -0.8;
            }
            if (poL > 0.8) {
                poL = 0.8;
            }
            if (poL < -0.8) {
                poL = -0.8;
            }
            if (poR > 0.8) {
                poR = 0.8;
            }

            initSpeedUse = maxPower;

            //set powers
            hardware.frontLeftDriveMotor.setPower(initSpeedUse + poL);
            hardware.frontRightDriveMotor.setPower(initSpeedUse + poR);
            hardware.backLeftDriveMotor.setPower(initSpeedUse + poL);
            hardware.backRightDriveMotor.setPower(initSpeedUse + poR);

            //set last powers
            Ilast = I;
            errorlast = error;
            timeLast = timeNow;

            //telemetry for testing
            telemetry.addData("SpeedUse:", initSpeedUse);
            telemetry.addData("PoL: ", poL);
            telemetry.addData("PoR: ", poR);
            telemetry.addData("angleNOW",anglenow);
            telemetry.addData("reference1", reference1);
            telemetry.addData("error: ", error);
            telemetry.addData("output: ", output);
            telemetry.addData("P: ", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            telemetry.update();
        }
        hardware.frontLeftDriveMotor.setPower(0);
        hardware.frontRightDriveMotor.setPower(0);
        hardware.backLeftDriveMotor.setPower(0);
        hardware.backRightDriveMotor.setPower(0);
    }

    public void PIDsideInches(double gainP, double gainI, double gainD, double maxPower, int direction, double targetInches, double reference1, Hardware hardware) {
        //Function to move sideways with PID
        /*direction + => right
        direction - => left*/

        double timeLast = 0;//last time recorded
        double Ilast = 0;//last integral recorded
        double errorlast = 0;//last error recorded
        double P;//Proportional
        double I;//Integral
        double D;//Derivative
        double dT;//Change in time
        double output;//Output of controller
        double Kp = gainP;//gain for p
        double Ki = gainI;//gain for i
        double Kd = gainD;//gain for d
        double timeNow;//Current time
        double poB;//power for back wheels
        double poF;//power for forward wheels
        double error;//error
        double anglenow;//current angle
        double refTime = getRuntime();//time at start
        double elapsedTime = 0;//time passed
        long TIMESLEEP = 50;//time that we sleep
        int ticsPerMotor = (1120);//tics per drive motor
        double initSpeedUse;//speed to use
        double distanceTraveled;//distance traveled till now
        double minPowerUp = 0.3;//minimum power for ramp up
        double minPowerDown = 0.2;//minimum power for ramp down
        double circumference = 12.125;//circumference of wheel
        double ticsMultiple = 1.2;//multiple for slippage
        double ticsPerInch = ((ticsPerMotor / circumference) * ticsMultiple) / 2;//tics per inch of movement
        double startPos = hardware.frontRightDriveMotor.getCurrentPosition();//start position of motor

        int target = ((int)(targetInches * ticsPerInch)); //target position to move

        //go to target position
        hardware.frontRightDriveMotor.setTargetPosition(-direction *(target)+ hardware.frontRightDriveMotor.getCurrentPosition());
        hardware.frontLeftDriveMotor.setTargetPosition(direction *(target)+ hardware.frontLeftDriveMotor.getCurrentPosition());
        hardware.backRightDriveMotor.setTargetPosition(direction *(target)+ hardware.backRightDriveMotor.getCurrentPosition());
        hardware.backLeftDriveMotor.setTargetPosition(-direction *(target)+ hardware.backLeftDriveMotor.getCurrentPosition());

        hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //while we have not reached
        while ((absolute((hardware.frontRightDriveMotor.getCurrentPosition() - startPos)) < ((target)) - 50) && opModeIsActive()) {

            sleep(TIMESLEEP);

            // calculate time
            timeNow = getRuntime();

            elapsedTime = timeNow - refTime;
            dT = timeNow - timeLast;

            // P
            anglenow = getAngle(hardware);
            error = anglenow - reference1;
            P = error;
            // I
            I = Ilast + (error * dT);
            // D
            D = (error - errorlast) / dT;

            //calculate output
            output = ((Kp * P) + (Ki * I) + (Kd * D));

            //back motor power
            poB = output * direction;
            //front motor power
            poF = -output * direction;
            //set max corrections
            if (poF < -0.8) {
                poF = -0.8;
            }
            if (poB > 0.8) {
                poB = 0.8;
            }
            if (poB < -0.8) {
                poB = -0.8;
            }
            if (poF > 0.8) {
                poF = 0.8;
            }

            //ramp distances
            double rampdistanceUp = 10;
            double rampdistanceDown = 20;

            //distance traveled till now
            distanceTraveled = absolute((hardware.frontRightDriveMotor.getCurrentPosition() - startPos) / ticsPerInch);
            //if it will fit in the trapezoid
            if (targetInches > rampdistanceUp+rampdistanceDown){
                //if it is in the first part of the ramp
                if(distanceTraveled < rampdistanceUp){
                    //incrase speed
                    initSpeedUse = (((maxPower - minPowerUp) / rampdistanceUp) * (distanceTraveled) + minPowerUp);
                    //if it is in the second part of the ramp
                }else if((distanceTraveled >= rampdistanceUp) && (distanceTraveled <= (targetInches - rampdistanceDown))){
                    //remain at a constant speed
                    initSpeedUse = maxPower;
                    //if it is in the third part of the ramp
                }else{
                    //decrease speed
                    initSpeedUse = ((-(maxPower - minPowerDown) / rampdistanceDown) * (distanceTraveled - targetInches) + minPowerDown);
                }
                //if it is too small to fit in the ramp
            }else{
                //set to constant power
                initSpeedUse = maxPower;
            }

            //set powers
            hardware.frontLeftDriveMotor.setPower(initSpeedUse + poF);
            hardware.frontRightDriveMotor.setPower(initSpeedUse + poF);
            hardware.backLeftDriveMotor.setPower(initSpeedUse + poB);
            hardware.backRightDriveMotor.setPower(initSpeedUse + poB);

            //last variable values
            Ilast = I;
            errorlast = error;
            timeLast = timeNow;

            //update telemetry
            telemetry.addData("PoB: ", poB);
            telemetry.addData("PoF: ", poF);
            telemetry.addData("error: ", error);
            telemetry.addData("output: ", output);
            telemetry.addData("P: ", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            telemetry.addData("Time that's passed: ", elapsedTime);
            telemetry.update();

        }
        hardware.frontLeftDriveMotor.setPower(0);
        hardware.frontRightDriveMotor.setPower(0);
        hardware.backLeftDriveMotor.setPower(0);
        hardware.backRightDriveMotor.setPower(0);

    }

    public double moveTurnDegrees(double wheelPower, int direction, double degrees, Hardware hardware) {
        // direction true => right
        // direction false => left
        //this function is used to spin a certain number of degrees

        double ticsPerMotor = 1120;//tics for the drive motors
        double degreesPerRotation = 48;//degrees moved for one full rotation of the wheel
        double ticsToMove = ((degrees * ticsPerMotor) / degreesPerRotation) / 2;//tics needed to turn inputed number of degrees
        //targets for motor
        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 25;//tics tolerance in case it overshoots

        //if turning right
        if (direction > 0) {

            //go to target position
            FLtarget = (int)(-ticsToMove) + hardware.frontLeftDriveMotor.getCurrentPosition();
            FRtarget =(int)(ticsToMove)+ hardware.frontRightDriveMotor.getCurrentPosition();
            BLtarget = (int)(-ticsToMove) + hardware.backLeftDriveMotor.getCurrentPosition();
            BRtarget = (int)(ticsToMove)+ hardware.backRightDriveMotor.getCurrentPosition();

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //if turning left
        } else {
            //go to target position
            FLtarget = (int)(ticsToMove) + hardware.frontLeftDriveMotor.getCurrentPosition();
            FRtarget =(int)(-ticsToMove)+ hardware.frontRightDriveMotor.getCurrentPosition();
            BLtarget = (int)(ticsToMove) + hardware.backLeftDriveMotor.getCurrentPosition();
            BRtarget = (int)(-ticsToMove)+ hardware.backRightDriveMotor.getCurrentPosition();

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        //while not at target
        while ((absolute(hardware.frontLeftDriveMotor.getCurrentPosition()-FLtarget) > ticksTol) && (absolute(hardware.frontRightDriveMotor.getCurrentPosition()-FRtarget) > ticksTol) && (absolute(hardware.backLeftDriveMotor.getCurrentPosition()-BLtarget) > ticksTol ) && (absolute(hardware.backRightDriveMotor.getCurrentPosition()-BRtarget) > ticksTol) && (opModeIsActive())){
            //set power to the motors
            hardware.frontLeftDriveMotor.setPower(wheelPower);
            hardware.frontRightDriveMotor.setPower(wheelPower);
            hardware.backLeftDriveMotor.setPower(wheelPower);
            hardware.backRightDriveMotor.setPower(wheelPower);
        }
        hardware.frontLeftDriveMotor.setPower(0);
        hardware.frontRightDriveMotor.setPower(0);
        hardware.backLeftDriveMotor.setPower(0);
        hardware.backRightDriveMotor.setPower(0);

        return (0);
    }

    public double moveForwardInches(double wheelPower, boolean direction, double inches, Hardware hardware) {
        // direction true => forward
        // direction false => backward
        //this function is to move forward a certain number of inches

        int ticsPerMotor = 1120;//tics per drive motor
        double circumference = 12.125;//circumference of the wheels
        double ticsPerInch = (ticsPerMotor / circumference) / 2;//tics needed to drive an inch

        //wheel targets
        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 25;//tics tolerance in case of overshooting
        double poweruse;//power to use

        //if going forward
        if (direction) {
            //go to target position
            FLtarget = (int) (ticsPerInch * inches + hardware.frontLeftDriveMotor.getCurrentPosition());
            FRtarget = (int) (ticsPerInch * inches + hardware.frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int) (ticsPerInch * inches + hardware.backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int) (ticsPerInch * inches + hardware.backRightDriveMotor.getCurrentPosition());

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //if going backward
        } else {
            //go to target (negative)
            FLtarget = (int) (-ticsPerInch * inches + hardware.frontLeftDriveMotor.getCurrentPosition());
            FRtarget = (int) (-ticsPerInch * inches + hardware.frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int) (-ticsPerInch * inches + hardware.backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int) (-ticsPerInch * inches + hardware.backRightDriveMotor.getCurrentPosition());

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        //while not at target position
        while ((absolute(hardware.frontLeftDriveMotor.getCurrentPosition() - FLtarget) > ticksTol) && (absolute(hardware.frontRightDriveMotor.getCurrentPosition() - FRtarget) > ticksTol) && (absolute(hardware.backLeftDriveMotor.getCurrentPosition() - BLtarget) > ticksTol) && (absolute(hardware.backRightDriveMotor.getCurrentPosition() - BRtarget) > ticksTol) && (opModeIsActive())) {
            //set power
            poweruse = wheelPower;
            hardware.frontLeftDriveMotor.setPower(poweruse);
            hardware.frontRightDriveMotor.setPower(poweruse);
            hardware.backLeftDriveMotor.setPower(poweruse);
            hardware.backRightDriveMotor.setPower(poweruse);
        }
        hardware.frontLeftDriveMotor.setPower(0);
        hardware.frontRightDriveMotor.setPower(0);
        hardware.backLeftDriveMotor.setPower(0);
        hardware.backRightDriveMotor.setPower(0);
        return (0);

    }

    public double moveSideInches(double wheelPower, boolean direction, double inches, Hardware hardware) {
        //this function is to move sideways a specified number of inches
        // direction true => right
        // direction false => left
        double ticsPerMotor = 1120; //tics per drive motor
        double circumference = 12.125;//circumference of wheels
        double ticsPerInch = (ticsPerMotor / circumference) / 2;//tics needed to move an inch

        //targets for the wheels
        int FLtarget;
        int FRtarget;
        int BLtarget;
        int BRtarget;
        int ticksTol = 25;//tics tolerance in case of overshooting
        double poweruse;//Power to use

        //if going forward
        if (direction) {
            double sideMultiple = 1.2;//multiple for slippage while going sideways
            //go to target position
            FLtarget = (int) (ticsPerInch * sideMultiple * inches + hardware.frontLeftDriveMotor.getCurrentPosition());
            FRtarget = (int) (-ticsPerInch * sideMultiple * inches + hardware.frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int) (-ticsPerInch * sideMultiple * inches + hardware.backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int) (ticsPerInch * sideMultiple * inches + hardware.backRightDriveMotor.getCurrentPosition());

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //if going backward
        } else {
            double sideMultiple = 1.2;//multiple for slippage while going sideways

            //go to target positions
            FLtarget = (int) (-ticsPerInch * sideMultiple * inches + hardware.frontLeftDriveMotor.getCurrentPosition());
            FRtarget = (int) (ticsPerInch * sideMultiple * inches + hardware.frontRightDriveMotor.getCurrentPosition());
            BLtarget = (int) (ticsPerInch * sideMultiple * inches + hardware.backLeftDriveMotor.getCurrentPosition());
            BRtarget = (int) (-ticsPerInch * sideMultiple * inches + hardware.backRightDriveMotor.getCurrentPosition());

            hardware.frontLeftDriveMotor.setTargetPosition(FLtarget);
            hardware.frontRightDriveMotor.setTargetPosition(FRtarget);
            hardware.backLeftDriveMotor.setTargetPosition(BLtarget);
            hardware.backRightDriveMotor.setTargetPosition(BRtarget);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        //while not at target position
        while ((absolute(hardware.frontLeftDriveMotor.getCurrentPosition() - FLtarget) > ticksTol) && (absolute(hardware.frontRightDriveMotor.getCurrentPosition() - FRtarget) > ticksTol) && (absolute(hardware.backLeftDriveMotor.getCurrentPosition() - BLtarget) > ticksTol) && (absolute(hardware.backRightDriveMotor.getCurrentPosition() - BRtarget) > ticksTol) && (opModeIsActive())) {
            //set power
            poweruse = wheelPower;
            hardware.frontLeftDriveMotor.setPower(poweruse);
            hardware.frontRightDriveMotor.setPower(poweruse);
            hardware.backLeftDriveMotor.setPower(poweruse);
            hardware.backRightDriveMotor.setPower(poweruse);
        }
        hardware.frontLeftDriveMotor.setPower(0);
        hardware.frontRightDriveMotor.setPower(0);
        hardware.backLeftDriveMotor.setPower(0);
        hardware.backRightDriveMotor.setPower(0);
        return (0);

    }

    public void moveArchDegrees(double outsidePower, double degrees, double radius1, double directionForward, double directionTurn, Hardware hardware){
        //This function moves in an arch with specified measurements
        /*directionForward + => forward
          directionForward - => backward
          directionTurn + => right
          directionTurn - => left*/
        double ticsPerMotor = 1120;//tics per drive motor
        double circumference = 12.125;//wheel circumference
        double ticsPerInch = (ticsPerMotor / circumference) / 2;//tics needed to move forward one inch
        double radius2 = radius1 - WHEEL_DISTANCE;//calculate inside radius by subtracting wheel distance from inputed outside radius
        double circumference1 = 2 * ((Math.PI) * radius1);//calculate outside circle circumference (2*PI*Radius)
        double circumference2 = 2 * ((Math.PI) * radius2);//calculate inside circle circumference (2*PI*Radius)
        double distance1 = (((circumference1 * degrees) / 360) * 2);//calculate distance around the outside circle to move
        int tics1 = (int) (directionForward * (distance1 * ticsPerInch));//tics to move by
        double distance2 = (((circumference2 * degrees) / 360) * 2);//calculate distance around the inside circle to move
        int tics2 = (int) (directionForward * (distance2 * ticsPerInch));//tics to move by
        double insidePower = ((outsidePower * radius2) / radius1);//calculate inside power by setting up and solving a proportion
        //if turning left
        if (directionTurn < 0){
            //go to target position
            hardware.frontLeftDriveMotor.setTargetPosition(hardware.frontLeftDriveMotor.getCurrentPosition() + tics1);
            hardware.frontRightDriveMotor.setTargetPosition(hardware.frontRightDriveMotor.getCurrentPosition() + tics2);
            hardware.backLeftDriveMotor.setTargetPosition(hardware.backLeftDriveMotor.getCurrentPosition() + tics1);
            hardware.backRightDriveMotor.setTargetPosition(hardware.backRightDriveMotor.getCurrentPosition() + tics2);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //while not at target
            while ((absolute(hardware.frontLeftDriveMotor.getCurrentPosition() - (hardware.frontLeftDriveMotor.getTargetPosition())) > 50) && opModeIsActive()){
                //set power
                hardware.frontLeftDriveMotor.setPower(directionForward * outsidePower);
                hardware.frontRightDriveMotor.setPower(directionForward * insidePower);
                hardware.backLeftDriveMotor.setPower(directionForward * outsidePower);
                hardware.backRightDriveMotor.setPower(directionForward * insidePower);
            }
            hardware.frontLeftDriveMotor.setPower(0);
            hardware.frontRightDriveMotor.setPower(0);
            hardware.backLeftDriveMotor.setPower(0);
            hardware.backRightDriveMotor.setPower(0);
            //if turning right
        } else {
            //go to target position
            hardware.frontLeftDriveMotor.setTargetPosition(hardware.frontLeftDriveMotor.getCurrentPosition() + tics2);
            hardware.frontRightDriveMotor.setTargetPosition(hardware.frontRightDriveMotor.getCurrentPosition() + tics1);
            hardware.backLeftDriveMotor.setTargetPosition(hardware.backLeftDriveMotor.getCurrentPosition() + tics2);
            hardware.backRightDriveMotor.setTargetPosition(hardware.backRightDriveMotor.getCurrentPosition() + tics1);

            hardware.frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //while not at target position
            while ((absolute(hardware.frontRightDriveMotor.getCurrentPosition() - hardware.frontRightDriveMotor.getTargetPosition()) > 50) && (opModeIsActive())) {
                //set power
                hardware.frontLeftDriveMotor.setPower(insidePower);
                hardware.frontRightDriveMotor.setPower(outsidePower);
                hardware.backLeftDriveMotor.setPower(insidePower);
                hardware.backRightDriveMotor.setPower(outsidePower);
            }
            hardware.frontLeftDriveMotor.setPower(0);
            hardware.frontRightDriveMotor.setPower(0);
            hardware.backLeftDriveMotor.setPower(0);
            hardware.backRightDriveMotor.setPower(0);
        }
    }

    public void liftOrDropStones(double leftOrRight, double pickOrDrop, Hardware hardware){
        //left => -
        //right => +
        //pick => +
        //drop => -
        //this function either picks up or drops stones using either stone picker.
        //if picking up stones
        if(pickOrDrop > 0){
            //if using right stone picker
            if(leftOrRight > 0){
                //set both servos to down
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_RIGHT_DOWN);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_DOWN_FULLY);
                sleep(500);
                //go to position is set to down before the loop
                double goToR = SMALL_STONE_PICKER_RIGHT_DOWN;
                double goToR2 = STONE_PICKER_RIGHT_DOWN_FULLY;
                //while the small picker is not at the target
                while((goToR < SMALL_STONE_PICKER_PICKUP) && (opModeIsActive())){
                    //slowly change position to grab the block and pick it up
                    goToR = goToR + 0.05;
                    goToR2 = goToR2 - 0.04;
                    hardware.smallStoneServoRight.setPosition(goToR);
                    hardware.stoneServoRight.setPosition(goToR2);
                    sleep(50);
                }
                //if using left stone picker
            } else if(leftOrRight < 0){
                //set both servos to down
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_LEFT_DOWN);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_DOWN_FULLY);
                sleep(500);
                //go to position is set to down before the loop
                double goToL = SMALL_STONE_PICKER_LEFT_DOWN;
                double goToL2 = STONE_PICKER_LEFT_DOWN_FULLY;
                //while the small picker is not at the target
                while((goToL < SMALL_STONE_PICKER_PICKUP) && opModeIsActive()){
                    //slowly change position and grab the block and pick it up
                    goToL = goToL + 0.05;
                    goToL2 = goToL2 - 0.04;
                    hardware.smallStoneServoLeft.setPosition(goToL);
                    hardware.stoneServoLeft.setPosition(goToL2);
                    sleep(50);
                }
            }
            //if dropping the stones
        } else if (pickOrDrop < 0){
            //if using right picker
            if(leftOrRight > 0){
                //set positions to picking up the stone
                hardware.smallStoneServoRight.setPosition(SMALL_STONE_PICKER_PICKUP);
                hardware.stoneServoRight.setPosition(STONE_PICKER_RIGHT_PICKUP);
                sleep(500);
                //set go to positions to pickup before the loop
                double goToR = SMALL_STONE_PICKER_PICKUP;
                double goToR2 = STONE_PICKER_RIGHT_PICKUP;
                //while small picker is not at target
                while((goToR > SMALL_STONE_PICKER_RIGHT_DOWN) && (opModeIsActive())){
                    //slowly change position and let go of the block and lower it down
                    goToR = goToR - (0.05);
                    goToR2 = goToR2 + (0.04);
                    hardware.smallStoneServoRight.setPosition(goToR);
                    hardware.stoneServoRight.setPosition(goToR2);
                    sleep(50);
                }
                //if using left picker
            }else if(leftOrRight < 0){
                //set positions to picking up the stone
                hardware.smallStoneServoLeft.setPosition(SMALL_STONE_PICKER_PICKUP);
                hardware.stoneServoLeft.setPosition(STONE_PICKER_LEFT_PICKUP);
                sleep(500);
                //set go to positions to pickup before the loop
                double goToL = SMALL_STONE_PICKER_PICKUP;
                double goToL2 = STONE_PICKER_LEFT_PICKUP;
                //while small picker is not at target
                while((goToL > SMALL_STONE_PICKER_LEFT_DOWN) && (opModeIsActive())){
                    //slowly change position and let go of the block and lower it down
                    goToL = goToL - (0.05);
                    goToL2 = goToL2 + (0.04);
                    hardware.smallStoneServoLeft.setPosition(goToL);
                    hardware.stoneServoLeft.setPosition(goToL2);
                    sleep(50);
                }
            }
        }
    }

    public double getAngle(Hardware hardware){
        //this function gets the current angle of the imu
        Orientation angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //init imu
        return angles.firstAngle;//return angle
    }

    public double absolute(double inputval) {
        //this function gets the absolute value of an inputed number
        //if it's positive
        if (inputval > 0) {
            //keep it the same
            return inputval;
            //if it's negative
        } else {
            //multiply by negative 1
            return -1 * inputval;
        }
    }
}