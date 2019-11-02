package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link TestPID} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 *
 * Autonomous mode for BLUE to move two Skystones to building zone and park under skybridge
 */
@Autonomous(name = "BlueSkystoneParkClose", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list

public class AutoBlueSkystoneParkClose extends LinearOpMode {
    public void runOpMode (){
        // set duration to move to block
        // set duration to move arm down

        // set blockCounter = 0

        // while (blockCounter < 2)
            // move toward blocks using timer and move arm below 14 inches
                // arrived = false
                // armIsDown = false
                // get start time
                // set power to drive wheels
                // set power to arm
                // while (!arrived or !armIsDown)
                //      if !arrived and (start time + duration to move to block > current time)
                //          set power to drive wheels to 0
                //          arrived = true
                //      if !armIsDown and (start time + duration to move arm down > current time)
                //          set power to arm to 0
                //          armIsDown = true

            // move toward blocks using distance sensor and move arm below 14 inches (later)
                // arrived = false
                // armIsDown = false
                // set power to drive wheels
                // set power to arm
                // while (!arrived or !armIsDown)
                //      if !arrived and (distance sensor reading < 6 inches)
                //          set power to drive wheels to 0
                //          arrived = true
                //      if !armIsDown and (start time + duration to move arm down > current time)
                //          set power to arm to 0
                //          armIsDown = true
                //
                // skystoneFound = false
                // while !skystoneFound
                //      set power to drive wheels to move toward wall
                //      if (camera detects skystone)
                //          set power to drive wheels to stop moving toward wall
                //          set power to drive wheels to get block

            // use camera to detect skystone (later)
            // arrived = false
            // armIsDown = false
            // set power to drive wheels
            // set power to arm
            // while (!arrived or !armIsDown)
            //      if !arrived and (camera indicates block is in front and x inches away)
            //          set power to drive wheels to 0
            //          arrived = true
            //      if !armIsDown and (start time + duration to move arm down > current time)
            //          set power to arm to 0
            //          armIsDown = true
            //
            // make minor adjustments to grab skystone using camera/distance sensor

            // grab block(skystone)
                // set power to block grabber servo to grab block

            // move back, pulling skystone
                // set power to drive wheels to move back (2 second?)

            // move to loading zone with block
                // set power to drive wheels to move to loading zone (5 seconds?)
                // later
                //      move toward building zone trying to detect blue tape
                //      move toward building zone for 1 more second?

            // release block
                // set power to block grabber servo to release block
                // blockCounter++
                // if (blockCounter == 1)
                    // move toward loading zone
                        // set power to drive wheels to move to loading zone
                // else
                    // park under skybridge
                        //drive toward loading zone
                        //stop when tape detected
    }
}