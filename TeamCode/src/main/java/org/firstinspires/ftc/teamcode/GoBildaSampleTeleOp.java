/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Datalogging.BatteryDatalogger;

import java.util.ArrayList;

/*
 * This OpMode is an example driver-controlled (TeleOp) mode for the goBILDA 2024-2025 FTC
 * Into The Deep Starter Robot
 * The code is structured as a LinearOpMode
 *
 * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
 * With a left and right drive motor.
 * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
 * controlling the forward movement and the right stick X axis controlling rotation.
 * This allows easy transition to a standard "First Person" control of a
 * mecanum or omnidirectional chassis.
 *
 * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
 * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
 *
 * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
 * by a servo, and an intake driven by a continuous rotation servo.
 *
 * The arm is powered by a 5203-2402-0051 (50.9:1 Yellow Jacket Planetary Gearmotor) with an
 * external 5:1 reduction. This creates a total ~254.47:1 reduction.
 * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
 * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
 * starting position.
 *
 * Make super sure that the arm is reset into the robot, and the wrist is folded in before
 * you run start the OpMode. The motor's encoder is "relative" and will move the number of degrees
 * you request it to based on the starting position. So if it starts too high, all the motor
 * setpoints will be wrong.
 *
 * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
 *
 * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
 */


@TeleOp(name = "FTC Starter Kit Example Robot (INTO THE DEEP)", group = "Robot")
//@Disabled
public class GoBildaSampleTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftDrive = null; //the left drivetrain motor
    public DcMotor rightDrive = null; //the right drivetrain motor
    public DcMotor armMotor = null; //the arm motor
    public Servo claw = null;
    public Servo wrist = null; //the wrist servo
    public DistanceSensor distanceSensor = null;
    public LynxModule controlHub = null;


    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    //Was 160 before.
    final double ARM_SCORE_SAMPLE_IN_LOW = 140 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.4;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    public final static double batteryCapacity = 3000;  //Storage of battery in mAh
    double dT; //Time since last loop iteration in seconds
    public static double armBatteryConsumption = 0.0;
    public static double leftDriveBatteryConsumption = 0.0;
    public static double rightDriveBatteryConsumption = 0.0;
    public static double totalMotorBatteryConsumption = 0.0;
    public static double accessoriesBatteryConsumption = 0.0;
    private BatteryDatalogger.Datalog datalog;
    private BHI260IMU imu;
    private VoltageSensor battery;
    private int loopCounterVar = 1;
    private double openClawPosition = .3;
    private double closeClawPosition = 0;
    private volatile ArrayList<Double> distanceAverages;

    @Override
    public void runOpMode() {
        //Coding to be done
        //Make arm stay in position to grab sample off the wall via button press. (armMotor encoders = -110.)
        //Make arm go backwards to scoring on high rung position via button press. (

        //Rotating servo goes left 30 degrees, right 30 degrees, rests at 180.
        //Servo open and closing clip starts at 180, goes 90 to the left.
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double armDrive;
        double rotate;
        double max;

        telemetry.setAutoClear(true);

        /* Define and Initialize Motors */
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive"); //the left drivetrain motor
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive"); //the right drivetrain motor
        armMotor = hardwareMap.get(DcMotor.class, "liftArm"); //the arm motor
        controlHub = ((LynxModule) hardwareMap.get(LynxModule.class, "Control Hub"));//the control hub
        battery = hardwareMap.voltageSensor.get("Control Hub");

        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        //armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* Define and initialize servos.*/
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        /* Make sure that the intake is off, and the wrist is folded in. */
        wrist.setPosition(0.5);
        claw.setPosition(openClawPosition);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        //Battery Consumption Math:
        ElapsedTime timePassed = new ElapsedTime();

        // Get devices from the hardwareMap.
        // If needed, change "Control Hub" to (e.g.) "Expansion Hub 1".
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        // Initialize the datalog
        datalog = new BatteryDatalogger.Datalog("datalog_01");

        // You do not need to fill every field of the datalog
        // every time you call writeLine(); those fields will simply
        // contain the last value.
        datalog.opModeStatus.set("INIT");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();

        //BHI260IMU.Parameters parameters = new BHI260IMU();
        //parameters.angleUnit = BHI260IMU.AngleUnit.DEGREES;
        //imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);

        /* Wait for the game driver to press play */
        waitForStart();

        //distanceAverages = new ArrayList<>(10);

        //for (int i = 0; i < 10; i++) {
            //distanceAverages.add(0.0);
        //}

        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            /* Set the drive and turn variables to follow the joysticks on the gamepad.
            the joysticks decrease as you push them up. So reverse the Y axis. */
            forward = -gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
            armDrive = -gamepad2.right_stick_y;
            
            /* Here we "mix" the input channels together to find the power to apply to each motor.
            The both motors need to be set to a mix of how much you're retesting the robot move
            forward, and how much you're requesting the robot turn. When you ask the robot to rotate
            the right and left motors need to move in opposite directions. So we will add rotate to
            forward for the left motor, and subtract rotate from forward for the right motor. */

            left = forward + rotate;
            right = forward - rotate;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            armMotor.setPower(armDrive);

            if (gamepad1.right_bumper) {
                leftDrive.setPower(left / 2.0);
                rightDrive.setPower(right / 2.0);
            } else {
                /* Set the motor power to the variables we've mixed and normalized */
                leftDrive.setPower(left);
                rightDrive.setPower(right);
            }

            //Open servo
            if (gamepad2.left_bumper) {
                claw.setPosition(0.4);
            }
            //Close servo
            if (gamepad2.right_bumper) {
                claw.setPosition(closeClawPosition);
            }

            if (gamepad2.right_trigger > 0.50) {
                wrist.setPosition(0.5);
            }

            if (gamepad2.left_trigger > 0.50) {
                wrist.setPosition(0.15);
            }

            //Set arm and clip to score on high rung.
            if (gamepad2.dpad_up) {
                //convert the encoder val to a degree value.
                //If lower than 80 degrees move arm up.
                if (armMotor.getCurrentPosition() * ARM_TICKS_PER_DEGREE < 80) {
                    armMotor.setPower(0.3);
                }
                //if over 83 degrees bring arm down.
                else if (armMotor.getCurrentPosition() * ARM_TICKS_PER_DEGREE > 83) {
                    armMotor.setPower(-0.3);
                }
                //else arm doesn't need to move or do anything.
                else {
                }
                claw.setPosition(closeClawPosition);
                //teleopArmRotate(0.3, 80.0);
            }

            //Bring arm down to score specimen and open clip.
            if (gamepad2.dpad_down) {
                //if arm's lower than 60 degrees raise it.
                if (armMotor.getCurrentPosition() * ARM_TICKS_PER_DEGREE < 60) {
                    armMotor.setPower(0.3);
                }
                //if arm's higher than 63 degrees bring arm down.
                else if (armMotor.getCurrentPosition() * ARM_TICKS_PER_DEGREE > 63) {
                    armMotor.setPower(-0.3);
                }
                //arm doesn't need to do anything.
                else {
                }
                //teleopArmRotate(1.0, 60);
                claw.setPosition(openClawPosition);
            }

            //Set arm to pick up off the wall.
            if (gamepad2.dpad_left) {
                //if the arm's lower than 35 degrees raise it.
                if (armMotor.getCurrentPosition() * ARM_TICKS_PER_DEGREE < 35) {
                    armMotor.setPower(0.3);
                }
                //if the arm's higher than 35 degrees lower the arm.
                else if (armMotor.getCurrentPosition() * ARM_TICKS_PER_DEGREE > 38) {
                    armMotor.setPower(-0.3);
                }
                //Arm doesn't need to do anything.
                else {
                }
                teleopArmRotate(0.3, 35.0);
            }

            //distanceAverages.add(distanceSensor.getDistance(DistanceUnit.INCH));
            //distanceAverages.remove(0);

            //double sum = 0.0;
            //for (double num : distanceAverages) {
                //sum += num;
            //}
            //double avg = sum / (double) distanceAverages.size();

            //telemetry.addData("distanceSensorArraylist: ", distanceAverages.toString());
            telemetry.addData("distanceSensor: ", distanceSensor.getDistance(DistanceUnit.INCH));

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            //Battery Consumption Statistics
            dT = timePassed.time();
            timePassed.reset();
            //Add up the battery usage from the last loop so that the voltage used by the end of the program is accurate.
            totalMotorBatteryConsumption = totalMotorBatteryConsumption + (armBatteryConsumption + leftDriveBatteryConsumption + rightDriveBatteryConsumption + controlHub.getCurrent(CurrentUnit.MILLIAMPS)) * dT / 3600;
            armBatteryConsumption = armBatteryConsumption + ((DcMotorEx) armMotor).getCurrent(CurrentUnit.MILLIAMPS) * dT / 3600;
            leftDriveBatteryConsumption = leftDriveBatteryConsumption + ((DcMotorEx) leftDrive).getCurrent(CurrentUnit.MILLIAMPS) * dT / 3600;
            rightDriveBatteryConsumption = rightDriveBatteryConsumption + ((DcMotorEx) rightDrive).getCurrent(CurrentUnit.MILLIAMPS) * dT / 3600;
            accessoriesBatteryConsumption = accessoriesBatteryConsumption + (totalMotorBatteryConsumption - armBatteryConsumption - leftDriveBatteryConsumption - rightDriveBatteryConsumption) * dT / 3600;

            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("Arm Current:", ((DcMotorEx) armMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Left Drive Current:", ((DcMotorEx) leftDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Drive Current:", ((DcMotorEx) rightDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("dT: ", dT);
            telemetry.addData("Arm Battery Consumption mAh: ", armBatteryConsumption);
            telemetry.addData("Left Drive Battery Consumption mAh: ", leftDriveBatteryConsumption);
            telemetry.addData("Right Drive Battery Consumption mAh: ", rightDriveBatteryConsumption);
            telemetry.addData("Total Robot Battery Consumption mAh: ", totalMotorBatteryConsumption);
            telemetry.addData("Accessories Battery Consumption mAh: ", accessoriesBatteryConsumption);

            //Datalogging
            datalog.opModeStatus.set("RUNNING");

            datalog.loopCounter.set(loopCounterVar);
            datalog.battery.set(battery.getVoltage());

            datalog.totalMotorBatteryConsumption.set(GoBildaSampleTeleOp.totalMotorBatteryConsumption);
            datalog.armBatteryConsumption.set(GoBildaSampleTeleOp.armBatteryConsumption);
            datalog.leftDriveBatteryConsumption.set(GoBildaSampleTeleOp.leftDriveBatteryConsumption);
            datalog.rightDriveBatteryConsumption.set(GoBildaSampleTeleOp.rightDriveBatteryConsumption);
            datalog.accessoriesBatteryConsumption.set(GoBildaSampleTeleOp.accessoriesBatteryConsumption);
            datalog.batteryCapacity.set(GoBildaSampleTeleOp.batteryCapacity);

            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();

            // Datalog fields are stored as text only; do not format here.
            telemetry.addData("totalMotorBatteryConsumption", datalog.totalMotorBatteryConsumption);
            telemetry.addData("armBatteryConsumption", datalog.armBatteryConsumption);
            telemetry.addData("leftDriveBatteryConsumption", datalog.leftDriveBatteryConsumption);
            telemetry.addData("rightDriveBatteryConsumption", datalog.rightDriveBatteryConsumption);
            telemetry.addData("accessoriesBatteryConsumption", datalog.accessoriesBatteryConsumption);
            telemetry.addLine();
            telemetry.addData("OpMode Status", datalog.opModeStatus);
            telemetry.addData("Loop Counter", datalog.loopCounter);
            telemetry.addData("Battery", datalog.battery);
            telemetry.addData("batteryCapacity", datalog.batteryCapacity);

            telemetry.update();

            sleep(20);

            //Increase by one the amount of times the code has run.
            loopCounterVar++;
        }
    }

    public void autoArmRotate(double speed, double degrees) {
        int newArmTarget;

        if (opModeIsActive()) {
            newArmTarget = armMotor.getCurrentPosition() + (int) (degrees * ARM_TICKS_PER_DEGREE);

            armMotor.setTargetPosition(newArmTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (armMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to: ", newArmTarget);
                telemetry.addData("Currently at: ", armMotor.getCurrentPosition());
                telemetry.update();
            }

            armMotor.setPower(0);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //if (!hold) {
            //  armMotor.setPower(0);
            //}
        }
    }

    public void teleopArmRotate(double speed, double degrees) {
        int newArmTarget;

        if (opModeIsActive()) {
            newArmTarget = (int) (degrees * ARM_TICKS_PER_DEGREE);

            armMotor.setTargetPosition(newArmTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (armMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to: ", newArmTarget);
                telemetry.addData("Currently at: ", armMotor.getCurrentPosition());
                telemetry.update();
            }

            armMotor.setPower(0);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
