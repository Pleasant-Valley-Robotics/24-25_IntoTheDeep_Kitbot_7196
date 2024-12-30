/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class BatteryDatalogger
{
    Datalog datalog;
    BNO055IMU imu;
    VoltageSensor battery;

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");

        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");
        public Datalogger.GenericField totalMotorBatteryConsumption          = new Datalogger.GenericField("totalMotorBatteryConsumption");
        public Datalogger.GenericField armBatteryConsumption       = new Datalogger.GenericField("armBatteryConsumption");
        public Datalogger.GenericField leftDriveBatteryConsumption        = new Datalogger.GenericField("leftDriveBatteryConsumption");
        public Datalogger.GenericField rightDriveBatteryConsumption = new Datalogger.GenericField("rightDriveBatteryConsumption");
        public Datalogger.GenericField accessoriesBatteryConsumption = new Datalogger.GenericField("accessoriesBatteryConsumption");
        public Datalogger.GenericField batteryCapacity = new Datalogger.GenericField("batteryCapacity");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            loopCounter,
                            totalMotorBatteryConsumption,
                            armBatteryConsumption,
                            leftDriveBatteryConsumption,
                            rightDriveBatteryConsumption,
                            accessoriesBatteryConsumption,
                            batteryCapacity
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}
