package com.qualcomm.ftcrobotcontroller.opmodes.Team2891;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * Created by Team 2891 on 3/14/2016.
 */
public class Methods{

    public static Telemetry telemetry = new Telemetry();

    public static void doStuff () {
        telemetry.addData("poop", "poop");
    }

}
