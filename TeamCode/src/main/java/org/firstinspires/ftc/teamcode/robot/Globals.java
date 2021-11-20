package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Globals {
    static public OpMode opmode = null;

    static public OpMode opmode(OpMode om) {
        if (opmode == null) {
            opmode = om;
        }
        return opmode;
    }
}
