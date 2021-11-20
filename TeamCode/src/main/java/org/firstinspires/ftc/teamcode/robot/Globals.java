package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Globals {
    static public OpMode opmode = null;

    // Callers must provide an OpMode, so this will never fail to return an OpMode
    // Makes it easy to use in one-liners that may or may not be the parent OpMode
    static public OpMode opmode(OpMode om) {
        if (opmode == null) {
            opmode = om;
        }
        return opmode;
    }
}
