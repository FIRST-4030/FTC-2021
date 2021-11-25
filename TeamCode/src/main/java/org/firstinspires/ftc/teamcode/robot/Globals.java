package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.gamepad.InputHandler;

public class Globals {
    static public OpMode opmode = null;
    static public InputHandler input = null;

    // Callers must provide an OpMode, so this will never fail to return an OpMode
    // Makes it easy to use in one-liners that may or may not be the parent OpMode
    static public OpMode opmode(OpMode om) {
        if (opmode == null) {
            Globals.opmode = om;
        }
        return opmode;
    }

    static public InputHandler input(OpMode om) {
        if (input == null) {
            input = new InputHandler(opmode(om));
        }
        return input;
    }
}
