package org.firstinspires.ftc.teamcode.momm.sample;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Globals;

@Config
public class MOMM_Speech extends OpMode {
    // Constants
    private static final double SPEECH_DELAY = 5.0;

    // Members
    OpMode om;
    ElapsedTime speechDelay = new ElapsedTime();

    @Override
    public void init() {
        // Pull in Globals
        om = Globals.opmode(this);
        telemetry = om.telemetry;
    }

    @Override
    public void init_loop() {
        // Use om.time, in case we aren't the primary OpMode
        telemetry.addData("Init Time", "%.2f", om.time);
    }

    @Override
    public void start() {
        telemetry.log().add(getClass().getSimpleName() + ": start()");
    }

    @Override
    public void loop() {
        if (speechDelay.seconds() > SPEECH_DELAY) {
            telemetry.speak("Speak");
            speechDelay.reset();
        }
        telemetry.addData("Loop Time", "%.2f", om.time);
    }
}
