package org.firstinspires.ftc.teamcode.momm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class MOMM_Speech extends MultiOpModeManager {
    // Constants
    private static final double SPEECH_DELAY = 5.0;

    // Members
    ElapsedTime speechDelay = new ElapsedTime();

    @Override
    public void init_loop() {
        telemetry.addData("Init Time", "%.2f", time);
    }

    @Override
    public void start() {
        telemetry.log().add(getClass().getSimpleName() + ": start()");
    }

    @Override
    public void loop() {
        if (speechDelay.seconds() > SPEECH_DELAY) {
            speechDelay.reset();
            telemetry.speak("Speak");
        }
        telemetry.addData("Loop Time", "%.2f", time);
    }
}
