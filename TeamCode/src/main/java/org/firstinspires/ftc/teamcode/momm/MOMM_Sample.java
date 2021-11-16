package org.firstinspires.ftc.teamcode.momm;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MOMM_Sample", group = "MOMM")
public class MOMM_Sample extends MultiOpModeManager {
    // External OMs
    // OMs that you will call directly should have members
    // OMs that are complete independent can be defined in-line (see init())
    private MOMM_Drive drive;

    /*
     * Standard OM methods
     *
     * All of the standard OM methods are available for @override
     * It is acceptable to exclude a method; for example, if init_loop is empty it can be excluded
     * If you define one of the standard methods it must call the same method's super()
     * super() can be called anywhere in the method; first or last is often best
     */

    @Override
    public void init() {
        // Register the drive OM
        drive = new MOMM_Drive();
        super.register(drive);

        // Register the speech OM
        super.register(new MOMM_Speech());

        // Be sure to register other OMs before this line or they won't get an init() call
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        // Adjust the drive power from outside the OM
        if ((int) time % 2 == 0) {
            drive.lowSpeed(0.5);
        } else {
            drive.lowSpeed(1.0);
        }

        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}