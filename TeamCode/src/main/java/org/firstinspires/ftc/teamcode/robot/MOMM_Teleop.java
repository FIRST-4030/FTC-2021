package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;

// Extend MultiOpModeManager instead of OpMode
// Register with @TeleOp or @Autonomous as with any other OpMode
@TeleOp(name = "MOMM_Teleop", group = "MOMM")
@Disabled
public class MOMM_Teleop extends MultiOpModeManager {
    // External OMs
    // OMs that you will call directly should have members
    // OMs that are complete independent can be defined in-line (see init())
    private Drive drive;
    private DuckSpin duck;

    /*
     * Standard OM methods
     *
     * All of the standard OM methods are available to @override
     * It is acceptable to exclude a method; for example, if init_loop is empty it can be excluded
     * If you define one of the standard methods it must call the same method's super()
     * super() can be called anywhere within the method; first or last is often easiest to grok
     */

    @Override
    public void init() {
        // Register the drive OM
        drive = new Drive();
        super.register(drive);

        // Register the duck spinner OM
        duck = new DuckSpin();
        super.register(duck);

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
        // Start the auto method for the duck spinner
        if (duck.isDone()) {
            if (gamepad2.left_stick_button) {
                duck.auto(true);
            } else if (gamepad2.right_stick_button) {
                duck.auto(false);
            }
        }

        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}