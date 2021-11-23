package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;

@TeleOp(name = "MOMM_Teleop", group = "MOMM")
public class MOMM_Teleop extends MultiOpModeManager {
    private Drive drive;
    private DuckSpin duck;
    private Distance distance;
    private Depositor depositor;

    public static int TURN_SMALL = 5;
    public static float TURN_SPEED = 0.4f;

    @Override
    public void init() {
        super.register(new Depositor());
        super.register(new Capstone());
        super.register(new Collector());

        drive = new Drive();
        super.register(drive);
        duck = new DuckSpin();
        super.register(duck);
        distance = new Distance();
        super.register(distance);
        depositor = new Depositor();
        super.register(depositor);

        input.register("BARCODE", gamepad2, PAD_KEY.guide);
        input.register("TURN_CW", gamepad1, PAD_KEY.a);
        input.register("TURN_CCW", gamepad1, PAD_KEY.b);

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
        super.loop();

        // Start the auto method for the duck spinner
        if (duck.isDone()) {
            if (gamepad2.left_stick_button) {
                duck.auto(true);
            } else if (gamepad2.right_stick_button) {
                duck.auto(false);
            }
        }

        // Small angle turns
        if (input.down("TURN_CW")) {
            drive.turnTo(TURN_SPEED, TURN_SMALL);
        }
        if (input.down("TURN_CCW")) {
            drive.turnTo(-TURN_SPEED, -TURN_SMALL);
        }

        // Trigger a distance scan manually at any time
        if (input.down("BARCODE")) {
            distance.startScan();
        }
        // Trigger updates every 10 seconds, but not until the scan has run at least once
        if (distance.age() > 10 && distance.state() != Distance.AUTO_STATE.IDLE) {
            distance.startScan();
        }
        // Distance scan status and result
        telemetry.addData("Barcode", "P %s, A %.2f, S %s",
                distance.position(), distance.age(), distance.state());
    }

    @Override
    public void stop() {
        super.stop();
    }
}