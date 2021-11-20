package org.firstinspires.ftc.teamcode.robot;

import android.database.StaleDataException;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;

@TeleOp(name = "MOMM_Teleop", group = "MOMM")
public class MOMM_Teleop extends MultiOpModeManager {
    private DuckSpin duck;
    private Distance distance;

    @Override
    public void init() {
        super.register(new Drive());
        super.register(new Depositor());
        super.register(new Capstone());
        super.register(new Collector());

        duck = new DuckSpin();
        super.register(duck);

        distance = new Distance();
        super.register(distance);

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

        // Trigger a distance scan, but only once
        if (gamepad2.guide && distance.state() == Distance.AUTO_STATE.IDLE) {
            distance.startScan();
        }
        // Distance scan status or result, when available
        if (distance.state() == Distance.AUTO_STATE.DONE) {
            telemetry.addData("Barcode", distance.position());
        } else {
            telemetry.addData("Barcode", distance.state());
        }

        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}