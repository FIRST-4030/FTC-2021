package org.firstinspires.ftc.teamcode.momm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.Vector;

public class MultiOpModeManager extends OpMode {

    private final Vector<OpMode> opmodes;
    private final OpMode primary;

    // Init the list and register the primary OM
    public MultiOpModeManager() {
        opmodes = new Vector<>();
        primary = this;
    }

    // Add new OMs for concurrent execution
    public void register(OpMode opMode) {
        if (opMode == null) {
            throw new NullPointerException(getClass().getSimpleName() + ": " +
                    "OpMode not specified");
        }
        if (opmodes.contains(opMode)) {
            throw new IllegalArgumentException(getClass().getSimpleName() + ": " +
                    "OpMode already registered: " + opMode.getClass().getSimpleName());
        }
        if (opMode.equals(primary)) {
            throw new IllegalArgumentException(getClass().getSimpleName() + ": " +
                    "Refusing to re-register the primary OpMode: " + opMode.getClass().getSimpleName());
        }
        opmodes.add(opMode);
    }

    // Remove registered OMs
    public void deregister(OpMode opMode) {
        if (opMode == null) {
            throw new NullPointerException(getClass().getSimpleName() + ": " +
                    "OpMode not specified");
        }
        opmodes.remove(opMode);
    }

    // Provide access to the primary OM
    public OpMode opMode() {
        return primary;
    }

    /*
     * Run each of the standard OM methods for each registered OM
     */
    @Override
    public void init() {
        for (OpMode om : opmodes) {
            om.init();
        }
    }

    @Override
    public void init_loop() {
        for (OpMode om : opmodes) {
            om.init_loop();
        }
    }

    @Override
    public void start() {
        for (OpMode om : opmodes) {
            om.start();
        }
    }

    @Override
    public void loop() {
        for (OpMode om : opmodes) {
            om.loop();
        }
    }

    @Override
    public void stop() {
        for (OpMode om : opmodes) {
            om.stop();
        }
    }
}