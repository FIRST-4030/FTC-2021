package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "InputHandlerTest", group = "Test")
public class InputHandlerTest extends OpMode {
    // Class under test
    private InputHandler in;

    // Timestamps and last values
    private long active;
    private boolean lastActive;
    private long down;
    private boolean lastDown;
    private long up;
    private boolean lastUp;
    private long changed;
    private boolean lastChanged;
    private long auto;
    private boolean lastAuto;
    private long toggle;
    private boolean lastToggle;
    private long held;
    private boolean lastHeld;
    private long analog;
    private float lastAnalog;

    @Override
    public void init() {
        in = new InputHandler(this);
        in.register("BINARY", gamepad1, PAD_KEY.a);
        in.register("ANALOG", gamepad1, PAD_KEY.left_stick_x);

        telemetry.clearAll();
    }

    @Override
    public void init_loop() {
        telemetry.addData(getClass().getSimpleName(), "Ready");
        telemetry.addData("BINARY", in.active("BINARY"));
        telemetry.addData("ANALOG", in.value("ANALOG"));
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        // Copy the current time in so we don't make repeated system calls
        long now = System.currentTimeMillis();

        // Repeated calls to in.down() and friends are guaranteed to return the same value
        // This means you don't have to copy the value into your local context
        // Calling InputHandler.loop() may change the values, and should be done first
        in.loop();

        // Collect timestamps for changes
        if (in.active("BINARY") != lastActive) {
            active = now;
        }
        lastActive = in.active("BINARY");
        if (in.down("BINARY") != lastDown) {
            down = now;
        }
        lastDown = in.down("BINARY");
        if (in.up("BINARY") != lastUp) {
            up = now;
        }
        lastUp = in.up("BINARY");
        if (in.changed("BINARY") != lastChanged) {
            changed = now;
        }
        lastChanged = in.changed("BINARY");
        if (in.auto("BINARY") != lastAuto) {
            auto = now;
        }
        lastAuto = in.auto("BINARY");
        if (in.toggle("BINARY") != lastToggle) {
            toggle = now;
        }
        lastToggle = in.toggle("BINARY");
        if (in.held("BINARY") != lastHeld) {
            held = now;
        }
        lastHeld = in.held("BINARY");
        if (in.value("ANALOG") != lastAnalog) {
            analog = now;
        }
        lastAnalog = in.value("ANALOG");

        // Print the current value and the time of last change [relative to down(D) and now(N)]
        telemetry.addData("active", "%d: D %d, N %d",
                in.active("BINARY"), active - down, now - active);
        telemetry.addData("down", "%d: D %d, N %d",
                in.down("BINARY"), 0, now - down);
        telemetry.addData("up", "%d: D %d, N %d",
                in.up("BINARY"), up - down, now - up);
        telemetry.addData("changed", "%d: D %d, N %d",
                in.changed("BINARY"), changed - down, now - changed);
        telemetry.addData("auto", "%d: D %d, N %d",
                in.auto("BINARY"), auto - down, now - auto);
        telemetry.addData("toggle", "%d: D %d, N %d",
                in.toggle("BINARY"), toggle - down, now - toggle);
        telemetry.addData("held", "%d: D %d, N %d",
                in.held("BINARY"), held - down, now - held);
        telemetry.addData("analog", "%.2f: D %d, N %d",
                in.value("BINARY"), analog - down, now - analog);
        telemetry.update();
    }
}
