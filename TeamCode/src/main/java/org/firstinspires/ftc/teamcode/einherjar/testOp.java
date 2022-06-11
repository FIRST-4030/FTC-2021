package org.firstinspires.ftc.teamcode.einherjar;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.LoopUtil;

import java.util.Locale;


//@Config
//@TeleOp(name = "!GLADOS::TestOp", group = "!GLADOS")
public class testOp extends LoopUtil {

    private InputHandler controller1;
    private InputHandler controller2;
    private final String c1name = "C1_", c2name = "C2_";

    @Override
    public void opInit() {
        controller1 = new InputHandler(this);
        controller2 = new InputHandler(this);

        for (PAD_KEY key: PAD_KEY.values()) {
            controller1.register(c1name + key.name().toUpperCase(), GAMEPAD.driver1, key);
            controller2.register(c2name + key.name().toUpperCase(), GAMEPAD.driver2, key);
        }
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {

    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
