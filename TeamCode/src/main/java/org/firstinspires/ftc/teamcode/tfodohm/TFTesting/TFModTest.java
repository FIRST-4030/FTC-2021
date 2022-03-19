package org.firstinspires.ftc.teamcode.tfodohm.TFTesting;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tfodohm.ODMain.TFODModule;
import org.firstinspires.ftc.teamcode.utils.LoopUtil;

public class TFModTest extends LoopUtil {
    private static TFODModule tf;

    @Override
    public void opInit() {
        tf = new TFODModule();
        try{
            super.register(tf);
        } catch (Exception exception) {
            telemetry.addData("Error: ", tf.getData());
        }

        initOpModes();
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
