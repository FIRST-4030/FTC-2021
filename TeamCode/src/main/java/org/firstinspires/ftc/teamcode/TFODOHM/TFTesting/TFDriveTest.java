package org.firstinspires.ftc.teamcode.TFODOHM.TFTesting;

import org.firstinspires.ftc.teamcode.TFODOHM.ODMain.TFODModule;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.TFMathExtension;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Vector2f;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Vector3f;
import org.firstinspires.ftc.teamcode.momm.MultiOpModeManager;
import org.firstinspires.ftc.teamcode.robot.NewNewDrive;
import org.firstinspires.ftc.teamcode.robot.tfMathArcTest;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

public class TFDriveTest extends MultiOpModeManager {


    private TFODModule tfodModule;
    private NewNewDrive drive;

    private AUTO_STATE state = AUTO_STATE.DONE;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }
    private boolean scanned = false, sorted = false, calculated = false;
    private Vector2f tempV2, target;
    private Vector3f targetPreCasted;
    private double storedRadius, storedArcLength;
    @Override
    public void loop() {
        switch (state){
            case SCAN: //scan for objects
                if (!tfodModule.isBusy() && (scanned == false)) {
                    tfodModule.scan();
                    scanned = true;
                }
                if (!tfodModule.isBusy() && (sorted == false)){
                    tempV2 = tfodModule.sortCBBB();
                    sorted = true;
                }
                if (!tfodModule.isBusy() && (calculated == false)){
                    targetPreCasted = tfodModule.calcCoordinate(tempV2);
                }
                if (!tfodModule.isBusy() && (scanned && sorted && calculated)){
                    state = AUTO_STATE.RESET_VAR;
                }
                break;
            case RESET_VAR: //reset all variables used to check stuff in the previous case
                scanned = false;
                sorted = false;
                calculated = false;
                target = new Vector2f(targetPreCasted.getX(), targetPreCasted.getZ());
                double[] f = TFMathExtension.makeArcV1(target);
                storedRadius = f[0];
                storedArcLength = f[1];
                state = AUTO_STATE.START_MOVE;
                break;
            case START_MOVE:

                break;
            case REVERSE_MOVE:
                break;
            case DONE:
                break;
        }
    }

    @Override
    public void stop() {
        state = AUTO_STATE.DONE;
        super.stop();
    }

    enum AUTO_STATE implements OrderedEnum {
        SCAN,
        RESET_VAR,
        START_MOVE,
        REVERSE_MOVE,
        DONE;
        public TFDriveTest.AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}
