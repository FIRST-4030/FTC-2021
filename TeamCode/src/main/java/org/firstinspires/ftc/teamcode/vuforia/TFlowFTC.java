package org.firstinspires.ftc.teamcode.vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Available;

import java.util.List;

public class TFlowFTC implements Available {
    private TFObjectDetector tflow;
    private Robot robot;

    private final static double CONFIDENCE_DEFAULT = 0.75;

    public TFlowFTC(Robot robot) {
        if (robot == null) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Robot not available");
        }
        if (robot.telemetry == null) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Telemetry not available");
        }
        if (robot.map == null) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": HardwareMap not available");
        }
        if (!robot.vuforia.isAvailable()) {
            robot.telemetry.log().add(this.getClass().getSimpleName() + ": Vuforia not available");
            return;
        }

        this.robot = robot;
    }

    public void init() {
        init(CONFIDENCE_DEFAULT);
    }

    public void init(double confidence) {
        int tfodMonitorViewId = robot.map.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robot.map.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        //Change the minimum confidence - default is .4
        tfodParameters.minimumConfidence = confidence;

        // Connect to the camera
        tflow = ClassFactory.getInstance().
                createTFObjectDetector(tfodParameters, robot.vuforia.vuforia);
    }

    public boolean isAvailable() {
        return (tflow != null);
    }

    public void start(String assetName, String... labels) {
        if (!isAvailable()) {
            return;
        }
        stop();

        tflow.loadModelFromAsset(assetName, labels);
        tflow.activate();
    }

    public void stop() {
        if (!isAvailable()) {
            return;
        }
        tflow.deactivate();
        tflow.shutdown();
    }

    public List<Recognition> getRecognitions() {
        return tflow.getUpdatedRecognitions();
    }
}
