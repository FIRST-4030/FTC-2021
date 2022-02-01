package org.firstinspires.ftc.teamcode.TFODOHM.ODMain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Matrix4f;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Matrix4fBuilder;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Vector2f;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Vector3f;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "TFODModuleTest", group = "Test")
public class TFODModule extends OpMode {

    //Vuforia stuff for camera handling
    private static final String VUFORIA_KEY = "AV9rwXT/////AAABma+8TAirNkVYosxu9qv0Uz051FVEjKU+nkH+MaIvGuHMijrdgoZYBZwCW2aG8P3+eZecZZPq9UKsZiTHAg73h09NT48122Ui10c8DsPe0Tx5Af6VaBklR898w8xCTdOUa7AlBEOa4KfWX6zDngegeZT5hBLfJKE1tiDmYhJezVDlITIh7SHBv0xBvoQuXhemlzL/OmjrnLuWoKVVW0kLanImI7yra+L8eOCLLp1BBD/Iaq2irZCdvgziZPnMLeTUEO9XUbuW8txq9i51anvlwY8yvMXLvIenNC1xg4KFhMmFzZ8xnpx4nWZZtyRBxaDU99aXm7cQgkVP0VD/eBIDYN4AcB0/Pa7V376m6tRJ5UZh";
    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters vuforiaParameters;
    private static final String CAM_NAME = "MainCam";

    //init TF stuff for AI object detection
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private TFObjectDetector tfod;
    private TFObjectDetector.Parameters tfParameters;
    private List<Recognition> tfodRecognitions = new ArrayList<>();
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    //Object detection stuff
    private boolean busy = false;
    private ArrayList<Vector2f> bbCenter = new ArrayList<>();
    private int currentListSize = 0;
    private CameraLens camera = new CameraLens(CameraLens.C270_FOV);

    public TFODModule(){}

    @Override
    public void init() {
        try{
            initVuforia();
        } catch (Exception e) {
            telemetry.log().add("Vuforia Cannot Initialize!");
        }

        try{
            initTensorFlow();
        } catch (Exception e) {
            telemetry.log().add("TensorFlow Cannot Initialize!");
        }

        Matrix4f lensRot = Matrix4f.matMul(Matrix4f.matMul(Matrix4fBuilder.buildRotY(-8) ,Matrix4fBuilder.buildRotX(-45)), Matrix4fBuilder.buildRotZ(180));

        camera.setTranslation(new Vector3f(4.1f, 16.2f, -7.2f));
        camera.setRotation(lensRot);

        telemetry.log().add(camera.toString());
    }

    /**
     * Initializes TF for us to use
     * <p>This method inits: minResultConfidence; ModelTensorFlow2; inputSize;
     */
    public void initTensorFlow(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.tfParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfParameters.minResultConfidence = 0.69f;
        tfParameters.isModelTensorFlow2 = true;
        tfParameters.useObjectTracker = true;
        tfParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    /**
     * Initializes Vuforia for us to use
     * <p>This method gives the localizer the VUFORIA_KEY;
     */
    public void initVuforia(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, TFODModule.CAM_NAME);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        parameters.cameraMonitorFeedback = null;

    }

    @Override
    public void loop() {

    }

    public void scan(){
        busy = true;

        if (tfod != null){
            bbCenter.clear(); //clear

            tfodRecognitions = tfod.getRecognitions();
            currentListSize = tfodRecognitions.size();
        }

        busy = false;
    }
}
