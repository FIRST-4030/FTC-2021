package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.buttons.BUTTON_TYPE;
import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.vuforia.VuforiaFTC;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RunToPosTest", group = "Production")
public class RunToPosTest extends LinearOpMode implements RobotConstants {


    // Devices and subsystems
    private Robot robot = null;
    private VuforiaFTC vuforia = null;
    private ButtonHandler buttons;
    private NewAuto auto;
    // Runtime vars
    private boolean gameReady = false;
    private Field.AllianceColor color = Field.AllianceColor.BLUE;
    private int selectedPid = 0;

    public void runOpMode() {
        telemetry.addLine("Init…");
        telemetry.update();


        // Init the common tasks elements
        robot = new Robot(hardwareMap, telemetry);
        vuforia = robot.vuforia;

        // Check robot
        if (robot.bot != BOT.PRODUCTION) {
            telemetry.log().add("Opmode not compatible with bot " + robot.bot);
            requestOpModeStop();
        }

        // Init the camera system (if you need to)
        //vuforia.start();
        //vuforia.enableCapture();

        //Replace BL and FR with whichever motors run auto.
        auto = new NewAuto("BL","FR", hardwareMap, robot.odometry);

        // Register buttons
        buttons = new ButtonHandler(robot);
        buttons.register("EXAMPLE_BUTTON", gamepad1, PAD_BUTTON.a, BUTTON_TYPE.SINGLE_PRESS);

        //REMEMBER TO INIT SERVOS HERE
        //robot.servo.setPosition(servoInitPos);

        // Process driver input
        while(!robot.gyro.isReady() && opModeIsActive() && !isStarted() && !isStopRequested()) {
            // Overall ready status
            gameReady = (robot.gyro.isReady());
            telemetry.addLine(gameReady ? "READY" : "NOT READY");

            // Detailed feedback
            telemetry.addData("Gyro", robot.gyro.isReady() ? "Ready" : "Calibrating…");

            // Update
            telemetry.update();
        }

        while(!isStopRequested() && !isStarted()) userSettings();

        waitForStart();

        // STUFF STARTS HAPPENING

        telemetry.clearAll();

        // Log if we didn't exit init as expected
        if (!gameReady) {
            telemetry.log().add("! STARTED BEFORE READY !");
        }
        telemetry.update();

        if(!isStopRequested()) {
            /*
            AUTO ROUTINE HERE
            */
            auto.driveToPosition(24, 0, 1);
            telemetry.addData("atan2", auto.ATAN2);
            telemetry.update();
            sleep(5000);
            auto.driveToPosition(24, 24, 1);
            telemetry.addData("atan2", auto.ATAN2);
            telemetry.addData("X", robot.odometry.getPoseEstimate().getX());
            telemetry.addData("Y", robot.odometry.getPoseEstimate().getY());
            telemetry.update();
            sleep(5000);
            auto.driveToPosition(48, 24, 1);
            telemetry.addData("atan2", auto.ATAN2);
            telemetry.addData("X", robot.odometry.getPoseEstimate().getX());
            telemetry.addData("Y", robot.odometry.getPoseEstimate().getY());
            telemetry.update();
            sleep(5000);
            auto.driveToPosition(72, 0, 1);
            telemetry.addData("atan2", auto.ATAN2);
            telemetry.addData("X", robot.odometry.getPoseEstimate().getX());
            telemetry.addData("Y", robot.odometry.getPoseEstimate().getY());
            telemetry.update();
            sleep(5000);
        }

    }


    /**
     * Sets config booleans according to user input
     */
    private void userSettings(){



    }

}
