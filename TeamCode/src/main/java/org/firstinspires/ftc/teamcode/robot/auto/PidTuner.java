package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.driveto.AutoDriver;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;
import org.firstinspires.ftc.teamcode.vuforia.VuforiaFTC;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Pid", group = "Production")
public class PidTuner extends LinearOpMode {


    // Devices and subsystems
    private Robot robot = null;
    private VuforiaFTC vuforia = null;
    private ButtonHandler buttons;
    private NewAuto auto;
    private double increment = 0.1;
    // Runtime vars
    private boolean gameReady = false;
    private Field.AllianceColor color = Field.AllianceColor.BLUE;
    private boolean stopByWall = true;

    private PIDFCoefficients newPIDF;
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

        // Init the camera system
        //vuforia.start();
        //vuforia.enableCapture();

        auto = new NewAuto("BL", "FR", hardwareMap, robot.odometry);

        newPIDF = auto.getPIDFCoefficients();

        buttons = new ButtonHandler(robot);
        //buttons.register("SELECT_PID", gamepad1, PAD_BUTTON.y);
        buttons.register("GO", gamepad1, PAD_BUTTON.y);
        buttons.register("TURN", gamepad1, PAD_BUTTON.b);
        buttons.register("BACK", gamepad1, PAD_BUTTON.a);
        buttons.register("TURN_CC", gamepad1, PAD_BUTTON.x);
        buttons.register("UP", gamepad1, PAD_BUTTON.dpad_up);
        buttons.register("DOWN", gamepad1, PAD_BUTTON.dpad_down);
        buttons.register("INC_DOWN", gamepad1, PAD_BUTTON.dpad_left);
        buttons.register("INC_UP", gamepad1, PAD_BUTTON.dpad_right);


        // Process driver input
        while (!robot.gyro.isReady() && opModeIsActive()) {
            // Overall ready status
            gameReady = (robot.gyro.isReady());
            telemetry.addLine(gameReady ? "READY" : "NOT READY");

            // Detailed feedback
            telemetry.addData("Gyro", robot.gyro.isReady() ? "Ready" : "Calibrating…");

            // Update
            telemetry.update();
        }

        waitForStart();
        telemetry.clearAll();

        // Log if we didn't exit init as expected
        if (!gameReady) {
            telemetry.log().add("! STARTED BEFORE READY !");
        }


        while(opModeIsActive()){
            buttons.update();

            if (buttons.get("UP")) {
                //auto.TICKS_PER_DEG += 0.1;
            }


            if (buttons.get("DOWN")) {
                //auto.TICKS_PER_DEG -= 0.1;
            }
            if(buttons.get("GO")) auto.drive(48, 1.0f);
            if(buttons.get("TURN")) auto.rotate(180, 1.0f);
            if(buttons.get("BACK")) auto.drive(-48, 1.0f);
            if(buttons.get("TURN_CC")) auto.rotate(-180, 1.0f);
            //telemetry.addData("D", (robot.odometry.getLeftEncoder()-robot.odometry.getRightEncoder())/(Math.PI*10));
            //telemetry.addData("TPD", auto.TICKS_PER_DEG);

            telemetry.update();


        }


    }

}
