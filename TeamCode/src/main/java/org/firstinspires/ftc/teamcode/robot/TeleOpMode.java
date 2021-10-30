package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.buttons.BUTTON_TYPE;
import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.robot.auto.NewAuto;
import org.firstinspires.ftc.teamcode.robot.auto.RingStackTF;
import org.firstinspires.ftc.teamcode.wheels.MOTOR_END;
import org.firstinspires.ftc.teamcode.wheels.MOTOR_SIDE;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Prod")
public class TeleOpMode extends LinearOpMode implements RobotConstants {

    // Devices and subsystems
    private Robot robot = null;
    private ButtonHandler buttons;

    private boolean controlLocked = false;

    // other constants
    private static final float NORMAL_SPEED = 1.0f;
    private static final float SLOW_MODE = 0.35f;

    private NewAuto auto;


    @Override
    public void runOpMode() {

        controlLocked = false;
        // Placate drivers
        telemetry.addData(">", "NOT READY");
        telemetry.update();

        // Init the common tasks elements
        robot = new Robot(hardwareMap, telemetry);
        robot.wheels.setTeleop(true);
        //auto = new NewAuto(robot.wheels.getMotor(MOTOR_SIDE.LEFT, MOTOR_END.FRONT),robot.wheels.getMotor(MOTOR_SIDE.RIGHT, MOTOR_END.FRONT), hardwareMap, robot.odometry);

        // Check robot
        if (robot.bot != BOT.PRODUCTION) {
            telemetry.log().add("Opmode not compatible with bot " + robot.bot);
            requestOpModeStop();
            return;
        }

        // Register buttons
        buttons = new ButtonHandler(robot);
        //buttons.register("UPDATE_ODOMETRY", gamepad1, PAD_BUTTON.x);

        // Wait for the game to begin
        telemetry.addData(">", "Ready for game start");
        telemetry.update();

        waitForStart();

        //@Override
        while (opModeIsActive()) {
            // Update buttons
            buttons.update();

            // Move the robot


            if (!controlLocked) {
                driveBase();
                auxiliary();
            }

            telemetry.update();
            idle();
        }
    }

    /*@Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }*/


    //Slow mode
    private void driveBase() {
        //robot.odometry.update();
        /* if (buttons.get("SLOW_MODE")) {
            robot.wheels.setSpeedScale(SLOW_MODE);
        } else {
            robot.wheels.setSpeedScale(NORMAL_SPEED);
        } */
        robot.wheels.loop(gamepad1);
    }

    //telemetry data showed on phone during OpMode
    private void auxiliary() {
        //telemetry.addData("H:", robot.odometry.getPoseEstimate().getHeading());
        //telemetry.addData("S1:", robot.odometry.getS1());
        //telemetry.addData("S2:", robot.odometry.getS2());
        //telemetry.addData("L:", robot.odometry.getLeftEncoder());
        //telemetry.addData("R:", robot.odometry.getRightEncoder());
        //telemetry.addData("X:", robot.odometry.getPoseEstimate().getX());
        //telemetry.addData("Y:", robot.odometry.getPoseEstimate().getY());

    }



    //public void stop() {
    //}
}