package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.buttons.BUTTON_TYPE;
import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ServoPosFinder", group = "Test")
public class ServoPositionFinder extends OpMode {

    // Devices and subsystems
    private Robot robot = null;
    private ButtonHandler buttons;

    private float servoPos = 0.4f;


    //servo constants
    private static final float INCREMENT = 0.01f;

    // other consts
    private static final float NORMAL_SPEED = 0.75f;
    private static final float SLOW_MODE = 0.25f;



    @Override
    public void init() {
        // Placate drivers
        telemetry.addData(">", "NOT READY");
        telemetry.update();

        // Init the common tasks elements
        robot = new Robot(hardwareMap, telemetry);
        robot.wheels.setTeleop(true);

        // Check robot
        if (robot.bot != BOT.PRODUCTION) {
            telemetry.log().add("Opmode not compatible with bot " + robot.bot);
            requestOpModeStop();
            return;
        }

        // Register buttons
        buttons = new ButtonHandler(robot);
        buttons.register("SERVO_UP", gamepad1, PAD_BUTTON.dpad_up, BUTTON_TYPE.SINGLE_PRESS);
        buttons.register("SERVO_DOWN", gamepad1, PAD_BUTTON.dpad_down, BUTTON_TYPE.SINGLE_PRESS);


        // Wait for the game to begin
        telemetry.addData(">", "Ready for game start");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Update buttons
        buttons.update();

        // Move the robot
        auxiliary();

        telemetry.update();
    }

    private void driveBase() {
        if (buttons.get("SLOW_MODE")) {
            robot.wheels.setSpeedScale(SLOW_MODE);
        } else {
            robot.wheels.setSpeedScale(NORMAL_SPEED);
        }
        robot.wheels.loop(gamepad1);
    }

    private void auxiliary() {
        // Shows number of servoPos
        telemetry.addData("Pos:", servoPos);
        //telemetry.addData("Y:", odometry.getPoseEstimate().getY());
        // Moving the servo position and number should increase
        if (buttons.get("SERVO_UP")) {
            servoPos += INCREMENT;
            servoPos = Math.min(1.0f, servoPos);
        // Moving the servo position and number should decrease
        } else if (buttons.get("SERVO_DOWN")) {
            servoPos -= INCREMENT;
            servoPos = Math.max(0.0f, servoPos);
        }
        // Set position of desired servo
        robot.wobbleGoalArm.setPosition(servoPos);
    }



    public void stop() {
    }
}