package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.buttons.BUTTON_TYPE;
import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.utils.RateLimit;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp - Fancy Arm", group = "Arm")
public class FancyArmTeleOp extends OpMode {

    // Devices and subsystems
    private Robot robot = null;
    private ButtonHandler buttons;

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
        if (robot.bot != BOT.ARM) {
            telemetry.log().add("Opmode not compatible with bot " + robot.bot);
            requestOpModeStop();
            return;
        }

        // Register buttons
        buttons = new ButtonHandler(robot);
        buttons.register("CLAW", gamepad2, PAD_BUTTON.x, BUTTON_TYPE.TOGGLE);
        buttons.register("SLOW_MODE", gamepad1, PAD_BUTTON.b, BUTTON_TYPE.TOGGLE);
        buttons.register("HOME", gamepad2, PAD_BUTTON.b, BUTTON_TYPE.TOGGLE);

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
        driveBase();
        auxiliary();

        telemetry.update();
    }

    private void driveBase() {
        // Skip drivebase if the arm isn't homed
        if (!buttons.get("HOME")) return;
        if (buttons.get("SLOW_MODE")) {
            robot.wheels.setSpeedScale(SLOW_MODE);
        } else {
            robot.wheels.setSpeedScale(NORMAL_SPEED);
        }
        robot.wheels.loop(gamepad1);
    }

    private void auxiliary() {
        // Calculate arm positions
        float wrist = 1-((gamepad2.left_stick_y + 1)/2);
        float rotation = (gamepad2.right_stick_x + 1)/2;
        float upper = (gamepad2.right_trigger) * 0.76f;
        float lower = ((gamepad2.left_trigger) * 0.9f)+0.05f;

        // Check if we're homed
        if (buttons.get("HOME")) {
            // Magic numbers!!
            wrist = 0.44f;
            rotation = 0.422f;
            lower = 0.95f;
            upper = 0.34f;
        }

        // Move servos
        robot.wrist.setPosition(wrist);
        robot.rotation.setPosition(rotation);
        robot.upper.setPosition(upper);
        robot.lower.setPosition(lower);

        // the claaaww
        if (buttons.get("CLAW")) {
            robot.claw.min();
        } else {
            robot.claw.max();
        }

        telemetry.addData("upper", robot.upper.getPosition());
        telemetry.addData("lower", robot.lower.getPosition());
        telemetry.addData("rotation", robot.rotation.getPosition());
        telemetry.addData("wrist", robot.wrist.getPosition());
        telemetry.addData("claw", gamepad2.left_stick_x);

    }

    public void stop() {
    }
}