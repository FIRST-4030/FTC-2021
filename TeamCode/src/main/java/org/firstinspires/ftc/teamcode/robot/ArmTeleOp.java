package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.buttons.BUTTON_TYPE;
import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.utils.RateLimit;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp - Arm", group = "Arm")
public class ArmTeleOp extends OpMode {

    // Devices and subsystems
    private Robot robot = null;
    private ButtonHandler buttons;

    // arm consts
    private static final float ARM_MOVEMENT_SCALE = 1.0f/16;
    private static final float ARM_ROTATION_SCALE = 1.0f/512;
    private static final float WRIST_ROTATION_SCALE = 1.0f/512;
    private static final float ARM_HOME_X = 0.64f;
    private static final float ARM_HOME_Y = 3.81f;
    private static final float ARM_HOME_R = 0.4f;
    private static final float ARM_HOME_W = 0.8f;

    // Arm rate limiting
    private RateLimit rateX;
    private RateLimit rateY;
    private RateLimit rateR;
    private RateLimit rateW;
    private static final double MAX_ARM_RATE_X = 1.5d; // In inches per second
    private static final double MAX_ARM_RATE_Y = 1.5d; // In inches per second
    private static final double MAX_ARM_RATE_R = 0.125d; // In servo position per second
    private static final double MAX_ARM_RATE_W = 0.1d; // In servo position per second

    // other consts
    private static final float NORMAL_SPEED = 0.75f;
    private static final float SLOW_MODE = 0.25f;

    // vars
    private float armRotation = ARM_HOME_R;
    private float wristRotation = ARM_HOME_W;

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
        buttons.register("HOME_ARM", gamepad2, PAD_BUTTON.b, BUTTON_TYPE.SINGLE_PRESS);
        buttons.register("SLOW_MODE", gamepad1, PAD_BUTTON.b, BUTTON_TYPE.TOGGLE);
        buttons.register("SWIVEL_FORWARD", gamepad2, PAD_BUTTON.left_bumper, BUTTON_TYPE.SINGLE_PRESS);
        buttons.register("SWIVEL_SIDEWAYS", gamepad2, PAD_BUTTON.left_bumper, BUTTON_TYPE.SINGLE_PRESS);

        // Init rate limits for the arm
        rateX = new RateLimit(this, MAX_ARM_RATE_X);
        rateY = new RateLimit(this, MAX_ARM_RATE_Y);
        rateR = new RateLimit(this, MAX_ARM_RATE_R);
        rateW = new RateLimit(this, MAX_ARM_RATE_W);

        // Move arm to home
        robot.common.arm.setPosition(ARM_HOME_X, ARM_HOME_Y);
        robot.rotation.setPosition(ARM_HOME_R);
        robot.wrist.setPosition(wristRotation);

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
        if (buttons.get("SLOW_MODE")) {
            robot.wheels.setSpeedScale(SLOW_MODE);
        } else {
            robot.wheels.setSpeedScale(NORMAL_SPEED);
        }
        robot.wheels.loop(gamepad1);
    }

    private void auxiliary() {
        // arm movement
        float dx = (float) rateX.update(-gamepad2.left_stick_y * ARM_MOVEMENT_SCALE);
        float dy = (float) rateY.update(-gamepad2.right_stick_y * ARM_MOVEMENT_SCALE);
        armRotation -= (float) rateR.update(Math.pow(gamepad2.left_stick_x, 3) * ARM_ROTATION_SCALE);
        wristRotation -= (float) rateW.update(Math.pow(gamepad2.right_stick_x, 3) * WRIST_ROTATION_SCALE);

        // cap values
        armRotation = Math.min(1.0f, armRotation);
        armRotation = Math.max(0.0f, armRotation);
        wristRotation = Math.min(1.0f, wristRotation);
        wristRotation = Math.max(0.0f, wristRotation);

        robot.common.arm.setPositionDelta(dx, dy);

        if (buttons.get("HOME_ARM")) {
            robot.common.arm.setPosition(ARM_HOME_X, ARM_HOME_Y);
            armRotation = ARM_HOME_R;
            wristRotation = ARM_HOME_W;
        }

        robot.rotation.setPosition(armRotation);
        robot.wrist.setPosition(wristRotation);

        telemetry.addData("arm x", robot.common.arm.getArmX());
        telemetry.addData("arm y", robot.common.arm.getArmY());
        telemetry.addData("arm rotation", armRotation);
        telemetry.addData("wrist angle", wristRotation);
        telemetry.addData("lower", robot.lower.getPosition());
        telemetry.addData("upper", robot.upper.getPosition());

        // claw
        if (buttons.get("CLAW")) {
            robot.common.claw.close();
        } else {
            robot.common.claw.open();
        }
    }

    public void stop() {
    }
}