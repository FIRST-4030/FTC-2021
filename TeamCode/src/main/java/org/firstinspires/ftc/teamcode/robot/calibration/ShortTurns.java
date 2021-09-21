package org.firstinspires.ftc.teamcode.robot.calibration;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.buttons.BUTTON_TYPE;
import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;
import org.firstinspires.ftc.teamcode.calibration.Subsystem;
import org.firstinspires.ftc.teamcode.driveto.AutoDriver;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.common.Drive;
import org.firstinspires.ftc.teamcode.utils.Round;

public class ShortTurns extends Subsystem {
    private static final String P = "TURN_P";
    private static final String I = "TURN_I";
    private static final String D = "TURN_D";
    private static final String INCREMENT = "TURN-INCREMENT";
    private static final float MIN_INCREMENT = 0.0001f;
    private static final float MAX_INCREMENT = 0.1f;

    private static final String JOYSTICK = "JOYSTICK";
    private static final String CW = "PLUS_90";
    private static final String CCW = "MINUS_90";
    private static final String ANGLE_0 = "0";
    private static final String ANGLE_90 = "90";
    private static final String ANGLE_180 = "180";
    private static final String ANGLE_270 = "270";

    private static final String SMALL = "SMALL";
    private static final String LARGE = "LARGE";
    private static final String BACK = "BACK";

    private AutoDriver driver = new AutoDriver();

    public ShortTurns(OpMode opmode, Robot robot, ButtonHandler buttons) {
        super(opmode, robot, buttons);
    }

    public String name() {
        return this.getClass().getSimpleName();
    }

    protected void load() {
        // Value to increment by
        buttons.spinners.add(INCREMENT,
                opmode.gamepad1, PAD_BUTTON.right_bumper, PAD_BUTTON.left_bumper,
                MIN_INCREMENT, 0.0f);
        buttons.spinners.setLimit(INCREMENT, MIN_INCREMENT, false);
        buttons.spinners.setLimit(INCREMENT, MAX_INCREMENT, true);

        // P, I, and D values
        buttons.spinners.add(P,
                opmode.gamepad1, PAD_BUTTON.dpad_up, PAD_BUTTON.dpad_down,
                INCREMENT, Drive.SHORT_TURN_PARAMS.P);
        buttons.spinners.add(I,
                opmode.gamepad1, PAD_BUTTON.dpad_right, PAD_BUTTON.dpad_left,
                INCREMENT, Drive.SHORT_TURN_PARAMS.I);
        buttons.spinners.add(D,
                opmode.gamepad1, PAD_BUTTON.y, PAD_BUTTON.b,
                INCREMENT, Drive.SHORT_TURN_PARAMS.D);

        // Angles
        buttons.register(SMALL, opmode.gamepad1, PAD_BUTTON.a);
        buttons.register(LARGE, opmode.gamepad1, PAD_BUTTON.x);
        buttons.register(BACK, opmode.gamepad1, PAD_BUTTON.back, BUTTON_TYPE.TOGGLE);

        // Manual Control
        buttons.register(JOYSTICK, opmode.gamepad1, PAD_BUTTON.start, BUTTON_TYPE.TOGGLE);
    }

    protected void unload() {
        buttons.spinners.remove(P);
        buttons.spinners.remove(I);
        buttons.spinners.remove(D);
        buttons.spinners.remove(INCREMENT);

        buttons.deregister(SMALL);
        buttons.deregister(LARGE);
        buttons.deregister(BACK);

        buttons.deregister(JOYSTICK);
    }

    protected void update() {
        Drive.SHORT_TURN_PARAMS.P = buttons.spinners.getFloat(P);
        Drive.SHORT_TURN_PARAMS.I = buttons.spinners.getFloat(I);
        Drive.SHORT_TURN_PARAMS.D = buttons.spinners.getFloat(D);

        robot.telemetry.addData("Gyro", robot.gyro.isReady() ? Round.truncate(robot.gyro.getHeading()) : "<Calibrating>");

        // Handle AutoDriver driving
        driver = robot.common.drive.loop(driver);
        if (driver.isRunning(opmode.time)) {
            return;
        }

        // Allow joystick driving
        if (buttons.get(JOYSTICK)) {
            robot.wheels.loop(opmode.gamepad1);
        }

        // Allow driving backwards in auto
        float scale = 1;
        if (buttons.get(BACK)) {
            scale = -1;
        }

        // Process new AutoDriver commands
        if (buttons.get(LARGE)) {
            driver.drive = robot.common.drive.degrees(90 * scale);
        } else if (buttons.get(SMALL)) {
            driver.drive = robot.common.drive.degrees(45 * scale);
        }
    }

    @Override
    protected void stop() {
        if (driver != null) {
            driver.stop();
        }
    }
}
