package org.firstinspires.ftc.teamcode.robot.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.actuators.Motor;
import org.firstinspires.ftc.teamcode.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.sensors.switches.Switch;
import org.firstinspires.ftc.teamcode.wheels.MOTOR_END;
import org.firstinspires.ftc.teamcode.wheels.MOTOR_SIDE;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Hardware Test", group = "Test")

public class HardwareTest extends OpMode {

    private static final float SERVO_INCREMENT = 0.05f;
    private static final String MOTOR_FWD = "_M_FWD";
    private static final String MOTOR_BACK = "_M_BACK";
    private static final String SERVO_FWD = "_S_FWD";
    private static final String SERVO_BACK = "_S_BACK";

    // Devices and subsystems
    private Robot robot = null;
    private ButtonHandler buttons = null;

    @Override
    public void init() {
        telemetry.addData(">", "Init…");
        telemetry.update();

        // Common init
        robot = new Robot(hardwareMap, telemetry);

        // Buttons
        buttons = new ButtonHandler(robot);
        buttons.register("ENCODER_RESET", gamepad1, PAD_BUTTON.guide);
        buttons.register("LOWER" + SERVO_FWD, gamepad1, PAD_BUTTON.dpad_right);
        buttons.register("LOWER" + SERVO_BACK, gamepad1, PAD_BUTTON.dpad_left);
        buttons.register("UPPER" + SERVO_FWD, gamepad1, PAD_BUTTON.dpad_up);
        buttons.register("UPPER" + SERVO_BACK, gamepad1, PAD_BUTTON.dpad_down);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Gyro", robot.gyro.isReady() ? "Ready" : "Calibrating…");
        if (robot.gyro.isReady()) {
            telemetry.addData(">", "Ready for game start");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        buttons.update();
/*

        telemetry.addData("BL", robot.wheels.getEncoder(MOTOR_SIDE.LEFT, MOTOR_END.BACK));
        telemetry.addData("FL", robot.wheels.getEncoder(MOTOR_SIDE.LEFT, MOTOR_END.FRONT));
        telemetry.addData("BR", robot.wheels.getEncoder(MOTOR_SIDE.RIGHT, MOTOR_END.BACK));
        telemetry.addData("FR", robot.wheels.getEncoder(MOTOR_SIDE.RIGHT, MOTOR_END.FRONT));
        robot.wheels.loop(gamepad1);

        if (buttons.get("ENCODER_RESET")) {
            robot.wheels.resetEncoder(MOTOR_SIDE.LEFT, MOTOR_END.BACK);
            robot.wheels.resetEncoder(MOTOR_SIDE.RIGHT, MOTOR_END.BACK);
            robot.wheels.resetEncoder(MOTOR_SIDE.LEFT, MOTOR_END.FRONT);
            robot.wheels.resetEncoder(MOTOR_SIDE.RIGHT, MOTOR_END.FRONT);
        }

 */
        updateServo("UPPER", robot.upper);
        updateServo("LOWER", robot.lower);

        telemetry.update();
    }

    @Override
    public void stop() {
    }

    private void updateSwitch(String name, Switch d) {
        telemetry.addData(name + " (D)", d.get());
    }

    private void updateMotor(String name, Motor motor) {
        float speed = 0;
        if (buttons.held(name + MOTOR_BACK)) {
            speed = -1;
        } else if (buttons.held(name + MOTOR_FWD)) {
            speed = 1;
        }
        motor.setPower(speed);
        telemetry.addData(name + " (M)", motor.getEncoder());
    }

    private void updateServo(String name, ServoFTC servo) {
        float pos = servo.getPosition();
        if (buttons.get(name + SERVO_BACK)) {
            pos -= SERVO_INCREMENT;
        } else if (buttons.get(name + SERVO_FWD)) {
            pos += SERVO_INCREMENT;
        }
        servo.setPositionRaw(pos);
        telemetry.addData(name + " (S)", servo.getPosition());
    }
}