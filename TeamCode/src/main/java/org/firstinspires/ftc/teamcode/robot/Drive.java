package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;

@Config
//@TeleOp(name = "Drive", group = "Test")
public class Drive extends OpMode {
    // Config
    public static boolean DEBUG = false;
    public static final double LOW_SPEED_OFFSET = 0.7;
    private static final int INPUT_SCALING_EXPONENT = 3;
    public static double TICKS_PER_INCH = 43.24;
    public static double TURN_RATIO = 7;

    // Hardware
    private DcMotor driveLeft;
    private DcMotor driveRight;

    // Members
    private boolean enabled = false;
    private InputHandler in;

    // Standard methods
    @Override
    public void init() {
        // Pull in Globals
        telemetry = Globals.opmode(this).telemetry;
        gamepad1 = Globals.opmode(this).gamepad1;
        in = Globals.input();

        // Drive wheels
        try {
            driveLeft = hardwareMap.get(driveLeft.getClass(), "BL");
            driveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            driveRight = hardwareMap.get(driveRight.getClass(), "BR");
            driveRight.setDirection(DcMotorSimple.Direction.FORWARD);
            driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            in.register("DRIVE_FORWARD", GAMEPAD.driver1, PAD_KEY.left_stick_y);
            in.register("DRIVE_TURN", GAMEPAD.driver1, PAD_KEY.right_stick_x);
            in.register("DRIVE_SLOW", GAMEPAD.driver1, PAD_KEY.right_trigger);

            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": Could not initialize");
        }
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }
        // Input
        in.loop();

        // PoV drive
        double drive = Math.pow(-in.value("DRIVE_FORWARD"), INPUT_SCALING_EXPONENT);
        double turn = in.value("DRIVE_TURN");
        driveLeft.setPower(Math.pow(-in.value("DRIVE_FORWARD"), INPUT_SCALING_EXPONENT));
        driveRight.setPower(Math.pow(-gamepad1.right_stick_y, INPUT_SCALING_EXPONENT));

        // Low-speed PoV drive
        if (in.value("DRIVE_SLOW") != 0) {
            double speed = 1 - in.value("DRIVE_SLOW");
            if (in.value("DRIVE_SLOW") >= LOW_SPEED_OFFSET) {
                speed = 1 - LOW_SPEED_OFFSET;
            }
            driveLeft.setPower(Range.clip(drive + turn, -speed, speed));
            driveRight.setPower(Range.clip(drive - turn, -speed, speed));
        }

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Drive Input", "LY %.2f, RX %.2f, RT %.2f",
                    in.value("DRIVE_FORWARD"), in.value("DRIVE_TURN"),
                    in.value("DRIVE_SLOW"));
            telemetry.addData("Drive Output", "L %.2f/%d, R %.2f/%d",
                    driveLeft.getPower(), driveLeft.getCurrentPosition(),
                    driveRight.getPower(), driveRight.getCurrentPosition());
        }
    }

    @Override
    public void stop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        // Stop the drive motors
        driveLeft.setPower(0);
        driveRight.setPower(0);
    }

    // Custom methods
    public boolean isBusy() {
        return driveLeft.isBusy() || driveRight.isBusy();
    }

    public void driveTo(float speed, float distance) {
        // Don't allow new moves if we're still busy
        if (isBusy()) {
            telemetry.log().add(getClass().getSimpleName() + "::driveTo(): Motors in use");
            return;
        }

        // Set a target, translated from inches to encoder ticks
        int leftTarget = driveLeft.getCurrentPosition();
        int rightTarget = driveRight.getCurrentPosition();
        leftTarget += distance * TICKS_PER_INCH;
        rightTarget += distance * TICKS_PER_INCH;
        driveLeft.setTargetPosition(leftTarget);
        driveRight.setTargetPosition(rightTarget);

        // Start the motors
        driveLeft.setPower(speed);
        driveRight.setPower(speed);
    }

    public void turnTo(float speed, int angle) {
        // Don't allow new moves if we're still busy
        if (isBusy()) {
            telemetry.log().add(getClass().getSimpleName() + "::turnTo(): Motors in use");
            return;
        }

        // Fake turns using a distance translation
        // We have a gyro but let's start with just one control mode
        int leftTarget = driveLeft.getCurrentPosition();
        int rightTarget = driveRight.getCurrentPosition();
        leftTarget += angle * TURN_RATIO;
        rightTarget -= angle * TURN_RATIO;
        driveLeft.setTargetPosition(leftTarget);
        driveRight.setTargetPosition(rightTarget);

        // Start the motors
        driveLeft.setPower(speed);
        driveRight.setPower(-speed);
    }
}
