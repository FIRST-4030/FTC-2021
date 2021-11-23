package org.firstinspires.ftc.teamcode.momm.sample;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.robot.Globals;

// Extend OpMode as usual
// If you must extend MultiOpModeManger be sure to @override all of the standard methods (or else)
//
// Register with @TeleOp or @Autonomous if you want to expose this OpMode independently
// It will still work as a sub-mode in MOMM even if it is not registered for independent use
//@TeleOp(name = "MOMM_Drive", group = "Test")
@Config
public class MOMM_Drive extends OpMode {
    // Config
    public static boolean DEBUG = false;
    private static final double LOW_SPEED_DEFAULT = 0.7;
    private static final int INPUT_SCALING_EXPONENT = 3;

    // Hardware
    private DcMotor driveLeft;
    private DcMotor driveRight;

    // Members
    private boolean enabled = false;
    private InputHandler in;
    private double LOW_SPEED_OFFSET = LOW_SPEED_DEFAULT;

    // Custom methods
    public void lowSpeed(double speed) {
        LOW_SPEED_OFFSET = speed;
    }

    // Standard methods
    @Override
    public void init() {
        // Pull in Globals
        telemetry = Globals.opmode(this).telemetry;
        in = Globals.input();

        // Drive wheels
        try {
            // TODO: Map to actual hardware
            driveLeft = hardwareMap.get(driveLeft.getClass(), "DL");
            driveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // TODO: Map to actual hardware
            driveRight = hardwareMap.get(driveRight.getClass(), "DR");
            driveRight.setDirection(DcMotorSimple.Direction.REVERSE);
            driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            in.register("DRIVE_FORWARD", GAMEPAD.driver1, PAD_KEY.left_stick_y);
            in.register("DRIVE_TURN", GAMEPAD.driver1, PAD_KEY.right_stick_x);
            in.register("DRIVE_SLOW", GAMEPAD.driver1, PAD_KEY.right_trigger);

            // Don't enable this OM unless we find the necessary hardware
            // This avoids null-pointer exceptions and allows other code
            // to run normally even while this OM fails
            //
            // Be sure to protect methods that use drive hardware by checking this flag
            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": Could not initialize");
        }
    }

    @Override
    public void start() {
        telemetry.log().add(getClass().getSimpleName() + ": start()");
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
}
