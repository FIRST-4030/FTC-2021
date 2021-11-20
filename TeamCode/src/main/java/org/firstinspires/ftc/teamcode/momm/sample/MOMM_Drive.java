package org.firstinspires.ftc.teamcode.momm.sample;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

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
    private static DcMotor driveLeft;
    private static DcMotor driveRight;

    // Members
    private boolean enabled = false;
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

        // PoV drive
        double drive = Math.pow(-gamepad1.left_stick_y, INPUT_SCALING_EXPONENT);
        double turn = gamepad1.right_stick_x;
        driveLeft.setPower(Math.pow(-gamepad1.left_stick_y, INPUT_SCALING_EXPONENT));
        driveRight.setPower(Math.pow(-gamepad1.right_stick_y, INPUT_SCALING_EXPONENT));

        // Low-speed PoV drive
        if (gamepad1.right_trigger != 0) {
            double speed = 1 - gamepad1.right_trigger;
            if (gamepad1.right_trigger >= LOW_SPEED_OFFSET) {
                speed = 1 - LOW_SPEED_OFFSET;
            }
            driveLeft.setPower(Range.clip(drive + turn, -speed, speed));
            driveRight.setPower(Range.clip(drive - turn, -speed, speed));
        }

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Drive Input", "LY %.2f, RX %.2f",
                    gamepad1.left_stick_y, gamepad1.right_stick_x);
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
