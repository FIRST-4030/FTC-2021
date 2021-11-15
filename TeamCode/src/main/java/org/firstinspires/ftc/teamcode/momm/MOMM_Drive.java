package org.firstinspires.ftc.teamcode.momm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
public class MOMM_Drive extends MultiOpModeManager {
    // Hardware
    private static DcMotor driveLeft;
    private static DcMotor driveRight;

    // Members
    private double DRIVE_POWER = 1.0;

    // Custom methods
    public void power(double power) {
        DRIVE_POWER = power;
    }

    // Standard methods
    @Override
    public void init() {
        // Drive wheels
        try {
            driveLeft = hardwareMap.get(DcMotor.class, "DL");
            driveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            driveRight = hardwareMap.get(DcMotor.class, "DR");
            driveRight.setDirection(DcMotorSimple.Direction.REVERSE);
            driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.log().add("Could not initialize drive");
        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.log().add(getClass().getSimpleName() + ": start()");
    }

    @Override
    public void loop() {
        // PoV drive
        double drive = Math.pow(-gamepad1.left_stick_y, 3);
        double turn = gamepad1.right_stick_x;
        driveLeft.setPower(Math.pow(-gamepad1.left_stick_y, 3));
        driveRight.setPower(Math.pow(-gamepad1.right_stick_y, 3));

        if (gamepad1.right_trigger != 0) {
            double speed = 1 - gamepad1.right_trigger;
            if (gamepad1.right_trigger >= 0.7) {
                speed = 1 - 0.7;
            }
            driveLeft.setPower(Range.clip(drive + turn, -speed, speed));
            driveRight.setPower(Range.clip(drive - turn, -speed, speed));
        }
    }

    @Override
    public void stop() {
        // Safely stop the drive motors
        driveLeft.setPower(0);
        driveRight.setPower(0);
    }
}