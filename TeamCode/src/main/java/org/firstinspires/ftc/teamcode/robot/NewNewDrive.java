package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.PiecewiseFunction;

@Config
public class NewNewDrive extends OpMode {
    // Config
    public static boolean DEBUG = false;
    private static final int INPUT_SCALING_EXPONENT = 3;
    public static double TICKS_PER_INCH = 43.24;
    public static double TURN_RATIO = 6.3;
    public static double ACCEL_CONSTANT = 0.4;
    public static double trackWidth = 15.25;
    public static double trackWidthHalf = trackWidth / 2.0;

    // Hardware
    private DcMotor driveLeft;
    private DcMotor driveRight;

    // Members
    private boolean enabled = false;
    private boolean auto = false;
    private InputHandler in;
    private ElapsedTime rampTimer = new ElapsedTime();
    private boolean started;
    private boolean done;
    private PiecewiseFunction speedCurve;

    // Standard methods
    @Override
    public void init() {
        // Pull in Globals
        in = Globals.input(this);

        // Drive wheels
        try {
            driveLeft = hardwareMap.get(DcMotor.class, "BL");
            driveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            driveRight = hardwareMap.get(DcMotor.class, "BR");
            driveRight.setDirection(DcMotorSimple.Direction.FORWARD);
            driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            in.register("DRIVE_LEFT", GAMEPAD.driver1, PAD_KEY.left_stick_y);
            in.register("DRIVE_RIGHT", GAMEPAD.driver1, PAD_KEY.right_stick_y);
            in.register("DRIVE_ACCEL", GAMEPAD.driver1, PAD_KEY.right_trigger);

            done = false;
            started = false;

            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": Could not initialize");
        }
    }

    @Override
    public void start() {
        done = false;
        started = false;
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }
        // Input
        in.loop();

        // Skip driver control while there's an active auto command
        if (auto) {
            if (!isBusy()) {
                auto = false;
            }
            return;
        }

        // Tank drive
        double fastFactor = ACCEL_CONSTANT;
        if (in.value("DRIVE_ACCEL") > ACCEL_CONSTANT) {
            fastFactor = in.value("DRIVE_ACCEL");
        }
        double LEFT_DRIVE_POW = Math.pow(-in.value("DRIVE_LEFT"), INPUT_SCALING_EXPONENT);
        double RIGHT_DRIVE_POW = Math.pow(-in.value("DRIVE_RIGHT"), INPUT_SCALING_EXPONENT);
        driveLeft.setPower(LEFT_DRIVE_POW * fastFactor);
        driveRight.setPower(RIGHT_DRIVE_POW * fastFactor);

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Drive Input", "L %.2f, R %.2f, A %.2f",
                    in.value("DRIVE_LEFT"), in.value("DRIVE_RIGHT"),
                    in.value("DRIVE_ACCEL"));
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
        return (driveLeft.getPower() != 0 || driveRight.getPower() != 0);
    }

    public boolean isDone() {
        return done;
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
        auto = true;
        driveLeft.setPower(speed);
        driveRight.setPower(speed);
    }

    public void turnTo(float speed, float angle) {
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
        auto = true;
        driveLeft.setPower(speed);
        driveRight.setPower(-speed);
    }


    // angle = angle of rotation, degrees
    // r - radius of rotation, inches
    // speedMin - minimum speed of the drive, 0 - 1
    // speedMax - maximum speed of the drive, 0 - 1
    public void arcTo(double angle, double r, double speedMin, double speedMax) {
        // it should be, but ensure that the radius is positive
        r = Math.abs(r);
        if (r < 5) r = 5.0;

        double speed = 0;
        double rampTime;
        double v = 40 * (speedMax * 4 + speedMin * 2) / 6; // inches per second
        double arcLength = Math.PI * (angle / 180.0) * r; // inches
        double arcLengthL;
        double arcLengthR;
        if (angle < 0) {    // if angle is negative, we are turning to the right
            // difference is signs on the trackWidth
            angle *= -1;
            arcLengthL = Math.PI * (angle / 180.0) * (r + trackWidth / 2.0);
            arcLengthR = Math.PI * (angle / 180.0) * (r - trackWidth / 2.0);
        } else {
            arcLengthL = Math.PI * (angle / 180.0) * (r - trackWidth / 2.0);
            arcLengthR = Math.PI * (angle / 180.0) * (r + trackWidth / 2.0);
        }
        double time = Math.abs(arcLength / v);
        double leftVel = v * arcLengthL / arcLength;
        double rightVel = v * arcLengthR / arcLength;
        double maxRatio = 1;

        if (!started) {
            rampTimer.reset();
            driveLeft.setPower(speedMin * (leftVel / v));
            driveRight.setPower(speedMin * (rightVel / v));

            // initialize speedCurve to have time be the X coordinate and motor speed be the Y coordinate
            // note that elements need to be added in ascending order of X
            speedCurve.setClampLimits(true);
            speedCurve.addElement(0.00 * time, speedMin);
            speedCurve.addElement(0.25 * time, speedMax);
            speedCurve.addElement(0.75 * time, speedMax);
            speedCurve.addElement(1.00 * time, speedMin);
            started = true;
            done = false;
        } else if (done) {
            driveLeft.setPower(0);
            driveRight.setPower(0);
        }

        if (isBusy() || !done) {
/*            if (rampTimer.seconds() <= (time * 0.25)) {
                if (speed != 0.0) speed = (speedMin + (rampTimer.seconds() / (time * 0.25)) * (speedMax - speedMin)) * Math.signum(speed);
                else              speed = (speedMin + (rampTimer.seconds() / (time * 0.25)) * (speedMax - speedMin));
            } else if (rampTimer.seconds() <= (time * 0.75)) {
                speed = speedMax;
            } else if (rampTimer.seconds() < time){
                if (speed != 0) speed = (speedMax + ((rampTimer.seconds() - (time * 0.75)) / (time * 0.25)) * (speedMin - speedMax)) * Math.signum(speed);
                else            speed = (speedMax + ((rampTimer.seconds() - (time * 0.75)) / (time * 0.25)) * (speedMin - speedMax));
            } else {
                speed = 0;
                done = true;
            }*/

            // speed is calculated using the curve defined above
            speed = speedCurve.getY(rampTimer.seconds());
            done = speedCurve.isClamped();

            maxRatio = Math.max(leftVel, rightVel) / v;
            driveLeft.setPower(speed * (leftVel / v) / maxRatio);
            driveRight.setPower(speed * (rightVel / v) / maxRatio);
        }
        telemetry.log().add(getClass().getSimpleName() + "::arcTo(): Motors in use");
        if (isBusy() || !done) telemetry.addData("max ratio", maxRatio);
        telemetry.addData("leftVel", leftVel);
        telemetry.addData("rightVel", rightVel);
        telemetry.addData("speed", speed);
        telemetry.addData("timer", rampTimer.seconds());
        telemetry.addData("time", time);
    }

    public void driveStop() {
        // Zero the drive encoders, and enable RUN_TO_POSITION
        driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeft.setTargetPosition(driveLeft.getCurrentPosition());
        driveLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLeft.setPower(0);

        driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRight.setTargetPosition(driveRight.getCurrentPosition());
        driveRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRight.setPower(0);
    }
}
