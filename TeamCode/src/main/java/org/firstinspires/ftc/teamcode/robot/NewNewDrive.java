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

import java.util.ArrayList;

@Config
public class NewNewDrive extends OpMode {
    // Config
    public static boolean DEBUG = false;
    private static final int INPUT_SCALING_EXPONENT = 3;
    public static double TICKS_PER_INCH = 43.24;
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
    private PiecewiseFunction speedCurveL;
    private PiecewiseFunction speedCurveR;
    private PiecewiseFunction testCurve;

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

            speedCurve = new PiecewiseFunction();
            speedCurveL = new PiecewiseFunction();
            speedCurveR = new PiecewiseFunction();
            testCurve = new PiecewiseFunction();

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

    // speedMin - minimum speed of the drive, (-1) - 1
    // speedMax - maximum speed of the drive, (-1) - 1
    // distance - drive distance, inches; positive
    public void driveTo(double speedMin, double speedMax, double distance) {
        if (distance == 0) {
            return;
        }

        double speed = 0;
        double v = 40 * (speedMax * 4 + speedMin * 2) / 6; // inches per second
        double time = Math.abs(distance / v);

        if ((isBusy() || !done) && speedCurve.isValid() && started) {
            // speed is calculated using the curve defined above
            speed = speedCurve.getY(rampTimer.seconds());
            done = speedCurve.isClamped();

            driveLeft.setPower(speed);
            driveRight.setPower(speed);
        }

        if (!started) {
            rampTimer.reset();
            driveLeft.setPower(speedMin);
            driveRight.setPower(speedMin);

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
            started = false;
        }

        telemetry.log().add(getClass().getSimpleName() + "::driveTo(): Motors in use");
        telemetry.addData("speed", speed);
        telemetry.addData("timer", rampTimer.seconds());
        telemetry.addData("time", time);
    }

    // angle = angle of rotation, degrees; positive is left, negative is right
    // speedMin - minimum speed of the drive, 0 - 1
    // speedMax - maximum speed of the drive, 0 - 1
    public void turnTo(double angle, double speedMin, double speedMax) {
        if (angle == 0) {
            return;
        }

        speedMin = Math.abs(speedMin);
        speedMax = Math.abs(speedMax);
        double speed = 0;
        double v = 40 * (speedMax * 4 + speedMin * 2) / 6; // inches per second
        double arcLength = Math.PI * (angle / 180.0) * trackWidthHalf; // inches

        // Don't allow new moves if we're still busy
        double time = Math.abs(arcLength / v);
        double leftVel;
        double rightVel;

        if (angle < 0) {
            leftVel = v;
            rightVel = -v;
        } else {
            leftVel = -v;
            rightVel = v;
        }

        if ((isBusy() || !done) && speedCurve.isValid() && started) {
            // speed is calculated using the curve defined above
            speed = speedCurve.getY(rampTimer.seconds());
            done = speedCurve.isClamped();

            driveLeft.setPower(speed * (leftVel / v));
            driveRight.setPower(speed * (rightVel / v));
        }

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
            started = false;
            speedCurve.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::turnTo(): Motors in use");
        telemetry.addData("leftVel", leftVel);
        telemetry.addData("rightVel", rightVel);
        telemetry.addData("speed", speed);
        telemetry.addData("timer", rampTimer.seconds());
        telemetry.addData("time", time);
    }


    // angle = angle of rotation, degrees; positive is left, negative is right
    // r - radius of rotation, inches
    // speedMin - minimum speed of the drive, (-1) - 1
    // speedMax - maximum speed of the drive, (-1) - 1
    public void arcTo(double angle, double r, double speedMin, double speedMax) {
        if (angle == 0) {
            return;
        }

        // it should be, but ensure that the radius is positive
        r = Math.abs(r);
        if (r < 5) r = 5.0;

        double speed = 0;
        double v; // inches per second
        if (!speedCurve.isValid()) v = 40 * speedMax;
        else {
            double totalLength = 0;
            for (int i = 1; i < speedCurve.getCoordSize() - 1; i++) {
                totalLength += (speedCurve.getElementX(i) - speedCurve.getElementX(i - 1)) *
                        (speedCurve.getElementY(i) + speedCurve.getElementY(i - 1)) / 2.0;
            }
            v = totalLength / time;
        }
        double arcLength = Math.PI * (angle / 180.0) * r; // inches
        double arcLengthL;
        double arcLengthR;
        if (angle < 0) {    // if angle is negative, we are turning to the right
            // difference is signs on the trackWidth
            angle *= -1;
            arcLengthL = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
        } else {
            arcLengthL = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
        }
        double time = Math.abs(arcLength / v);
        double leftVel = v * arcLengthL / arcLength;
        double rightVel = v * arcLengthR / arcLength;
        double maxRatio = 1;

        if ((isBusy() || !done) && speedCurve.isValid() && started) {
            // speed is calculated using the curve defined above
            speed = speedCurve.getY(rampTimer.seconds() / time);
            done = speedCurve.isClamped();

            maxRatio = Math.max(Math.abs(leftVel), Math.abs(rightVel)) / Math.abs(v);
            driveLeft.setPower(speed * (leftVel / v) / maxRatio);
            driveRight.setPower(speed * (rightVel / v) / maxRatio);
        }

        if (!started) {
            rampTimer.reset();
            driveLeft.setPower(speedMin * (leftVel / v));
            driveRight.setPower(speedMin * (rightVel / v));

            // initialize speedCurve to have time be the X coordinate and motor speed be the Y coordinate
            // note that elements need to be added in ascending order of X
            speedCurve.setClampLimits(true);
            speedCurve.addElement(0.00, speedMin);
            speedCurve.addElement(0.25, speedMax);
            speedCurve.addElement(0.75, speedMax);
            speedCurve.addElement(1.00, speedMin);

            started = true;
            done = false;
        } else if (done) {
            driveLeft.setPower(0);
            driveRight.setPower(0);
            started = false;
            speedCurve.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::arcTo(): Motors in use");
        if (isBusy() || !done) telemetry.addData("max ratio", maxRatio);
        telemetry.addData("leftVel", leftVel);
        telemetry.addData("rightVel", rightVel);
        telemetry.addData("speed", speed);
        telemetry.addData("timer", rampTimer.seconds());
        telemetry.addData("time", time);
    }

    // angle = angle of rotation, degrees; positive is left, negative is right
    // r - radius of rotation, inches
    // speedMin - minimum speed of the drive, (-1) - 1
    // speedMax - maximum speed of the drive, (-1) - 1
    public void arcToTicks(double angle, double r, double speedMin, double speedMax) {
        if (angle == 0) {
            return;
        }
        double arcLengthL;
        double arcLengthR;
        double leftTicks;
        double rightTicks;
        // it should be, but ensure that the radius is positive
        r = Math.abs(r);

        if (r < 5) r = 5.0;
        if (angle < 0) {    // if angle is negative, we are turning to the right
            // difference is signs on the trackWidth
            angle *= -1;
            arcLengthL = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
        } else {
            arcLengthL = Math.PI * (angle / 180.0) * (r - trackWidthHalf);
            arcLengthR = Math.PI * (angle / 180.0) * (r + trackWidthHalf);
        }
        leftTicks = arcLengthL * TICKS_PER_INCH;
        rightTicks = arcLengthR * TICKS_PER_INCH;

        double maxRatio = Math.max(leftTicks / rightTicks, leftTicks / rightTicks);
        if ((isBusy() || !done) && speedCurveL.isValid() && speedCurveR.isValid() && started) {
            // speed is calculated using the curve defined above
            driveLeft.setPower((leftTicks / rightTicks) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
            driveRight.setPower((rightTicks / leftTicks) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
            done = speedCurveL.isClamped() || speedCurveR.isClamped();
        }

        if (!started) {

            // initialize speedCurve to have motor ticks be the X coordinate and motor speed be the Y coordinate
            // note that elements need to be added in ascending order of X
            speedCurveL.setClampLimits(true);
            speedCurveR.setClampLimits(true);
            speedCurveL.addElement(0.00 * leftTicks, speedMin);
            speedCurveL.addElement(0.25 * leftTicks, speedMax);
            speedCurveL.addElement(0.75 * leftTicks, speedMax);
            speedCurveL.addElement(1.00 * leftTicks, speedMin);
            speedCurveR.addElement(0.00 * rightTicks, speedMin);
            speedCurveR.addElement(0.25 * rightTicks, speedMax);
            speedCurveR.addElement(0.75 * rightTicks, speedMax);
            speedCurveR.addElement(1.00 * rightTicks, speedMin);

            // This resets the encoder ticks to zero on both motors
            driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            started = true;
            done = false;
        } else if (done) {
            driveLeft.setPower(0);
            driveRight.setPower(0);
            started = false;
            speedCurveL.reset();
            speedCurveR.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::arcToTicks(): Motors in use");
        telemetry.addData("left ticks", driveLeft.getCurrentPosition());
        telemetry.addData("right ticks", driveRight.getCurrentPosition());
        telemetry.addData("left ticks", speedCurveL.getElementX(speedCurveL.getCoordSize() - 1));
        telemetry.addData("right ticks", speedCurveR.getElementX(speedCurveR.getCoordSize() - 1));
        telemetry.addData("leftVel", driveLeft.getPower());
        telemetry.addData("rightVel", driveRight.getPower());
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
