package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.PiecewiseFunction;

@Config
public class NewNewDrive extends OpMode {
    // Config
    public static boolean DEBUG = false;
    private static final int INPUT_SCALING_EXPONENT = 3;
    private static final double MAX_VELOCITY = 40.0; // inches per second
    public static double TICKS_PER_INCH = 40.75;
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
    final private ElapsedTime rampTimer = new ElapsedTime();
    private boolean started;
    private boolean done;
    private boolean loggingEnabled = false;
    private PiecewiseFunction speedCurve;
    private PiecewiseFunction speedCurveL;
    private PiecewiseFunction speedCurveR;

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
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": Could not initialize");
        }
        loggingEnabled = false;
        logDataInit();
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

    public boolean isLoggingEnabled() {
        return loggingEnabled;
    }

    public void enableLogging() {
        loggingEnabled = true;
    }

    public void disableLogging() {
        loggingEnabled = false;
    }

    /**
     * Initialize the headers for all of the logged data
     * Note that this does NOT add headers for any function-specific data
     */
    private void logDataInit() {
        RobotLog.d("");
        RobotLog.d(",Drive Function,Time (s),Left Position (in),Right Position (in),Left Velocity (in/s),Right Velocity (in/s)");
    }

    /**
     * Log the basic drive data to a text file. Also log the "additionalData" string;
     */
    private void logData(String functionName, String additionalData) {
        if (loggingEnabled)
            RobotLog.d("," + functionName + "," + getRuntime() + "," +
                    driveLeft.getCurrentPosition() / TICKS_PER_INCH + "," + driveRight.getCurrentPosition() / TICKS_PER_INCH + "," +
                    driveLeft.getPower() * MAX_VELOCITY + "," + driveRight.getPower() * MAX_VELOCITY + "," +
                    additionalData);
    }

    // speedMin - minimum speed of the drive, (-1) - 1
    // speedMax - maximum speed of the drive, (-1) - 1
    // distance - drive distance, inches; positive
    public void driveTo(double speedMin, double speedMax, double distance) {
        if (distance == 0) {
            return;
        }
        double midTicks = distance * TICKS_PER_INCH;

        if ((isBusy() || !done) && speedCurveL.isValid() && speedCurveR.isValid() && started) {
            // speed is calculated using the curve defined above
            driveLeft.setPower(speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
            driveRight.setPower(speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
            done = speedCurveL.isClamped() && speedCurveR.isClamped();
        }

        if (!started) {
            // initialize speedCurveL and speedCurveR to have motor ticks be the X coordinate and motor speed be the Y coordinate
            speedCurveL.setClampLimits(true);
            speedCurveL.addElement(driveLeft.getCurrentPosition() - 0.05 * midTicks, speedMin);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + 0.375 * midTicks, speedMax);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + 0.625 * midTicks, speedMax);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + 1.00 * midTicks, speedMin);
            speedCurveR.setClampLimits(true);
            speedCurveR.addElement(driveRight.getCurrentPosition() - 0.05 * midTicks, speedMin);
            speedCurveR.addElement(driveRight.getCurrentPosition() + 0.375 * midTicks, speedMax);
            speedCurveR.addElement(driveRight.getCurrentPosition() + 0.625 * midTicks, speedMax);
            speedCurveR.addElement(driveRight.getCurrentPosition() + 1.00 * midTicks, speedMin);
            started = true;
            done = false;
        } else if (done) {
            driveLeft.setPower(0);
            driveRight.setPower(0);
            started = false;
            speedCurveL.reset();
            speedCurveR.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::driveTo(): Motors in use");
        telemetry.addData("left ticks", driveLeft.getCurrentPosition());
        telemetry.addData("right ticks", driveRight.getCurrentPosition());
        telemetry.addData("leftVel", driveLeft.getPower());
        telemetry.addData("rightVel", driveRight.getPower());
        logData("driveTo()", started + "," + done + "," + speedCurveL.isValid() + "," + speedCurveL.getSize() + "," + speedCurveR.isValid() + "," + speedCurveR.getSize());
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
        double v; // inches per second
        if (!speedCurve.isValid()) v = 40.0 * speedMax;
        else v = 40.0 * speedCurve.getAverage();
        double arcLength = Math.PI * (angle / 180.0) * trackWidthHalf; // inches
        double arcLengthL;
        double arcLengthR;
        if (angle < 0) {    // if angle is negative, we are turning to the right
            // difference is signs on the trackWidth
            angle *= -1;
            arcLengthL = Math.PI * (angle / 180.0) * trackWidthHalf;
            arcLengthR = Math.PI * (angle / 180.0) * -trackWidthHalf;
        } else {
            arcLengthL = Math.PI * (angle / 180.0) * -trackWidthHalf;
            arcLengthR = Math.PI * (angle / 180.0) * trackWidthHalf;
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
        logData("turnTo()", "");
    }

    // angle = angle of rotation, degrees; positive is left, negative is right
    // r - radius of rotation, inches
    // speedMin - minimum speed of the drive, (-1) - 1
    // speedMax - maximum speed of the drive, (-1) - 1
    public void arcTo(double r, double arcLength, double speedMin, double speedMax) {
        if (arcLength == 0) {
            return;
        }
        double arcLengthL;
        double arcLengthR;
        double arcLengthInner;
        double arcLengthOuter;
        double leftTicks;
        double rightTicks;
        // it should be, but ensure that the radius is positive
        double angle = 0;

        if (r == 0) {
            arcLengthL = arcLength;
            arcLengthR = arcLength;
        } else {
            angle = Math.toDegrees(arcLength / Math.abs(r));
            arcLengthInner = Math.toRadians(angle) * (Math.abs(r) - trackWidthHalf);
            arcLengthOuter = Math.toRadians(angle) * (Math.abs(r) + trackWidthHalf);

            if (r > 0) {
                // if radius is greater than zero, we are moving to the left, so the right side is on the outside
                arcLengthL = arcLengthInner;
                arcLengthR = arcLengthOuter;
            } else {
                // if radius is greater than zero, we are moving to the left, so the right side is on the inside
                arcLengthL = arcLengthOuter;
                arcLengthR = arcLengthInner;
            }
        }

        leftTicks = arcLengthL * TICKS_PER_INCH;
        rightTicks = arcLengthR * TICKS_PER_INCH;
        double midTicks = arcLength * TICKS_PER_INCH;
        if (midTicks == 0) {
            midTicks = 1;
        }

        double maxRatio = Math.max(Math.abs(leftTicks), Math.abs(rightTicks)) / Math.abs(midTicks);
        if ((isBusy() || !done) && speedCurveL.isValid() && speedCurveR.isValid() && started) {
            // speed is calculated using the curve defined above
            driveLeft.setPower((leftTicks / midTicks) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
            driveRight.setPower((rightTicks / midTicks) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
            done = speedCurveL.isClamped() && speedCurveR.isClamped();
        }

        if (!started) {
            // initialize speedCurve to have motor ticks be the X coordinate and motor speed be the Y coordinate
            speedCurveL.setClampLimits(true);
            speedCurveR.setClampLimits(true);
            speedCurveL.addElement(driveLeft.getCurrentPosition() - 0.05 * leftTicks, speedMin);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + 0.375 * leftTicks, speedMax);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + 0.625 * leftTicks, speedMax);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + 1.00 * leftTicks, speedMin);
            speedCurveR.addElement(driveRight.getCurrentPosition() - 0.05 * rightTicks, speedMin);
            speedCurveR.addElement(driveRight.getCurrentPosition() + 0.375 * rightTicks, speedMax);
            speedCurveR.addElement(driveRight.getCurrentPosition() + 0.625 * rightTicks, speedMax);
            speedCurveR.addElement(driveRight.getCurrentPosition() + 1.00 * rightTicks, speedMin);

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
        telemetry.addData("leftVel", driveLeft.getPower());
        telemetry.addData("rightVel", driveRight.getPower());
        logData("arcTo()", started + "," + done + "," +
                speedCurveL.isValid() + "," + speedCurveL.getSize() + "," +
                speedCurveR.isValid() + "," + speedCurveR.getSize() + "," +
                arcLengthL + "," + arcLengthR + "," +
                leftTicks + "," + rightTicks + "," + midTicks + "," + maxRatio);
    }

    public void setDoneFalse() {
        done = false;
    }
}
