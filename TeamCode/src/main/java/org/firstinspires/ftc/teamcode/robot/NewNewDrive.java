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
    public static double TICKS_PER_INCH = 44.5;
    public static double ACCEL_CONSTANT = 0.4;
    public static double trackWidth = 15.25;
    public static double trackWidthHalf = trackWidth / 2.0;
    public static double firstRampPoint = -0.025;
    public static double secondRampPoint = 0.5;
    public static double thirdRampPoint = 0.675;
    public static double lastRampPoint = 1.0;
    public static double accelMax = 0.025;
    private double ogPosL = 0;
    private double ogPosR = 0;

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
            driveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            driveRight = hardwareMap.get(DcMotor.class, "BR");
            driveRight.setDirection(DcMotorSimple.Direction.FORWARD);
            driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        double angle;

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

        double accelLeft = Math.abs(speedMax - speedMin) / (secondRampPoint * (leftTicks / TICKS_PER_INCH));
        double accelRight = Math.abs(speedMax - speedMin) / (secondRampPoint * (rightTicks / TICKS_PER_INCH));
        /* if (r > 0) {
            secondRampPoint = Math.abs(speedMax - speedMin) / (accelMax * (leftTicks / TICKS_PER_INCH));
            secondRampPoint = Math.min(0.5, secondRampPoint);
            speedMax = (secondRampPoint * (accelMax * (leftTicks / TICKS_PER_INCH)) + Math.abs(speedMin)) * Math.signum(speedMax);
        } else if (r < 0) {
            secondRampPoint = Math.abs(speedMax - speedMin) / (accelMax * (rightTicks/ TICKS_PER_INCH));
            secondRampPoint = Math.min(0.5, secondRampPoint);
            speedMax = (secondRampPoint * (accelMax * (rightTicks/ TICKS_PER_INCH)) + Math.abs(speedMin)) * Math.signum(speedMax);
        } */
        //thirdRampPoint = lastRampPoint - secondRampPoint;

        speedMin = Math.max(-1, speedMin);
        speedMin = Math.min(1, speedMin);
        speedMax = Math.max(-1, speedMax);
        speedMax = Math.min(1, speedMax);

        if (!started) {
            // initialize speedCurve to have motor ticks be the X coordinate and motor speed be the Y coordinate
            speedCurveL.setClampLimits(true);
            speedCurveR.setClampLimits(true);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + firstRampPoint * leftTicks, speedMin);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + secondRampPoint * leftTicks, speedMax);
            //speedCurveL.addElement(driveLeft.getCurrentPosition() + thirdRampPoint * leftTicks, speedMax);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + lastRampPoint * leftTicks, speedMin);
            speedCurveR.addElement(driveRight.getCurrentPosition() + firstRampPoint * rightTicks, speedMin);
            speedCurveR.addElement(driveRight.getCurrentPosition() + secondRampPoint * rightTicks, speedMax);
            //speedCurveR.addElement(driveRight.getCurrentPosition() + thirdRampPoint * rightTicks, speedMax);
            speedCurveR.addElement(driveRight.getCurrentPosition() + lastRampPoint * rightTicks, speedMin);

            // Ramp-and-hold
            //rampAndHold(speedCurveL, (int)leftTicks, driveLeft.getCurrentPosition(), speedMin, speedMax);
            //rampAndHold(speedCurveR, (int)rightTicks, driveRight.getCurrentPosition(), speedMin, speedMax);

            ogPosL = driveLeft.getCurrentPosition();
            ogPosR = driveRight.getCurrentPosition();

            started = true;
            done = false;
        } else if (done) {
            driveLeft.setPower(0);
            driveRight.setPower(0);
            started = false;
            speedCurveL.reset();
            speedCurveR.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::arcTo(): Motors in use");
        telemetry.addData("left ticks", driveLeft.getCurrentPosition());
        telemetry.addData("right ticks", driveRight.getCurrentPosition());
        telemetry.addData("leftVel", driveLeft.getPower());
        telemetry.addData("rightVel", driveRight.getPower());
        telemetry.addData("AccelLeft", accelLeft);
        telemetry.addData("AccelRight", accelRight);
        logData("arcTo()", started + "," + done + "," +
                speedCurveL.isValid() + "," + speedCurveL.getSize() + "," +
                speedCurveR.isValid() + "," + speedCurveR.getSize() + "," +
                arcLengthL + "," + arcLengthR + "," +
                leftTicks + "," + rightTicks + "," + midTicks + "," + maxRatio + "," + accelLeft + "," + accelRight);
    }

    public void combinedCurves(double r1, double arcLength1, double speedMin1, double speedMax1, double r2, double arcLength2, double speedMin2, double speedMax2) {
        if (arcLength1 == 0 && arcLength2 == 0) {
            return;
        }
        double arcLengthL1;
        double arcLengthR1;
        double arcLengthL2;
        double arcLengthR2;
        double arcLengthInner1;
        double arcLengthOuter1;
        double arcLengthInner2;
        double arcLengthOuter2;
        double leftTicks1;
        double rightTicks1;
        double leftTicks2;
        double rightTicks2;
        double angle1 = 0;
        double angle2 = 0;

        if (r1 == 0) {
            arcLengthL1 = arcLength1;
            arcLengthR1 = arcLength1;
        } else {
            angle1 = Math.toDegrees(arcLength1 / Math.abs(r1));
            arcLengthInner1 = Math.toRadians(angle1) * (Math.abs(r1) - trackWidthHalf);
            arcLengthOuter1 = Math.toRadians(angle1) * (Math.abs(r1) + trackWidthHalf);

            if (r1 > 0) {
                // if radius is greater than zero, we are moving to the left, so the right side is on the outside
                arcLengthL1 = arcLengthInner1;
                arcLengthR1 = arcLengthOuter1;
            } else {
                // if radius is greater than zero, we are moving to the left, so the right side is on the inside
                arcLengthL1 = arcLengthOuter1;
                arcLengthR1 = arcLengthInner1;
            }
        }

        if (r2 == 0) {
            arcLengthL2 = arcLength2;
            arcLengthR2 = arcLength2;
        } else {
            angle2 = Math.toDegrees(arcLength2 / Math.abs(r2));
            arcLengthInner2 = Math.toRadians(angle2) * (Math.abs(r2) - trackWidthHalf);
            arcLengthOuter2 = Math.toRadians(angle2) * (Math.abs(r2) + trackWidthHalf);

            if (r1 > 0) {
                // if radius is greater than zero, we are moving to the left, so the right side is on the outside
                arcLengthL2 = arcLengthInner2;
                arcLengthR2 = arcLengthOuter2;
            } else {
                // if radius is greater than zero, we are moving to the left, so the right side is on the inside
                arcLengthL2 = arcLengthOuter2;
                arcLengthR2 = arcLengthInner2;
            }
        }

        leftTicks1 = arcLengthL1 * TICKS_PER_INCH;
        rightTicks1 = arcLengthR1 * TICKS_PER_INCH;
        leftTicks2 = arcLengthL2 * TICKS_PER_INCH;
        rightTicks2 = arcLengthR2 * TICKS_PER_INCH;
        double midTicks1 = arcLength1 * TICKS_PER_INCH;
        double midTicks2 = arcLength2 * TICKS_PER_INCH;

        double maxRatio = 0;
        if ((isBusy() || !done) && speedCurveL.isValid() && speedCurveR.isValid() && started) {
            // speed is calculated using the curve defined above
            if (driveLeft.getCurrentPosition() < ogPosL + leftTicks1 && driveRight.getCurrentPosition() >= ogPosR + rightTicks1) {
                maxRatio = Math.max(Math.abs(leftTicks1), Math.abs(rightTicks1)) / Math.abs(midTicks1);
                driveLeft.setPower((leftTicks1 / midTicks1) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
                driveRight.setPower((rightTicks1 / midTicks1) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
            } else {
                maxRatio = Math.max(Math.abs(leftTicks2), Math.abs(rightTicks2)) / Math.abs(midTicks2);
                driveLeft.setPower((leftTicks2 / midTicks2) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
                driveRight.setPower((rightTicks2 / midTicks2) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
            }
            done = speedCurveL.isClamped() && speedCurveR.isClamped();
        }

        if (!started) {
            // initialize speedCurve to have motor ticks be the X coordinate and motor speed be the Y coordinate
            speedCurveL.setClampLimits(true);
            speedCurveR.setClampLimits(true);
            speedCurveL.addElement(driveLeft.getCurrentPosition() - 0.05 * leftTicks1, speedMin1);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + 0.5 * leftTicks1, speedMax1);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + 1.00 * leftTicks1, (speedMax1 + speedMax2) / 2.0);
            speedCurveL.addElement(driveLeft.getCurrentPosition() - 0.05 * leftTicks2 + leftTicks1, (speedMax1 + speedMax2) / 2.0);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + 0.5 * leftTicks2 + leftTicks1, speedMax2);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + 1.00 * leftTicks2 + leftTicks1, speedMin2);

            speedCurveR.addElement(driveLeft.getCurrentPosition() - 0.05 * rightTicks1, speedMin1);
            speedCurveR.addElement(driveLeft.getCurrentPosition() + 0.5 * rightTicks1, speedMax1);
            speedCurveR.addElement(driveLeft.getCurrentPosition() + 1.00 * rightTicks1, (speedMax1 + speedMax2) / 2.0);
            speedCurveR.addElement(driveLeft.getCurrentPosition() - 0.05 * rightTicks2 + rightTicks1, (speedMax1 + speedMax2) / 2.0);
            speedCurveR.addElement(driveLeft.getCurrentPosition() + 0.5 * rightTicks2 + rightTicks1, speedMax2);
            speedCurveR.addElement(driveLeft.getCurrentPosition() + 1.00 * rightTicks2 + rightTicks1, speedMin2);

            ogPosL = driveLeft.getCurrentPosition();
            ogPosR = driveRight.getCurrentPosition();

            started = true;
            done = false;
        } else if (done) {
            driveLeft.setPower(0);
            driveRight.setPower(0);
            started = false;
            speedCurveL.reset();
            speedCurveR.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::arcTo(): Motors in use");
        telemetry.addData("left ticks", driveLeft.getCurrentPosition());
        telemetry.addData("right ticks", driveRight.getCurrentPosition());
        telemetry.addData("leftVel", driveLeft.getPower());
        telemetry.addData("rightVel", driveRight.getPower());
        logData("arcTo()", started + "," + done + "," +
                speedCurveL.isValid() + "," + speedCurveL.getSize() + "," +
                speedCurveR.isValid() + "," + speedCurveR.getSize() + "," +
                arcLengthL1 + "," + arcLengthR1 + "," +
                leftTicks1 + "," + rightTicks1 + "," + midTicks1 + "," + maxRatio);
    }

    public void circle(double r, double angle, double speedMin, double speedMax) {
        if (r == 0) {
            return;
        }
        double arcLengthL;
        double arcLengthR;
        double arcLengthInner;
        double arcLengthOuter;
        double leftTicks;
        double rightTicks;
        // it should be, but ensure that the radius is positive

        double arcLength = 2 * Math.PI * Math.abs(r);
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

        double accelLeft = Math.abs(speedMax - speedMin) / (secondRampPoint * leftTicks);
        double accelRight = Math.abs(speedMax - speedMin) / (secondRampPoint * rightTicks);
        /* if (r > 0) {
            secondRampPoint = Math.abs(speedMax - speedMin) / (accelMax * leftTicks);
            secondRampPoint = Math.min(0.5, secondRampPoint);
            speedMax = (secondRampPoint * (accelMax * leftTicks) + Math.abs(speedMin)) * Math.signum(speedMax);
        } else if (r < 0) {
            secondRampPoint = Math.abs(speedMax - speedMin) / (accelMax * rightTicks);
            secondRampPoint = Math.min(0.5, secondRampPoint);
            speedMax = (secondRampPoint * (accelRight * rightTicks) + Math.abs(speedMin)) * Math.signum(speedMax);
        }
        thirdRampPoint = lastRampPoint - secondRampPoint; */

        if (!started) {
            // initialize speedCurve to have motor ticks be the X coordinate and motor speed be the Y coordinate
            speedCurveL.setClampLimits(true);
            speedCurveR.setClampLimits(true);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + firstRampPoint * leftTicks, speedMin);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + secondRampPoint * leftTicks, speedMax);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + thirdRampPoint * leftTicks, speedMax);
            speedCurveL.addElement(driveLeft.getCurrentPosition() + lastRampPoint * leftTicks, speedMin);
            speedCurveR.addElement(driveRight.getCurrentPosition() + firstRampPoint * rightTicks, speedMin);
            speedCurveR.addElement(driveRight.getCurrentPosition() + secondRampPoint * rightTicks, speedMax);
            speedCurveR.addElement(driveRight.getCurrentPosition() + thirdRampPoint * rightTicks, speedMax);
            speedCurveR.addElement(driveRight.getCurrentPosition() + lastRampPoint * rightTicks, speedMin);

            ogPosL = driveLeft.getCurrentPosition();
            ogPosR = driveRight.getCurrentPosition();

            started = true;
            done = false;
        } else if (done) {
            driveLeft.setPower(0);
            driveRight.setPower(0);
            started = false;
            speedCurveL.reset();
            speedCurveR.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::arcTo(): Motors in use");
        telemetry.addData("left ticks", driveLeft.getCurrentPosition());
        telemetry.addData("right ticks", driveRight.getCurrentPosition());
        telemetry.addData("leftVel", driveLeft.getPower());
        telemetry.addData("rightVel", driveRight.getPower());
        telemetry.addData("AccelLeft", accelLeft);
        telemetry.addData("AccelRight", accelRight);
        logData("arcTo()", started + "," + done + "," +
                speedCurveL.isValid() + "," + speedCurveL.getSize() + "," +
                speedCurveR.isValid() + "," + speedCurveR.getSize() + "," +
                arcLengthL + "," + arcLengthR + "," +
                leftTicks + "," + rightTicks + "," + midTicks + "," + maxRatio + "," + accelLeft + "," + accelRight);
    }

    // Build a 3-point or 4-point piecewise function for linear-ramp-and-hold command curves
    // If ramp up/down ticks were parameters this could be a method in PiecewiseFunction
    // That would provide both a sample of code usage and of different control models
    private void rampAndHold(
            PiecewiseFunction pfunc,
            int pathTicks, int currentTicks,
            double speedMin, double speedMax) {
        // How many ticks does it take to ramp up/down between speedMin and speedMax
        // Higher values correlate with longer ramp times and smaller acceleration
        // Usually experiment and measurement can determine an approximate value
        int rampUpTicks = 150;
        int rampDownTicks = 150;

        if (rampUpTicks + rampDownTicks >= pathTicks) {
            // Path is shorter than the ramp up/down intervals
            // 3 ramp points at 0%, 50% and 100%
            pfunc.addElement(currentTicks, speedMin);
            pfunc.addElement(currentTicks + pathTicks / 2, speedMax);
            pfunc.addElement(currentTicks + pathTicks, speedMin);
        } else {
            // Path is long enough to ramp to full speed
            // 4 ramp points at 0%, rampUpTicks, 100% - rampDownTicks, and 100%
            pfunc.addElement(currentTicks, speedMin);
            pfunc.addElement(currentTicks + rampUpTicks, speedMax);
            pfunc.addElement(currentTicks + pathTicks - rampDownTicks, speedMax);
            pfunc.addElement(currentTicks + pathTicks, speedMin);
        }

        // Enable first/last element clamping in case the encoder values drift outside the model
        pfunc.setClampLimits(true);
    }

    public double leftPos() {
        return (driveLeft.getCurrentPosition() / TICKS_PER_INCH);
    }

    public double rightPos() {
        return (driveRight.getCurrentPosition() / TICKS_PER_INCH);
    }

    public double leftVel() {
        return driveLeft.getPower();
    }

    public double rightVel() {
        return driveRight.getPower();
    }

    public void setDoneFalse() {
        done = false;
    }
}
