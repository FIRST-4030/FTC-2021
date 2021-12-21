package org.firstinspires.ftc.teamcode.robot;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.PiecewiseFunction;

@Config
@Autonomous(name = "CodeTest", group = "Test")
public class PureCodeTest extends OpMode {
    // Config
    public static boolean DEBUG = false;
    private static final int INPUT_SCALING_EXPONENT = 3;
    public static double TICKS_PER_INCH = 43.24;
    public static double TURN_RATIO = 6.3;
    public static double ACCEL_CONSTANT = 0.4;
    public static double trackWidth = 15.25;
    public static double trackWidthHalf = trackWidth / 2.0;

    // Members
    private boolean enabled = false;
    private boolean auto = false;
    private ElapsedTime rampTimer = new ElapsedTime();
    private boolean started;
    private boolean done;
    private PiecewiseFunction speedCurve = new PiecewiseFunction();
    private int counts = 0;

    private double driveLeftPower;
    private double driveRightPower;

    // Standard methods
    @Override
    public void init() {
        try {
            done = false;
            started = false;
            enabled = true;
            counts = 0;

        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": Could not initialize");
        }
    }

    @Override
    public void start() {
        done = false;
        started = false;

        counts++;
        RobotLog.clearGlobalErrorMsg();
        RobotLog.clearGlobalWarningMsg();
        RobotLog.d(",try number," + counts);
        PiecewiseFunction testMe = new PiecewiseFunction();

        testMe.setDefaultValue(40.0);
        testMe.debug = false;
        testMe.addElement(-1,0);
        testMe.addElement(-0.91304347826087,0.407862239984646);
        testMe.addElement(-0.826086956521739,0.56354266942677);
        testMe.addElement(-0.739130434782609,0.673562321079551);
        testMe.addElement(-0.652173913043478,0.758069381485335);
        testMe.addElement(-0.565217391304348,0.824941998304795);
        testMe.addElement(-0.478260869565217,0.878217820727137);
        testMe.addElement(-0.391304347826087,0.920261325587684);
        testMe.addElement(-0.304347826086957,0.952560969574202);
        testMe.addElement(-0.217391304347826,0.976084535680159);
        testMe.addElement(-0.130434782608696,0.991456891390555);
        testMe.addElement(-0.0434782608695652,0.999054373310961);
        testMe.addElement(0.0434782608695652,0.999054373310961);
        testMe.addElement(0.130434782608696,0.991456891390555);
        testMe.addElement(0.217391304347826,0.976084535680159);
        testMe.addElement(0.304347826086957,0.952560969574202);
        testMe.addElement(0.391304347826087,0.920261325587684);
        testMe.addElement(0.478260869565217,0.878217820727137);
        testMe.addElement(0.565217391304348,0.824941998304795);
        testMe.addElement(0.652173913043478,0.758069381485335);
        testMe.addElement(0.739130434782609,0.673562321079551);
        testMe.addElement(0.826086956521739,0.563542669426771);
        testMe.addElement(0.91304347826087,0.407862239984646);
        testMe.addElement(1,0);

        testMe.setClampLimits(true);
        testPiecewise(testMe, "Clamped", 0.125, true);
        testMe.setClampLimits(false);
        testPiecewise(testMe, "Not Clamped", 0.125, false);

        testMe.reset();
        testMe.setClampLimits(false);
        testMe.debug = false;
        testMe.setDefaultValue(40.0);
        testMe.addElement(0,0);
        testMe.addElement(1,0);
        testMe.addElement(1,1);
        testMe.addElement(2,1);
        testMe.addElement(2,0);
        testMe.addElement(3,0);
        testMe.addElement(3,1);

        testMe.setDefaultHigh(true);
        testPiecewise(testMe, "Default High", 0.125, true);

        testMe.setDefaultHigh(false);
        testPiecewise(testMe, "Default Low", 0.125, false);

        testMe.reset();
        testMe.setClampLimits(true);
        testMe.debug = false;
        testMe.setDefaultValue(40.0);
        testMe.addElement(0,0);
        testMe.addElement(1,1);

        double X = testMe.getFirstX(), Y = testMe.getFirstY();
        while (!testMe.isClamped()) {
            Y = testMe.getY(X);
            X = X + 0.125;
        }
        RobotLog.d(",Final Y," + Y);

        testPiecewise(testMe, "While Loop", 0.125, true);

        telemetry.addData("logfile name", RobotLog.getLogFilename());
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }
    }

    @Override
    public void stop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        // Stop the drive motors
        driveLeftPower = 0;
        driveRightPower = 0;
    }

    // Custom methods
    public boolean isBusy() {
        return (driveLeftPower != 0 || driveRightPower != 0);
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

        // Start the motors
        auto = true;
        driveLeftPower = speed;
        driveRightPower = speed;
    }

    public void driveTo(double speedMin, double speedMax, double distance) {
        double speed = 0;
        double v = 40 * (speedMax * 4 + speedMin * 2) / 6; // inches per second
        double time = Math.abs(distance / v);

        if (!started) {
            rampTimer.reset();
            driveLeftPower = speedMin;
            driveRightPower = speedMin;

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
            driveLeftPower = 0;
            driveRightPower = 0;
        }

        if (isBusy() || !done) {
            // speed is calculated using the curve defined above
            speed = speedCurve.getY(rampTimer.seconds());
            done = speedCurve.isClamped();

            driveLeftPower = speed;
            driveRightPower = speed;
        }
        telemetry.log().add(getClass().getSimpleName() + "::driveTo(): Motors in use");
        telemetry.addData("speed", speed);
        telemetry.addData("timer", rampTimer.seconds());
        telemetry.addData("time", time);
    }

    public void turnTo(float speed, float angle) {
        // Don't allow new moves if we're still busy
        if (isBusy()) {
            telemetry.log().add(getClass().getSimpleName() + "::turnTo(): Motors in use");
            return;
        }

        // Fake turns using a distance translation
        // We have a gyro but let's start with just one control mode
        driveLeftPower = speed;
        driveRightPower = -speed;
    }

    public void testPiecewise(PiecewiseFunction function, String functionName, Double stepSize, Boolean logElements) {
        RobotLog.d(",");
        RobotLog.d(",name," + functionName);
        if (logElements) {
            RobotLog.d(",index,x,y");
            for (int i = 0; i < function.getCoordSize(); i++) {
                RobotLog.d("," + i + "," + function.getElementX(i) + "," + function.getElementY(i));
            }
        }
        RobotLog.d(",,x,y,isClamped");
        for (Double x = function.getFirstX() - stepSize; x <= function.getLastX() + stepSize; x += stepSize) {
            RobotLog.d(",," + x.toString() + "," + function.getY(x) + "," + function.isClamped());
        }
        RobotLog.d(",coord size," + function.getCoordSize());
        RobotLog.d(",clamp limits," + function.getClampLimits());
        RobotLog.d(",default high," + function.getDefaultHigh());
        RobotLog.d(",average," + function.getAverage());
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
            driveLeftPower = speedMin * (leftVel / v);
            driveRightPower = speedMin * (rightVel / v);

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
            driveLeftPower = 0;
            driveRightPower = 0;
        }

        if (isBusy() || !done) {
            // speed is calculated using the curve defined above
            speed = speedCurve.getY(rampTimer.seconds());
            done = speedCurve.isClamped();

            maxRatio = Math.max(leftVel, rightVel) / v;
            driveLeftPower = speed * (leftVel / v) / maxRatio;
            driveRightPower = speed * (rightVel / v) / maxRatio;
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
        driveLeftPower = 0;

        driveRightPower = 0;
    }
}
