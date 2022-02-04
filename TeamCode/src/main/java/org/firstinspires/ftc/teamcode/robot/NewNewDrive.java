package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.utils.PiecewiseFunction;

@Config
public class NewNewDrive extends OpMode {
    // Config
    public static boolean DEBUG = false;
    private static final double MAX_VELOCITY = 40.0; // inches per second
    private final double TICKS_PER_INCH = 44.5;
    public static double trackWidth = 17.5;
    public static double trackWidthHalf = trackWidth / 2.0;
    public static double firstRampPoint = -0.025;
    public static double secondRampPoint = 0.5;
    public static double thirdRampPoint = 0.675;
    public static double lastRampPoint = 1.0;
    public static double rampMaxTicks = 800;
    private double ogPosL = 0;
    private double ogPosR = 0;
    private boolean notOver1stMoveL = false;
    private boolean notOver1stMoveR = false;
    private boolean notOver2ndMoveL = false;
    private boolean notOver2ndMoveR = false;
    private boolean angleSet = false;

    // Hardware
    private DcMotor driveLeft;
    private DcMotor driveRight;

    // Members
    private boolean enabled = false;
    private boolean auto = false;
    final private ElapsedTime rampTimer = new ElapsedTime();
    private boolean started;
    private boolean done;
    private boolean loggingEnabled = false;
    private PiecewiseFunction speedCurve;
    private PiecewiseFunction speedCurveL;
    private PiecewiseFunction speedCurveR;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    public static double globalAngle, correction;
    public static double gain = 0.03;

    // Standard methods
    @Override
    public void init() {
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

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);

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

        // Skip driver control while there's an active auto command
        if (auto) {
            if (!isBusy()) {
                auto = false;
            }
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
    public void driveTo(double distance, double speedMin, double speedMax) {
        if (distance == 0) {
            return;
        }
        correction = checkDirection();
        double midTicks = distance * TICKS_PER_INCH;
        speedMin = Math.max(-1, speedMin);
        speedMin = Math.min(1, speedMin);
        speedMax = Math.max(-1, speedMax);
        speedMax = Math.min(1, speedMax);

        if ((isBusy() || !done) && speedCurveL.isValid() && speedCurveR.isValid() && started) {
            // speed is calculated using the curve defined above
            driveLeft.setPower(speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0) - correction);
            driveRight.setPower(speedCurveR.getY(driveRight.getCurrentPosition() * 1.0) + correction);
            done = speedCurveL.isClamped() && speedCurveR.isClamped();
        }

        if (!started) {
            resetAngle();

            // initialize speedCurveL and speedCurveR to have motor ticks be the X coordinate and motor speed be the Y coordinate
            // Ramp-and-hold
            rampAndHold(speedCurveL, (int) midTicks, driveLeft.getCurrentPosition(), speedMin, speedMax);
            rampAndHold(speedCurveR, (int) midTicks, driveRight.getCurrentPosition(), speedMin, speedMax);
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
            correction = checkDirection();
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
            if (r == 0) {
                if (!angleSet) {
                    resetAngle();
                    angleSet = true;
                }
                driveLeft.setPower((leftTicks / midTicks) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0) - correction);
                driveRight.setPower((rightTicks / midTicks) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0) + correction);
            } else {
                angleSet = false;
                driveLeft.setPower((leftTicks / midTicks) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
                driveRight.setPower((rightTicks / midTicks) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
            }
            done = speedCurveL.isClamped() || speedCurveR.isClamped();
        }

        speedMin = Math.max(-1, speedMin);
        speedMin = Math.min(1, speedMin);
        speedMax = Math.max(-1, speedMax);
        speedMax = Math.min(1, speedMax);

        if (!started) {
            // initialize speedCurve to have motor ticks be the X coordinate and motor speed be the Y coordinate
            // Ramp-and-hold
            rampAndHold2(speedCurveL, (int) leftTicks, driveLeft.getCurrentPosition(), speedMin, speedMax, ((leftTicks / midTicks) / maxRatio));
            rampAndHold2(speedCurveR, (int) rightTicks, driveRight.getCurrentPosition(), speedMin, speedMax, ((rightTicks / midTicks) / maxRatio));

            ogPosL = driveLeft.getCurrentPosition();
            ogPosR = driveRight.getCurrentPosition();

            started = true;
            done = false;
        } else if (done) {
            driveLeft.setPower(0);
            driveRight.setPower(0);
            started = false;
            angleSet = false;
            speedCurveL.reset();
            speedCurveR.reset();
        }

        //telemetry.log().add(getClass().getSimpleName() + "::arcTo(): Motors in use");
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

    public void combinedCurves(double r1, double arcLength1, double r2, double arcLength2, double speedMin, double speedMax) {
        if (arcLength1 == 0 && arcLength2 == 0) {
            return;
        }
        double arcLengthL1, arcLengthR1;
        double arcLengthL2, arcLengthR2;;
        double arcLengthInner1, arcLengthOuter1;
        double arcLengthInner2, arcLengthOuter2;
        double leftTicks1, rightTicks1;
        double leftTicks2, rightTicks2;
        double angle1, angle2;
        correction = checkDirection();

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

            if (r2 > 0) {
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
            if (leftTicks1 > 0) {
                notOver1stMoveL = (driveLeft.getCurrentPosition() <= (ogPosL + leftTicks1));
            } else {
                notOver1stMoveL = (driveLeft.getCurrentPosition() >= (ogPosL + leftTicks1));
            }
            if (rightTicks1 > 0) {
                notOver1stMoveR = (driveRight.getCurrentPosition() <= (ogPosR + rightTicks1));
            } else {
                notOver1stMoveR = (driveRight.getCurrentPosition() >= (ogPosR + rightTicks1));
            }
            if (notOver1stMoveL && notOver1stMoveR) {
                maxRatio = Math.max(Math.abs(leftTicks1), Math.abs(rightTicks1)) / Math.abs(midTicks1);
                if (r1 == 0) {
                    if (!angleSet) {
                        resetAngle();
                        angleSet = true;
                    }
                    driveLeft.setPower((leftTicks1 / midTicks1) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0) - correction);
                    driveRight.setPower((rightTicks1 / midTicks1) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0) + correction);
                } else {
                    angleSet = false;
                    driveLeft.setPower((leftTicks1 / midTicks1) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
                    driveRight.setPower((rightTicks1 / midTicks1) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
                }
            } else {
                maxRatio = Math.max(Math.abs(leftTicks2), Math.abs(rightTicks2)) / Math.abs(midTicks2);
                if (r2 == 0) {
                    if (!angleSet) {
                        resetAngle();
                        angleSet = true;
                    }
                    driveLeft.setPower((leftTicks2 / midTicks2) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0) - correction);
                    driveRight.setPower((rightTicks2 / midTicks2) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0) + correction);
                } else {
                    angleSet = false;
                    driveLeft.setPower((leftTicks2 / midTicks2) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
                    driveRight.setPower((rightTicks2 / midTicks2) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
                }
            }
            done = speedCurveL.isClamped() && speedCurveR.isClamped();
        }

        if (!started) {
            // initialize speedCurve to have motor ticks be the X coordinate and motor speed be the Y coordinate
            rampAndHold(speedCurveL, (int) (leftTicks1 + leftTicks2), driveLeft.getCurrentPosition(), speedMin, speedMax);
            rampAndHold(speedCurveR, (int) (rightTicks1 + rightTicks2), driveRight.getCurrentPosition(), speedMin, speedMax);

            ogPosL = driveLeft.getCurrentPosition();
            ogPosR = driveRight.getCurrentPosition();

            started = true;
            done = false;
        } else if (done) {
            driveLeft.setPower(0);
            driveRight.setPower(0);
            started = false;
            notOver1stMoveL = false;
            notOver1stMoveR = false;
            angleSet = false;
            speedCurveL.reset();
            speedCurveR.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::combinedCurves(): Motors in use");
        telemetry.addData("left ticks", driveLeft.getCurrentPosition());
        telemetry.addData("right ticks", driveRight.getCurrentPosition());
        telemetry.addData("leftVel", driveLeft.getPower());
        telemetry.addData("rightVel", driveRight.getPower());
        logData("arcTo()", started + "," + done + "," +
                speedCurveL.isValid() + "," + speedCurveL.getSize() + "," +
                speedCurveR.isValid() + "," + speedCurveR.getSize() + "," +
                arcLengthL1 + "," + arcLengthR1 + "," +
                leftTicks1 + "," + rightTicks1 + "," + midTicks1 + "," + arcLengthL2 + "," + arcLengthR2 + "," +
                leftTicks2 + "," + rightTicks2 + "," + midTicks2 + "," + maxRatio);
    }

    public void combined3Curves(double r1, double arcLength1, double r2, double arcLength2, double r3, double arcLength3, double speedMin, double speedMax) {
        if (arcLength1 == 0 && arcLength2 == 0 && arcLength3 == 0) {
            return;
        }
        double arcLengthL1, arcLengthR1;
        double arcLengthL2, arcLengthR2;;
        double arcLengthL3, arcLengthR3;
        double arcLengthInner1, arcLengthOuter1;
        double arcLengthInner2, arcLengthOuter2;
        double arcLengthInner3, arcLengthOuter3;
        double leftTicks1, rightTicks1;
        double leftTicks2, rightTicks2;
        double leftTicks3, rightTicks3;
        double angle1, angle2, angle3;
        correction = checkDirection();

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

            if (r2 > 0) {
                // if radius is greater than zero, we are moving to the left, so the right side is on the outside
                arcLengthL2 = arcLengthInner2;
                arcLengthR2 = arcLengthOuter2;
            } else {
                // if radius is greater than zero, we are moving to the left, so the right side is on the inside
                arcLengthL2 = arcLengthOuter2;
                arcLengthR2 = arcLengthInner2;
            }
        }

        if (r3 == 0) {
            arcLengthL3 = arcLength3;
            arcLengthR3 = arcLength3;
        } else {
            angle3 = Math.toDegrees(arcLength3 / Math.abs(r3));
            arcLengthInner3 = Math.toRadians(angle3) * (Math.abs(r3) - trackWidthHalf);
            arcLengthOuter3 = Math.toRadians(angle3) * (Math.abs(r3) + trackWidthHalf);

            if (r2 > 0) {
                // if radius is greater than zero, we are moving to the left, so the right side is on the outside
                arcLengthL3 = arcLengthInner3;
                arcLengthR3 = arcLengthOuter3;
            } else {
                // if radius is greater than zero, we are moving to the left, so the right side is on the inside
                arcLengthL3 = arcLengthOuter3;
                arcLengthR3 = arcLengthInner3;
            }
        }

        leftTicks1 = arcLengthL1 * TICKS_PER_INCH;
        rightTicks1 = arcLengthR1 * TICKS_PER_INCH;
        leftTicks2 = arcLengthL2 * TICKS_PER_INCH;
        rightTicks2 = arcLengthR2 * TICKS_PER_INCH;
        leftTicks3 = arcLengthL3 * TICKS_PER_INCH;
        rightTicks3 = arcLengthR3 * TICKS_PER_INCH;
        double midTicks1 = arcLength1 * TICKS_PER_INCH;
        double midTicks2 = arcLength2 * TICKS_PER_INCH;
        double midTicks3 = arcLength3 * TICKS_PER_INCH;

        double maxRatio = 0;
        if ((isBusy() || !done) && speedCurveL.isValid() && speedCurveR.isValid() && started) {
            // speed is calculated using the curve defined above
            if (leftTicks1 > 0) {
                notOver1stMoveL = (driveLeft.getCurrentPosition() <= ogPosL + leftTicks1);
            } else {
                notOver1stMoveL = (driveLeft.getCurrentPosition() >= ogPosL + leftTicks1);
            }
            if (rightTicks1 > 0) {
                notOver1stMoveR = (driveRight.getCurrentPosition() <= ogPosR + rightTicks1);
            } else {
                notOver1stMoveR = (driveRight.getCurrentPosition() >= ogPosR + rightTicks1);
            }
            if (leftTicks2 > 0) {
                notOver2ndMoveL = (driveLeft.getCurrentPosition() <= ogPosL + leftTicks1 + leftTicks2);
            } else {
                notOver2ndMoveL = (driveLeft.getCurrentPosition() >= ogPosL + leftTicks1 + leftTicks2);
            }
            if (rightTicks2 > 0) {
                notOver2ndMoveR = (driveRight.getCurrentPosition() <= ogPosR + rightTicks1 + rightTicks2);
            } else {
                notOver2ndMoveR = (driveRight.getCurrentPosition() >= ogPosR + rightTicks1 + rightTicks2);
            }
            if (notOver1stMoveL && notOver1stMoveR) {
                maxRatio = Math.max(Math.abs(leftTicks1), Math.abs(rightTicks1)) / Math.abs(midTicks1);
                if (r1 == 0) {
                    if (!angleSet) {
                        resetAngle();
                        angleSet = true;
                    }
                    driveLeft.setPower((leftTicks1 / midTicks1) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0) - correction);
                    driveRight.setPower((rightTicks1 / midTicks1) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0) + correction);
                } else {
                    angleSet = false;
                    driveLeft.setPower((leftTicks1 / midTicks1) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
                    driveRight.setPower((rightTicks1 / midTicks1) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
                }
            } else if (notOver2ndMoveL && notOver2ndMoveR) {
                maxRatio = Math.max(Math.abs(leftTicks2), Math.abs(rightTicks2)) / Math.abs(midTicks2);
                if (r2 == 0) {
                    if (!angleSet) {
                        resetAngle();
                        angleSet = true;
                    }
                    driveLeft.setPower((leftTicks2 / midTicks2) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0) - correction);
                    driveRight.setPower((rightTicks2 / midTicks2) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0) + correction);
                } else {
                    angleSet = false;
                    driveLeft.setPower((leftTicks2 / midTicks2) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
                    driveRight.setPower((rightTicks2 / midTicks2) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
                }
            } else {
                maxRatio = Math.max(Math.abs(leftTicks3), Math.abs(rightTicks3)) / Math.abs(midTicks3);
                if (r3 == 0) {
                    if (!angleSet) {
                        resetAngle();
                        angleSet = true;
                    }
                    driveLeft.setPower((leftTicks3 / midTicks3) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0) - correction);
                    driveRight.setPower((rightTicks3 / midTicks3) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0) + correction);
                } else {
                    angleSet = false;
                    driveLeft.setPower((leftTicks3 / midTicks3) / maxRatio * speedCurveL.getY(driveLeft.getCurrentPosition() * 1.0));
                    driveRight.setPower((rightTicks3 / midTicks3) / maxRatio * speedCurveR.getY(driveRight.getCurrentPosition() * 1.0));
                }
            }
            done = speedCurveL.isClamped() && speedCurveR.isClamped();
        }

        if (!started) {
            rampAndHold(speedCurveL, (int) (leftTicks1 + leftTicks2 + leftTicks3), driveLeft.getCurrentPosition(), speedMin, speedMax);
            rampAndHold(speedCurveR, (int) (rightTicks1 + rightTicks2 + rightTicks3), driveRight.getCurrentPosition(), speedMin, speedMax);

            ogPosL = driveLeft.getCurrentPosition();
            ogPosR = driveRight.getCurrentPosition();

            started = true;
            angleSet = false;
            done = false;
        } else if (done) {
            driveLeft.setPower(0);
            driveRight.setPower(0);
            started = false;
            notOver1stMoveL = false;
            notOver1stMoveR = false;
            notOver2ndMoveL = false;
            notOver2ndMoveR = false;
            angleSet = false;
            speedCurveL.reset();
            speedCurveR.reset();
        }

        telemetry.log().add(getClass().getSimpleName() + "::combined3Curves(): Motors in use");
        telemetry.addData("left ticks", driveLeft.getCurrentPosition());
        telemetry.addData("right ticks", driveRight.getCurrentPosition());
        telemetry.addData("leftVel", driveLeft.getPower());
        telemetry.addData("rightVel", driveRight.getPower());
        logData("arcTo()", started + "," + done + "," +
                speedCurveL.isValid() + "," + speedCurveL.getSize() + "," +
                speedCurveR.isValid() + "," + speedCurveR.getSize() + "," +
                arcLengthL1 + "," + arcLengthR1 + "," +
                leftTicks1 + "," + rightTicks1 + "," + midTicks1 + "," + arcLengthL2 + "," + arcLengthR2 + "," +
                leftTicks2 + "," + rightTicks2 + "," + midTicks2 + "," + arcLengthL3 + "," + arcLengthR3 + "," +
                leftTicks3 + "," + rightTicks3 + "," + midTicks3 + "," + maxRatio);
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
        double rampUpTicks = rampMaxTicks * Math.abs(speedMax - speedMin);
        double rampDownTicks = rampMaxTicks * Math.abs(speedMax - speedMin);

        rampUpTicks *= Math.signum(pathTicks);
        rampDownTicks *= Math.signum(pathTicks);

        if (Math.abs(rampUpTicks + rampDownTicks) >= Math.abs(pathTicks)) {
            // Path is shorter than the ramp up/down intervals
            // 3 ramp points at 0%, 50% and 100%
            speedMax = ((speedMax - speedMin) * (pathTicks / 2f) / rampUpTicks) + speedMin;
            pfunc.addElement(currentTicks - 0.025 * pathTicks, speedMin);
            pfunc.addElement(currentTicks + pathTicks / 2f, speedMax);
            pfunc.addElement(currentTicks + pathTicks, speedMin);
        } else {
            // Path is long enough to ramp to full speed
            // 4 ramp points at 0%, rampUpTicks, 100% - rampDownTicks, and 100%
            pfunc.addElement(currentTicks - 0.025 * pathTicks, speedMin);
            pfunc.addElement(currentTicks + rampUpTicks, speedMax);
            pfunc.addElement(currentTicks + pathTicks - rampDownTicks, speedMax);
            pfunc.addElement(currentTicks + pathTicks, speedMin);
        }

        // Enable first/last element clamping in case the encoder values drift outside the model
        pfunc.setClampLimits(true);
    }

    private void rampAndHold2(
            PiecewiseFunction pfunc,
            int pathTicks, int currentTicks,
            double speedMin, double speedMax, double treadSpeedMax) {
        // How many ticks does it take to ramp up/down between speedMin and speedMax
        // Higher values correlate with longer ramp times and smaller acceleration
        // Usually experiment and measurement can determine an approximate value
        double rampUpTicks = rampMaxTicks * Math.abs(speedMax - speedMin) * Math.abs(treadSpeedMax);
        double rampDownTicks = rampMaxTicks * Math.abs(speedMax - speedMin) * Math.abs(treadSpeedMax);

        rampUpTicks *= Math.signum(pathTicks);
        rampDownTicks *= Math.signum(pathTicks);

        if (Math.abs(rampUpTicks + rampDownTicks) >= Math.abs(pathTicks)) {
            // Path is shorter than the ramp up/down intervals
            // 3 ramp points at 0%, 50% and 100%
            pfunc.addElement(currentTicks - 0.01 * pathTicks, speedMin);
            pfunc.addElement(currentTicks + (pathTicks / 2.0), speedMax);
            pfunc.addElement(currentTicks + pathTicks, speedMin);
        } else {
            // Path is long enough to ramp to full speed
            // 4 ramp points at 0%, rampUpTicks, 100% - rampDownTicks, and 100%
            pfunc.addElement(currentTicks - 0.01 * pathTicks, speedMin);
            pfunc.addElement(currentTicks + rampUpTicks, speedMax);
            pfunc.addElement(currentTicks + pathTicks - rampDownTicks, speedMax);
            pfunc.addElement(currentTicks + pathTicks, speedMin);
        }

        boolean left = false;
        if (pfunc.equals(speedCurveL)) {
            left = true;
        } else if (pfunc.equals(speedCurveR)) {
            left = false;
        }
        telemetry.log().add("rampAndHold2: " + (left ? "L" : "R") + "," + pathTicks + "," + rampUpTicks);
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

    public void slowReverse() {
        if (!started) {
            ogPosL = driveLeft.getCurrentPosition();
            ogPosR = driveRight.getCurrentPosition();
            started = true;
        }
        driveLeft.setPower(-0.1);
        driveRight.setPower(-0.1);
    }

    public double returnPosL() {
        return driveLeft.getCurrentPosition();
    }

    public double returnPosR() {
        return driveRight.getCurrentPosition();
    }

    public double returnOffSet() {
        return ((Math.abs(driveLeft.getCurrentPosition() - ogPosL) + Math.abs(driveRight.getCurrentPosition() - ogPosR)) / 2.0 / TICKS_PER_INCH);
    }

    public void returnToPos(double distance) {
        arcTo(0, distance, 0.1, 0.2);
    }

    public void stopDrive() {
        driveLeft.setPower(0);
        driveRight.setPower(0);
        done = true;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double difference, angle;

        angle = getAngle();

        if (angle == 0)
            difference = 0;             // no adjustment.
        else
            difference = -angle;        // reverse sign of angle for correction.

        difference = difference * gain;

        return difference;
    }
}
