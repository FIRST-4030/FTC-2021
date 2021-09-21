package org.firstinspires.ftc.teamcode.wheels;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actuators.Motor;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.encoderTicksToInches;

public class TankDrive implements Wheels {
    private static final boolean DEBUG = false;
    private static final float JOYSTICK_DEADZONE = 0.1f;
    private static final float SPEED_DEADZONE = JOYSTICK_DEADZONE * 0.85f;
    private static final int JOYSTICK_EXPONENT = 2;
    //public static final MOTOR_SIDE DEFAULT_ENCODER_SIDE = MOTOR_SIDE.RIGHT;
    //public static final MOTOR_END DEFAULT_ENCODER_END = MOTOR_END.FRONT;

    protected WheelsConfig config = null;
    protected final Telemetry telemetry;
    protected float speedScale = 1.0f;
    private boolean teleop = false;

    public TankDrive(HardwareMap map, Telemetry telemetry, WheelsConfig config) {
        this.telemetry = telemetry;
        for (WheelMotor wheelMotor : config.motors) {
            if (wheelMotor == null) {
                telemetry.log().add(this.getClass().getSimpleName() + ": Null motor");
                break;
            }
            wheelMotor.motor = new Motor(map, telemetry, wheelMotor);
            if (!wheelMotor.motor.isAvailable()) {
                return;
            }
        }
        this.config = config;
        resetEncoder();
    }

    public boolean isAvailable() {
        return config != null;
    }

    public Motor getMotor(MOTOR_SIDE side, MOTOR_END end){
        int i = getIndex(side, end);
        return config.motors[i].get();
    }

    public Motor getMotor(MOTOR_SIDE side){
        int i = getIndex(side, MOTOR_END.FRONT);
        return config.motors[i].get();
    }

    private int getIndex(MOTOR_SIDE side, MOTOR_END end) {
        for (int i = 0; i < config.motors.length; i++) {
            /*if (end == null && side == null) {
                end = DEFAULT_ENCODER_END;
                side = DEFAULT_ENCODER_SIDE;
            }*/
            if ((end == null || config.motors[i].end == end) &&
                    (side == null || config.motors[i].side == side)) {
                return i;
            }
        }
        throw new IllegalArgumentException("no motor of given side and end: " + side + " and " + end);
    }

    public void resetEncoder() {
        resetEncoder(null, null);
    }

    public void resetEncoder(MOTOR_SIDE side) {
        resetEncoder(side, null);
    }

    public void resetEncoder(MOTOR_SIDE side, MOTOR_END end) {
        if (!isAvailable()) {
            return;
        }
        config.motors[getIndex(side, end)].motor.resetEncoder();
    }

    public float getTicksPerMM() {
        return getTicksPerMM(null, null);
    }

    public float getTicksPerMM(MOTOR_SIDE side) {
        return getTicksPerMM(side, null);
    }

    public float getTicksPerMM(MOTOR_SIDE side, MOTOR_END end) {
        if (!isAvailable()) {
            return 0.0f;
        }
        return config.motors[getIndex(side, end)].ticksPerMM;
    }

    public float getTranslationTicksPerMM() {
        return getTranslationTicksPerMM(null, null);
    }

    public float getTranslationTicksPerMM(MOTOR_SIDE side) {
        return getTranslationTicksPerMM(side, null);
    }

    public float getTranslationTicksPerMM(MOTOR_SIDE side, MOTOR_END end) {
        if (!isAvailable()) {
            return 0.0f;
        }
        return config.motors[getIndex(side, end)].tranlationTicksPerMM;
    }

    public int getEncoder() {
        return getEncoder(null, null);
    }

    public int getEncoder(MOTOR_SIDE side) {
        return getEncoder(side, null);
    }

    public int getEncoder(MOTOR_SIDE side, MOTOR_END end) {
        return config.motors[getIndex(side, end)].motor.getEncoder();
    }

    public void setSpeed(float x, float y, float rotation) {
        this.setSpeed(x);
    }

    public void setSpeed(float speed) {
        setSpeed(speed, null);
    }

    public void setSpeed(float speed, MOTOR_SIDE side) {
        if (!isAvailable()) {
            return;
        }
        for (WheelMotor motor : config.motors) {
            if (side == null || motor.side == side) {
                motor.motor.setPower(limit(speed * speedScale));
            }
        }
    }

    public boolean isPositionPID() {
        boolean pid = true;
        for (int i = 0; i < config.motors.length; i++) {
            if (!config.motors[i].motor.isPositionPID()) {
                if (i > 0 && pid != false) {
                    telemetry.log().add(this.getClass().getSimpleName() + ": Inconsistent PID state");
                }
                pid = false;
            }
        }
        return pid;
    }

    public void setPositionPID(boolean enable) {
        DcMotor.RunMode mode = config.mode;
        if (enable) {
            mode = DcMotor.RunMode.RUN_TO_POSITION;
            setTeleop(false);
        }

        stop();
        for (int i = 0; i < config.motors.length; i++) {
            config.motors[i].motor.setMode(mode);
        }
    }

    public boolean onTarget() {
        boolean done = true;
        for (int i = 0; i < config.motors.length; i++) {
            if (!config.motors[i].motor.onTarget()) {
                done = false;
            }
        }
        return done;
    }

    public void setTarget(int target) {
        if (!isAvailable()) {
            return;
        }
        if (!isPositionPID()) {
            stop();
            telemetry.log().add(this.getClass().getSimpleName() + ": Position PID not active");
            return;
        }
        for (int i = 0; i < config.motors.length; i++) {
            config.motors[i].motor.setTarget(target);
        }
    }

    public void stop() {
        if (!isAvailable()) {
            return;
        }
        for (int i = 0; i < config.motors.length; i++) {
            config.motors[i].motor.setPower(0.0f);
        }
    }

    public boolean isTeleop() {
        return this.teleop;
    }

    public void setTeleop(boolean enabled) {
        if (this.teleop != enabled) {
            stop();
        }
        this.teleop = enabled;
    }

    public void setSpeedScale(float scale) {
        this.speedScale = limit(scale);
    }

    public void loop(Gamepad pad) {
        if (!isAvailable() || !isTeleop() || pad == null) {
            return;
        }

        // Negative is forward; this is typically the opposite of native motor config

        // arcade controls
        float left = limit(cleanJoystick(-pad.left_stick_y + pad.right_stick_x));
        this.setSpeed(left, MOTOR_SIDE.LEFT);

        float right = limit(cleanJoystick(-pad.left_stick_y - pad.right_stick_x));
        this.setSpeed(right, MOTOR_SIDE.RIGHT);
    }

    protected float limit(float input) {
        return com.qualcomm.robotcore.util.Range.clip(input, -1.0f, 1.0f);
    }

    protected float cleanJoystick(float stick) {
        if (Math.abs(stick) < JOYSTICK_DEADZONE) {
            return 0.0f;
        }

        float power = limit(stick);
        power = (float) Math.pow(power, JOYSTICK_EXPONENT);
        power = Math.copySign(power, stick);
        return power;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return null;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return null;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return null;
    }

    public void turnAsync(double angle) {

    }

    public void turn(double angle) {

    }

    public void followTrajectoryAsync(Trajectory trajectory) {

    }

    public void followTrajectory(Trajectory trajectory) {

    }

    public Pose2d getLastError() {
        return null;
    }

    public void update() {

    }

    public void waitForIdle() {

    }

    public boolean isBusy() {
        return false;
    }

    public void setMode(DcMotor.RunMode runMode) {

    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {

    }

    public void setWeightedDrivePower(Pose2d drivePower) {

    }

    public List<Double> getWheelPositions() {
        return null;
    }

    public List<Double> getWheelVelocities() {
        return null;
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {

    }

    public double getRawExternalHeading() {
        return 0;
    }
}
