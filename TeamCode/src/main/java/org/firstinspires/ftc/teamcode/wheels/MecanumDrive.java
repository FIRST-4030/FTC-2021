package org.firstinspires.ftc.teamcode.wheels;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.List;

public class MecanumDrive extends TankDrive {

    public SampleMecanumDrive roadRunnerMecanum;

    public MecanumDrive(HardwareMap map, Telemetry telemetry, WheelsConfig config) {
        super(map, telemetry, config);
        DcMotorEx fl = getMotor(MOTOR_SIDE.LEFT, MOTOR_END.FRONT).get();
        DcMotorEx fr = getMotor(MOTOR_SIDE.RIGHT, MOTOR_END.FRONT).get();
        DcMotorEx bl = getMotor(MOTOR_SIDE.LEFT, MOTOR_END.BACK).get();
        DcMotorEx br = getMotor(MOTOR_SIDE.RIGHT, MOTOR_END.BACK).get();
        roadRunnerMecanum = new SampleMecanumDrive(map, fl, fr, bl, br);
    }

    public void setSpeed(float speed, MOTOR_SIDE side, MOTOR_END end) {
        if (side == null) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Null SIDE");
        }
        if (end == null) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": Null END");
        }
        if (!isAvailable()) {
            return;
        }
        for (WheelMotor motor : config.motors) {
            if (motor.side == side && motor.end == end) {
                motor.motor.setPower(speed * speedScale);
            }
        }
    }


    @Override
    public void setSpeed(float speed, MOTOR_SIDE side) {
        for (MOTOR_END end : MOTOR_END.values()) {
            setSpeed(speed, side, end);
        }
    }

    @Override
    public void setSpeed(float speed) {
        for (MOTOR_SIDE side : MOTOR_SIDE.values()) {
            setSpeed(speed, side);
        }
    }

    @Override
    public void setSpeed(float x, float y, float rotation) {

        // modified code from https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
        // from dmssargent
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        final float v1 = (float) (r * Math.cos(robotAngle)) + rotation;
        final float v2 = (float) (r * Math.sin(robotAngle)) - rotation;
        final float v3 = (float) (r * Math.sin(robotAngle)) + rotation;
        final float v4 = (float) (r * Math.cos(robotAngle)) - rotation;

        // except for this, this is mine
        setSpeed(v1, MOTOR_SIDE.LEFT, MOTOR_END.FRONT);
        setSpeed(v2, MOTOR_SIDE.RIGHT, MOTOR_END.FRONT);
        setSpeed(v3, MOTOR_SIDE.LEFT, MOTOR_END.BACK);
        setSpeed(v4, MOTOR_SIDE.RIGHT, MOTOR_END.BACK);
    }

    @Override
    public void loop(Gamepad pad) {
        // an unconventional implementation, because NO!! Lars wanted it so
        float lStickX = -cleanJoystick(pad.right_stick_x);
        float lStickY = -cleanJoystick(pad.left_stick_y);
        float rStickX = cleanJoystick(2.0f*pad.left_stick_x);

        setSpeed(rStickX, lStickY, lStickX);

        roadRunnerMecanum.update();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return roadRunnerMecanum.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return roadRunnerMecanum.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return roadRunnerMecanum.trajectoryBuilder(startPose, startHeading);
    }

    public void turnAsync(double angle) {
        roadRunnerMecanum.turnAsync(angle);
    }

    public void turn(double angle) {
        roadRunnerMecanum.turn(angle);
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        roadRunnerMecanum.followTrajectoryAsync(trajectory);
    }

    public void followTrajectory(Trajectory trajectory) {
        roadRunnerMecanum.followTrajectory(trajectory);
    }

    public Pose2d getLastError() {
        return roadRunnerMecanum.getLastError();
    }

    public void waitForIdle() {
        roadRunnerMecanum.waitForIdle();
    }

    public boolean isBusy() {
        return roadRunnerMecanum.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        roadRunnerMecanum.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        roadRunnerMecanum.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        roadRunnerMecanum.setPIDFCoefficients(runMode, coefficients);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        roadRunnerMecanum.setWeightedDrivePower(drivePower);
    }

    public List<Double> getWheelPositions() {
        return roadRunnerMecanum.getWheelPositions();
    }

    public List<Double> getWheelVelocities() {
        return roadRunnerMecanum.getWheelVelocities();
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        roadRunnerMecanum.setMotorPowers(v, v1, v2, v3);
    }

    public double getRawExternalHeading() {
        return roadRunnerMecanum.getRawExternalHeading();
    }
}
