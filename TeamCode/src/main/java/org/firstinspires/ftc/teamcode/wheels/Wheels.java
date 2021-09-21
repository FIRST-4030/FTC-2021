package org.firstinspires.ftc.teamcode.wheels;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.actuators.Motor;
import org.firstinspires.ftc.teamcode.utils.Available;

import java.util.List;

public interface Wheels extends Available {
    void stop();

    void setSpeed(float speed);

    void setSpeed(float x, float y, float rotation);

    void setSpeed(float speed, MOTOR_SIDE side);

    float getTicksPerMM();

    float getTicksPerMM(MOTOR_SIDE side);

    float getTicksPerMM(MOTOR_SIDE side, MOTOR_END end);

    float getTranslationTicksPerMM();

    float getTranslationTicksPerMM(MOTOR_SIDE side);

    float getTranslationTicksPerMM(MOTOR_SIDE side, MOTOR_END end);

    Motor getMotor(MOTOR_SIDE side, MOTOR_END end);

    Motor getMotor(MOTOR_SIDE side);

    int getEncoder();

    int getEncoder(MOTOR_SIDE side);

    int getEncoder(MOTOR_SIDE side, MOTOR_END end);

    void resetEncoder();

    void resetEncoder(MOTOR_SIDE side);

    void resetEncoder(MOTOR_SIDE side, MOTOR_END end);

    void loop(Gamepad pad);

    boolean isTeleop();

    void setTeleop(boolean enabled);

    void setSpeedScale(float scale);

    void setPositionPID(boolean enabled);

    boolean isPositionPID();

    boolean onTarget();

    void setTarget(int target);

    TrajectoryBuilder trajectoryBuilder(Pose2d startPose);

    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed);

    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading);

    void turnAsync(double angle);

    void turn(double angle);

    void followTrajectoryAsync(Trajectory trajectory);

    void followTrajectory(Trajectory trajectory);

    Pose2d getLastError();

    void waitForIdle();

    boolean isBusy();

    void setMode(DcMotor.RunMode runMode);

    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);

    void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients);

    void setWeightedDrivePower(Pose2d drivePower);

    List<Double> getWheelPositions();

    List<Double> getWheelVelocities();

    void setMotorPowers(double v, double v1, double v2, double v3);

    double getRawExternalHeading();
}
