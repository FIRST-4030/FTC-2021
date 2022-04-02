package org.firstinspires.ftc.teamcode.robot.rrImpl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.LoopUtil;

@Config
@Autonomous(name="RRIMPLTest", group="Test")
public class TankDriveTesting extends LoopUtil {

    //basic roadrunner stuff
    public static ModdedTankDrive drive;
    public static Pose2dRecorder estimatedPaths, currentPaths;
    public static Trajectory trajectory;
    public static TelemetryPacket tPacket;
    public static FtcDashboard dashboard;

    //poses
    public static Pose2d startingPose = new Pose2d(0, 0);
    public static double DISTANCE = 30;
    //public static Vector2d splineTarget = new Vector2d(9, 9);

    @Override
    public void opInit() {
        setUpdateCap(30);

        //roadrunner stuffs
        tPacket = new TelemetryPacket();
        drive = new ModdedTankDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        //classes so you can record the poses
        estimatedPaths = new Pose2dRecorder();
        currentPaths = new Pose2dRecorder();

        estimatedPaths.record(startingPose);
        currentPaths.record(startingPose);

        drive.setPoseEstimate(startingPose);
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {
        Trajectory trajectory = drive.trajectoryBuilder(startingPose)
                .forward(DISTANCE)
                .build();

        drive.followTrajectory(trajectory);
        drive.turn(Math.toRadians(-90));

        startingPose = trajectory.end().plus(new Pose2d(0, 0, Math.toRadians(-90)));
        estimatedPaths.record(startingPose);
        currentPaths.record(drive.getLastError());
    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
