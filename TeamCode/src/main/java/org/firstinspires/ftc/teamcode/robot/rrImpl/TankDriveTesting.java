package org.firstinspires.ftc.teamcode.robot.rrImpl;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.utils.LoopUtil;

import java.util.HashMap;

@Config
@Autonomous(name="RRIMPLTest", group="Test")
public class TankDriveTesting extends LoopUtil {

    private enum Pathing{
        STRAIGHT,
        SPLINE
    }

    public static SampleTankDrive drive;
    public static Path2dRecorder estimatedPaths, currentPaths;
    public static Trajectory trajectory;
    public static TelemetryPacket tPacket;

    @Override
    public void opInit() {
        setUpdateCap(30);

        //roadrunner stuffs
        tPacket = new TelemetryPacket();
        drive = new SampleTankDrive(hardwareMap);

        //classes so you can record the poses
        estimatedPaths = new Path2dRecorder();
        currentPaths = new Path2dRecorder();
    }

    @Override
    public void opInitLoop() {

    }

    @Override
    public void opStart() {

    }

    @Override
    public void opUpdate(double deltaTime) {

    }

    @Override
    public void opFixedUpdate(double deltaTime) {

    }

    @Override
    public void opStop() {

    }
}
