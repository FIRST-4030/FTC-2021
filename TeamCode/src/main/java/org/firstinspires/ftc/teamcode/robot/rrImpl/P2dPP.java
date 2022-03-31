package org.firstinspires.ftc.teamcode.robot.rrImpl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.tfodohm.TFMaths.Vector2f;

public class P2dPP {

    public enum Pathing{
        STRAIGHT,
        SPLINE
    }

    public class Path{
        private Pose2d pose2d;
        private Vector2f end;
        private double endDir;

        public Path(Pose2d pose2d, Vector2f end, double endDir){
            this.pose2d = pose2d;
            this.end = end;
            this.endDir = endDir;
        }

        public double getLength(){
            return end.length();
        }

        public Vector2f getDeltaVector(){
            return new Vector2f((float)(end.x - pose2d.getX()), (float)(end.y - pose2d.getY()));
        }

        public void setEnd(Vector2f nEnd){
            this.end = nEnd;
        }

        public Vector2f getEnd(){
            return this.end;
        }

        public void setPose2d(Pose2d nPose){
            this.pose2d = nPose;
        }

        public Pose2d getPose2d(){
            return this.pose2d;
        }
    }

    private Path path;
    private Pathing pathing;

    public P2dPP(Path path, Pathing pathing){
        this.path = path;
        this.pathing = pathing;
    }

    public Trajectory process(SampleTankDrive drive){
        switch (this.pathing){
            case SPLINE:
                return drive.trajectoryBuilder(path.pose2d)
                        .splineTo(path.getDeltaVector().getAsVector2d(), path.endDir)
                        .build();
            default:
            case STRAIGHT:
                return drive.trajectoryBuilder(path.pose2d)
                        .forward(path.getLength())
                        .build();
        }
    }

    public Pose2d getPose(){
        return this.path.pose2d;
    }

    public Pathing getPathing(){
        return this.pathing;
    }

    public void setPose(Pose2d nPose){
        this.path.setPose2d(nPose);
    }

    public void setPathing(Pathing nPathing){
        this.pathing = nPathing;
    }

    public void dispose(){
        this.path = null;
        this.pathing = null;
    }
}
