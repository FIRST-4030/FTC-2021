package org.firstinspires.ftc.teamcode.robot.rrImpl;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class Path2dRecorder {

    private List<Pose2d> recordedPoses;

    public Path2dRecorder(){
        this.recordedPoses = new ArrayList<>();
    }

    public void record(Pose2d path){
        recordedPoses.add(path);
    }

    public void recordAll(Pose2d[] pose2ds){
        for (Pose2d path : pose2ds){
            recordedPoses.add(path);
        }
    }

    public void removeAt(int index){
        recordedPoses.remove(index);
    }

    public void removeLatest(){
        recordedPoses.remove(recordedPoses.size() - 1);
    }

    public void removeAll(){
        recordedPoses.clear();
    }

    public void replaceAt(int index, Pose2d newPath){
        this.recordedPoses.set(index, newPath);
    }

    public void replaceIn(int offset, Pose2d[] paths){
        for (int i = 0; i < paths.length; i++){
            this.recordedPoses.set(i + offset, paths[i]);
        }
    }

    public List<Pose2d> getAsList(){
        return this.recordedPoses;
    }

    public Pose2d[] getAsArray(){
        Pose2d[] output = new Pose2d[this.recordedPoses.size()];
        for (int i = 0; i < this.recordedPoses.size(); i++){
            output[i] = this.recordedPoses.get(i);
        }
        return output;
    }
}
