package org.firstinspires.ftc.teamcode.robot.rrImpl;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class Path2dRecorder {

    private List<P2dPP> recordedPoses;

    public Path2dRecorder(){
        this.recordedPoses = new ArrayList<>();
    }

    public void record(P2dPP path){
        recordedPoses.add(path);
    }

    public void recordAll(P2dPP[] pose2ds){
        for (P2dPP path : pose2ds){
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

    public void replaceAt(int index, P2dPP newPath){
        this.recordedPoses.set(index, newPath);
    }

    public void replaceIn(int offset, P2dPP[] paths){
        for (int i = 0; i < paths.length; i++){
            this.recordedPoses.set(i + offset, paths[i]);
        }
    }

    public List<P2dPP> getAsList(){
        return this.recordedPoses;
    }

    public P2dPP[] getAsArray(){
        P2dPP[] output = new P2dPP[this.recordedPoses.size()];
        for (int i = 0; i < this.recordedPoses.size(); i++){
            output[i] = this.recordedPoses.get(i);
        }
        return output;
    }

    public Pose2d[] getAsPoseArray(){
        Pose2d[] output = new Pose2d[this.recordedPoses.size()];
        for (int i = 0; i < recordedPoses.size(); i++){
            output[i] = recordedPoses.get(i).getPose();
        }
        return output;
    }
}
