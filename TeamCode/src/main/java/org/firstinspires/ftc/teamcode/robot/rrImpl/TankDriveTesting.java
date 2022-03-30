package org.firstinspires.ftc.teamcode.robot.rrImpl;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.LoopUtil;

@Config
@Autonomous(name="RRIMPLTest", group="Test")
public class TankDriveTesting extends LoopUtil {

    SampleTankDrive drive;

    @Override
    public void opInit() {
        drive = new SampleTankDrive(hardwareMap);
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
