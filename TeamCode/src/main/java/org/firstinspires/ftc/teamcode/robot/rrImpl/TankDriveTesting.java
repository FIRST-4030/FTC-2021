package org.firstinspires.ftc.teamcode.robot.rrImpl;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class TankDriveTesting extends OpMode {
    private TankDriveTestingConstants tConsts = new TankDriveTestingConstants();

    public TankDriveTesting(){

    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}

class TankDriveTestingConstants{
    public final double TICKS_PER_REV = 537.6;
    public final double MAX_RPM = 340;
}