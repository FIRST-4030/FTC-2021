package org.firstinspires.ftc.teamcode.robot.common;

import org.firstinspires.ftc.teamcode.actuators.Motor;
import org.firstinspires.ftc.teamcode.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.driveto.PID;
import org.firstinspires.ftc.teamcode.driveto.PIDParams;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.sensors.switches.Switch;
import org.firstinspires.ftc.teamcode.utils.Background;

public class LiftArmClaw extends Background {

    public static final boolean DEBUG = false;

    protected boolean initialized = false;
    private Motor lift;
    private ServoFTC claw;
    private ServoFTC arm;

    public void init(Motor lift, ServoFTC claw, ServoFTC arm) {
        this.lift = lift;
        this.claw = claw;
        this.arm = arm;
        initialized = true;
    }

    @Override
    public void loop() {
        if (!isAvailable()) {
            return;
        }

        // Do something to disable driver input
        // This is a good reason to wrap all these bits in their own control class
        // Then you can call Arm.teleop(false) like we do with the Wheels class

        // Do some sort of driving
    }

    public boolean isAvailable() {
        return initialized;
    }
}
