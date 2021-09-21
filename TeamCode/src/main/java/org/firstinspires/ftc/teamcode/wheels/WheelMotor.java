package org.firstinspires.ftc.teamcode.wheels;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actuators.Motor;
import org.firstinspires.ftc.teamcode.actuators.MotorConfig;
import org.firstinspires.ftc.teamcode.driveto.PIDParams;
import org.firstinspires.ftc.teamcode.utils.Available;

public class WheelMotor extends MotorConfig implements Available {
    public final MOTOR_SIDE side;
    public final MOTOR_END end;
    public Motor motor;
    public final float ticksPerMM;
    public final float tranlationTicksPerMM;

    public WheelMotor(String name, MOTOR_SIDE side, MOTOR_END end, boolean reverse,
                      float ticksPerMM, float translationTicksPerMM, boolean brake, DcMotor.RunMode mode) {
        super(name, reverse, brake, mode);
        this.side = side;
        this.end = end;
        this.motor = null;
        this.ticksPerMM = ticksPerMM;
        this.tranlationTicksPerMM = translationTicksPerMM;
    }

    public WheelMotor(String name, MOTOR_SIDE side, boolean reverse) {
        this(name, side, MOTOR_END.FRONT, reverse, 1.0f, 0.0f, true, null);
    }

    public WheelMotor(String name, MOTOR_SIDE side, MOTOR_END end, boolean reverse, float ticksPerMM, float translationTicksPerMM) {
        this(name, side, end, reverse, ticksPerMM, translationTicksPerMM, true, null);
    }

    public WheelMotor(String name, MOTOR_SIDE side, MOTOR_END end, boolean reverse, float ticksPerMM) {
        this(name, side, end, reverse, ticksPerMM, 0.0f, true, null);
    }

    public WheelMotor(String name, MOTOR_SIDE side, boolean reverse, float ticksPerMM) {
        this(name, side, MOTOR_END.FRONT, reverse, ticksPerMM, 0.0f, true, null);
    }
    public Motor get(){
        return motor;
    }

    public boolean isAvailable() {
        return this.motor != null;
    }
}
