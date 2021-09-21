package org.firstinspires.ftc.teamcode.wheels;
//FL, FR
//BL, BR
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actuators.Motor;
import org.firstinspires.ftc.teamcode.config.Config;

public class WheelsConfig implements Config {
    public WheelMotor[] motors;
    public DRIVE_TYPE type;
    public DcMotor.RunMode mode;

    public WheelsConfig(DRIVE_TYPE type, WheelMotor[] motors, boolean brake, DcMotor.RunMode mode) {
        if (motors == null) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() + ": No motors provided");
        }
        if (mode == null) {
            mode = Motor.DEFAULT_MODE;
        }

        int minMotors = 0;
        switch (type) {
            case TANK:
                minMotors = 2;
                break;
            case MECANUM:
                minMotors = 4;
                break;
        }
        if (motors.length < minMotors) {
            throw new IllegalArgumentException(this.getClass().getSimpleName() +
                    ": Not enough motors configured " + motors.length + "/" + minMotors);
        }

        this.type = type;
        this.motors = motors;
        this.mode = mode;

        for (WheelMotor motor : motors) {
            if (motor == null || motor.motor == null) {
                continue;
            }
            motor.motor.setMode(mode);
            motor.motor.setBrake(brake);
        }
    }

    public WheelsConfig(DRIVE_TYPE type, WheelMotor[] motors, boolean brake) {
        this(type, motors, brake, null);
    }
}