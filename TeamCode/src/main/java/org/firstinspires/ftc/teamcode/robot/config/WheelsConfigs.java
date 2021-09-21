package org.firstinspires.ftc.teamcode.robot.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.config.Configs;
import org.firstinspires.ftc.teamcode.wheels.DRIVE_TYPE;
import org.firstinspires.ftc.teamcode.wheels.MOTOR_END;
import org.firstinspires.ftc.teamcode.wheels.MOTOR_SIDE;
import org.firstinspires.ftc.teamcode.wheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.wheels.WheelsConfig;
import org.firstinspires.ftc.teamcode.wheels.TankDrive;
import org.firstinspires.ftc.teamcode.wheels.WheelMotor;
import org.firstinspires.ftc.teamcode.wheels.Wheels;

public class WheelsConfigs extends Configs {
    private final static float DERATE = 1.0f;
    private final static float WC_MAX_RATE = 2.50f * DERATE;
    private final static float WC_TICKS_PER_MM = 1.360f;

    private final static float M_MAX_RATE = WC_MAX_RATE;
    private final static float M_TICKS_PER_MM = WC_TICKS_PER_MM;
    private final static float M_TRANSLATION_TICKS_PER_MM = 2.0f;

    public WheelsConfigs(HardwareMap map, Telemetry telemetry, BOT bot) {
        super(map, telemetry, bot);
    }

    public Wheels init() {
        WheelsConfig config = config();
        super.checkConfig(config);
        Wheels wheels = null;
        switch (config.type) {
            case TANK:
                wheels = new TankDrive(map, telemetry, config);
                break;
            case MECANUM:
                wheels = new MecanumDrive(map, telemetry, config);
                break;
        }
        super.checkAvailable(wheels);
        return wheels;
    }

    public WheelsConfig config() {
        super.checkBOT();

        WheelMotor[] motors;
        WheelsConfig config = null;
        switch (bot) {
            case PRODUCTION:
                motors = new WheelMotor[2];
                motors[0] = new WheelMotor("FR", MOTOR_SIDE.RIGHT, true);
                motors[1] = new WheelMotor("BL", MOTOR_SIDE.LEFT, false);
                config = new WheelsConfig(DRIVE_TYPE.TANK, motors, true, DcMotor.RunMode.RUN_USING_ENCODER);
                break;

            case ARM:
                motors = new WheelMotor[4];
                motors[0] = new WheelMotor("L1", MOTOR_SIDE.LEFT, MOTOR_END.FRONT, true,
                        WC_TICKS_PER_MM);
                motors[1] = new WheelMotor("L2", MOTOR_SIDE.LEFT, MOTOR_END.BACK, true,
                        WC_TICKS_PER_MM);
                motors[2] = new WheelMotor("R1", MOTOR_SIDE.RIGHT, MOTOR_END.FRONT, false,
                        WC_TICKS_PER_MM);
                motors[3] = new WheelMotor("R2", MOTOR_SIDE.RIGHT, MOTOR_END.BACK, false,
                        WC_TICKS_PER_MM);
                config = new WheelsConfig(DRIVE_TYPE.TANK, motors, true, DcMotor.RunMode.RUN_USING_ENCODER);
                break;

            case TEST:
                motors = new WheelMotor[4];
                motors[0] = new WheelMotor("slob", MOTOR_SIDE.LEFT, MOTOR_END.FRONT, false,
                        M_TICKS_PER_MM, M_TRANSLATION_TICKS_PER_MM);
                motors[1] = new WheelMotor("on", MOTOR_SIDE.LEFT, MOTOR_END.BACK, false,
                        M_TICKS_PER_MM, M_TRANSLATION_TICKS_PER_MM);
                motors[2] = new WheelMotor("my", MOTOR_SIDE.RIGHT, MOTOR_END.FRONT, true,
                        M_TICKS_PER_MM, M_TRANSLATION_TICKS_PER_MM);
                motors[3] = new WheelMotor("hog", MOTOR_SIDE.RIGHT, MOTOR_END.BACK, true,
                        M_TICKS_PER_MM, M_TRANSLATION_TICKS_PER_MM);
                config = new WheelsConfig(DRIVE_TYPE.MECANUM, motors, true, DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case MECANUM:
                motors = new WheelMotor[4];
                motors[0] = new WheelMotor("FL", MOTOR_SIDE.LEFT, MOTOR_END.FRONT, true,
                        WC_TICKS_PER_MM);
                motors[1] = new WheelMotor("BL", MOTOR_SIDE.LEFT, MOTOR_END.BACK, true,
                        WC_TICKS_PER_MM);
                motors[2] = new WheelMotor("FR", MOTOR_SIDE.RIGHT, MOTOR_END.FRONT, false,
                        WC_TICKS_PER_MM);
                motors[3] = new WheelMotor("BR", MOTOR_SIDE.RIGHT, MOTOR_END.BACK, false,
                        WC_TICKS_PER_MM);
                config = new WheelsConfig(DRIVE_TYPE.MECANUM, motors, true, DcMotor.RunMode.RUN_USING_ENCODER);
                break;

        }
        return config;
    }
}
