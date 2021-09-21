package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.teamcode.actuators.Motor;
import org.firstinspires.ftc.teamcode.actuators.ServoFTC;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.common.Common;
import org.firstinspires.ftc.teamcode.robot.config.ColorRangeConfigs;
import org.firstinspires.ftc.teamcode.robot.config.DistanceConfigs;
import org.firstinspires.ftc.teamcode.robot.config.GyroConfigs;
import org.firstinspires.ftc.teamcode.robot.config.LEDMatrixConfigs;
import org.firstinspires.ftc.teamcode.robot.config.MotorConfigs;
import org.firstinspires.ftc.teamcode.robot.config.PIDMotorConfigs;
import org.firstinspires.ftc.teamcode.robot.config.ServoConfigs;
import org.firstinspires.ftc.teamcode.robot.config.SwitchConfigs;
import org.firstinspires.ftc.teamcode.robot.config.WheelsConfigs;
import org.firstinspires.ftc.teamcode.sensors.color_range.ColorRange;
import org.firstinspires.ftc.teamcode.sensors.distance.DISTANCE_TYPES;
import org.firstinspires.ftc.teamcode.sensors.distance.Distance;
import org.firstinspires.ftc.teamcode.sensors.distance.RevDistance;
import org.firstinspires.ftc.teamcode.sensors.gyro.Gyro;
import org.firstinspires.ftc.teamcode.sensors.led_matrix.LEDMatrix;
import org.firstinspires.ftc.teamcode.vuforia.VuforiaFTC;
import org.firstinspires.ftc.teamcode.wheels.Odometry;
import org.firstinspires.ftc.teamcode.wheels.Wheels;
//tim is a massive dummy -the robot
public class Robot {
    public static Robot robot = null;
    public final Common common;
    public final BOT bot;
    public final HardwareMap map;
    public final Telemetry telemetry;
    public StandardTrackingWheelLocalizer odometry;

    // Shared
    public final Gyro gyro;
    public final Wheels wheels;
    public final VuforiaFTC vuforia;
    public ServoFTC claw;

    // Production
    // Motors and Servos
    public Motor collectorBack;
    public Motor collectorFront;
    public Motor shooter;
    public ServoFTC wobbleGoalArm;
    public ServoFTC backRaiseLower;
    public ServoFTC frontRaiseLower;
    public ServoFTC queue;
    public ServoFTC wobbleGoalGrip;
    public ServoFTC queueFlipper;

    // Mecanum
    public ServoFTC flag;

    // Arm
    public ServoFTC lower;
    public ServoFTC upper;
    public ServoFTC rotation;
    public ServoFTC wrist;

    // Test
    public LEDMatrix matrix;

    // Blank
    public Motor motoryBoi;
    public ServoFTC servoBoi;


    public Robot(HardwareMap map, Telemetry telemetry) {
        this(map, telemetry, null);
    }

    public Robot(HardwareMap map, Telemetry telemetry, BOT bot) {
        robot = this;
        this.map = map;
        this.telemetry = telemetry;
        if (bot == null) {
            bot = BOT.MECANUM;
            //bot = detectBot();
        }
        this.bot = bot;
        odometry = new StandardTrackingWheelLocalizer(map);

        GyroConfigs gyros = new GyroConfigs(map, telemetry, bot);
        WheelsConfigs wheels = new WheelsConfigs(map, telemetry, bot);
        MotorConfigs motors = new MotorConfigs(map, telemetry, bot);
        PIDMotorConfigs pids = new PIDMotorConfigs(map, telemetry, bot);
        ServoConfigs servos = new ServoConfigs(map, telemetry, bot);
        SwitchConfigs switches = new SwitchConfigs(map, telemetry, bot);
        DistanceConfigs distances = new DistanceConfigs(map, telemetry, bot);
        ColorRangeConfigs colors = new ColorRangeConfigs(map, telemetry, bot);
        LEDMatrixConfigs ledMatrices = new LEDMatrixConfigs(map, telemetry, bot);

        // Shared
        this.wheels = wheels.init();
        this.wheels.stop();
        gyro = gyros.init();
        vuforia = new VuforiaFTC(map, telemetry, bot);

        // Bot specific
        switch (bot) {
            case PRODUCTION:
                odometry = new StandardTrackingWheelLocalizer(map);
                wobbleGoalArm = servos.init(SERVOS.WOBBLE_GOAL_ARM);
                wobbleGoalGrip = servos.init(SERVOS.WOBBLE_GOAL_GRIP);
                queueFlipper = servos.init(SERVOS.QUEUE_FLIPPER);
                collectorBack = motors.init(MOTORS.COLLECTOR_BACK);
                collectorFront = motors.init(MOTORS.COLLECTOR_FRONT);
                backRaiseLower = servos.init(SERVOS.BACK_RAISE_LOWER);
                frontRaiseLower = servos.init(SERVOS.FRONT_RAISE_LOWER);
                shooter = motors.init(MOTORS.SHOOTER);
                queue = servos.init(SERVOS.QUEUE);
                shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                break;

            case ARM:
             //   lower = servos.init(SERVOS.LOWER);
             //   upper = servos.init(SERVOS.UPPER);
             //   rotation = servos.init(SERVOS.ROTATION);
             //   claw = servos.init(SERVOS.CLAW);
             //   wrist = servos.init(SERVOS.WRIST);
                break;

            case TEST:
                matrix = ledMatrices.init(LED_MATRICIES.TEST);
                break;

            case MECANUM:
                flag = servos.init(SERVOS.FLAG);
                break;
        }


        this.common = new Common(this);
    }

    public BOT detectBot() {
        // Try WheelsConfigs from each bot until something succeeds
        BOT bot = null;
        for (BOT b : BOT.values()) {
            WheelsConfigs wheels = new WheelsConfigs(map, telemetry, b);
            Wheels w = wheels.init();
            if (w != null && w.isAvailable()) {
                bot = b;
                break;
            }
        }
        if (bot == null) {
            bot = BOT.values()[0];
            telemetry.log().add("BOT detection failed. Default: " + bot);
        }
        if (bot.ordinal() != 0) {
            telemetry.log().add("Using BOT: " + bot);
        }
        return bot;
    }
}
