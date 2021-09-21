package org.firstinspires.ftc.teamcode.robot.config;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.config.Configs;
import org.firstinspires.ftc.teamcode.robot.DISTANCE_SENSORS;
import org.firstinspires.ftc.teamcode.sensors.color_range.COLOR_RANGE_TYPES;
import org.firstinspires.ftc.teamcode.sensors.color_range.ColorRange;
import org.firstinspires.ftc.teamcode.sensors.color_range.ColorRangeConfig;
import org.firstinspires.ftc.teamcode.sensors.color_range.RevColorRange;
import org.firstinspires.ftc.teamcode.sensors.distance.DISTANCE_TYPES;
import org.firstinspires.ftc.teamcode.sensors.distance.Distance;
import org.firstinspires.ftc.teamcode.sensors.distance.DistanceConfig;
import org.firstinspires.ftc.teamcode.sensors.distance.RevDistance;

public class DistanceConfigs extends Configs {
    public DistanceConfigs(HardwareMap map, Telemetry telemetry, BOT bot) {
        super(map, telemetry, bot);
    }

    public Distance init(DISTANCE_SENSORS name) {
        DistanceConfig config = config(name);
        super.checkConfig(config, name);
        Distance distance = null;
        switch (config.type) {
            case REV_LASER:
                distance = new RevDistance(map, telemetry, config.name);
                break;
        }

        super.checkAvailable(distance, name);
        return distance;
    }

    public DistanceConfig config(DISTANCE_SENSORS sensor) {
        super.checkBOT();

        DistanceConfig config = null;
        switch (bot) {
            case ARM:
                switch (sensor) {
                    case TEST:
                        config = new DistanceConfig(DISTANCE_TYPES.REV_LASER, "test");
                        break;
                }
        }
        return config;
    }
}
