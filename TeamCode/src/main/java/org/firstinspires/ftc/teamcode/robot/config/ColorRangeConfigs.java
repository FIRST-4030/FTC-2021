package org.firstinspires.ftc.teamcode.robot.config;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.config.Configs;
import org.firstinspires.ftc.teamcode.sensors.color_range.COLOR_RANGE_TYPES;
import org.firstinspires.ftc.teamcode.sensors.color_range.ColorRange;
import org.firstinspires.ftc.teamcode.sensors.color_range.ColorRangeConfig;
import org.firstinspires.ftc.teamcode.sensors.color_range.RevColorRange;

public class ColorRangeConfigs extends Configs {
    public ColorRangeConfigs(HardwareMap map, Telemetry telemetry, BOT bot) {
        super(map, telemetry, bot);
    }

    public ColorRange init() {
        ColorRangeConfig config = config();
        super.checkConfig(config);
        ColorRange color = null;
        switch (config.type) {
            case REV:
                color = new RevColorRange(map, telemetry, config.name);
                break;
        }

        super.checkAvailable(color);
        return color;
    }

    public ColorRangeConfig config() {
        super.checkBOT();

        ColorRangeConfig config = null;
        switch (bot) {
            case PRODUCTION:
                config = new ColorRangeConfig(COLOR_RANGE_TYPES.REV, "ColorRange");
                break;
        }
        return config;
    }
}
