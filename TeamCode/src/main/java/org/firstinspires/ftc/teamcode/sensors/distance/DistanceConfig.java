package org.firstinspires.ftc.teamcode.sensors.distance;

import org.firstinspires.ftc.teamcode.config.Config;
import org.firstinspires.ftc.teamcode.sensors.color_range.COLOR_RANGE_TYPES;

public class DistanceConfig implements Config {
    public final String name;
    public final DISTANCE_TYPES type;

    public DistanceConfig(DISTANCE_TYPES type, String name) {
        this.name = name;
        this.type = type;
    }
}
