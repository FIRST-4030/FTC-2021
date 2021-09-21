package org.firstinspires.ftc.teamcode.sensors.led_matrix;

import org.firstinspires.ftc.teamcode.config.Config;

public class LEDMatrixConfig implements Config {
    public final String name;
    public final LED_MATRIX_TYPES type;

    public LEDMatrixConfig(LED_MATRIX_TYPES type, String name) {
        this.name = name;
        this.type = type;
    }
}
