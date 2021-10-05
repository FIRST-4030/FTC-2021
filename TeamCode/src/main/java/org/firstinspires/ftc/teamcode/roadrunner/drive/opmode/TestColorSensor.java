package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class TestColorSensor extends LinearOpMode {
    //Define a variable for our color snesor
    ColorSensor color;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "C1");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values
        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            if (color.blue() <= color.red() * 0.8 && color.blue() <= color.green() *0.8) {
                telemetry.addData("Color Detected: ", "Yellow");
            } else {
                telemetry.addData("Color Detected: ", "Not Yellow");
            }
            telemetry.update();
        }
    }
}
