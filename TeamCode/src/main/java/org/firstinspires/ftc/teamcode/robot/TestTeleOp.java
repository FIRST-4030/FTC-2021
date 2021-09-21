package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.buttons.BUTTON_TYPE;
import org.firstinspires.ftc.teamcode.buttons.ButtonHandler;
import org.firstinspires.ftc.teamcode.buttons.PAD_BUTTON;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.driveto.AutoDriver;
import org.firstinspires.ftc.teamcode.robot.common.Common;
import org.firstinspires.ftc.teamcode.sensors.led_matrix.driver.BLINKING_MODE;
import org.firstinspires.ftc.teamcode.utils.RateLimit;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp - Test", group = "Test")
public class TestTeleOp extends OpMode {

    // Devices and subsystems
    private Robot robot = null;
    private Common common = null;
    private ButtonHandler buttons;
    private AutoDriver driver = new AutoDriver();

    private byte[] FUCK = {
            (byte) 0b11111111, (byte) 0b00011000,
            (byte) 0b10000001, (byte) 0b00011000,
            (byte) 0b10000001, (byte) 0b00011000,
            (byte) 0b10000001, (byte) 0b11111111,
            (byte) 0b10000001, (byte) 0b11111111,
            (byte) 0b10000001, (byte) 0b00011000,
            (byte) 0b10000001, (byte) 0b00011000,
            (byte) 0b11111111, (byte) 0b00011000
    };

    @Override
    public void init() {
        // Placate drivers
        telemetry.addData(">", "NOT READY");
        telemetry.update();

        // Init the common tasks elements
        robot = new Robot(hardwareMap, telemetry);
        common = robot.common;
        robot.wheels.setTeleop(true);

        // Check robot
        if (robot.bot != BOT.TEST) {
            telemetry.log().add("Opmode not compatible with bot " + robot.bot);
            requestOpModeStop();
            return;
        }

        // make the brightness bearable
        robot.matrix.setBrightness(16);
        robot.matrix.setBlinking(BLINKING_MODE.OFF);
        robot.matrix.setDisplayBuffer(FUCK);
        robot.matrix.write();

        // Wait for the game to begin
        telemetry.addData(">", "Ready for game start");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

    }

    public void stop() {
        robot.matrix.reset();
    }

}