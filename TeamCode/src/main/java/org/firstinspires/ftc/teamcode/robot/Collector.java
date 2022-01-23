/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

@Config
@Disabled
@TeleOp(name = "Collector", group = "Test")
public class Collector extends OpMode {
    // Hardware
    private Servo collectorArm = null;
    private DcMotor collector = null;
    //private DistanceSensor distance = null;
    private TouchSensor sensorCollector = null;

    // Config
    public static boolean DEBUG = false;
    public static double COLLECTOR_UP = 0.53;
    public static double COLLECTOR_DOWN = 0.90;
    private static double SPEED = 1;
    public static int DISTANCE = 30;
    public static double DELAY_TIME = 1;
    public static double EJECT_TIME = 2;

    // Members
    private boolean enabled = false;
    private InputHandler in;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime collectorTimer = new ElapsedTime();
    private collectCmd collectCmdState = collectCmd.IDLE;
    private boolean collected = false;

    @Override
    public void init() {
        // Collector
        try {
            collector = hardwareMap.get(DcMotor.class, "Collector");
            collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            collectorArm = hardwareMap.get(Servo.class, "CollectorArm");
            collectorArm.setPosition(COLLECTOR_UP);

            sensorCollector = hardwareMap.get(TouchSensor.class, "DC");
            //distance = hardwareMap.get(DistanceSensor.class, "DC");

            Globals.opmode = this;
            in = Globals.input(this);
            in.register("COLLECT", GAMEPAD.driver2, PAD_KEY.left_bumper);
            in.register("COLLECT1", GAMEPAD.driver1, PAD_KEY.left_bumper);

            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": " +
                    "Could not initialize");
        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        collectorArm.setPosition(COLLECTOR_UP);
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        in.loop();

        /*// Collector
        double spin = -gamepad2.left_stick_y;
        collector.setPower(Range.clip(spin, SPEED, -SPEED));*/

        // Arm
        telemetry.addData("collector pos", collectorArm.getPosition());
        telemetry.addData("collector pos", Math.round(collectorArm.getPosition() * 100.0)/100.0);
        telemetry.addData("State", collectCmdState);

        // Collector state
        collected = sensorCollector.isPressed();
        telemetry.addData("Collected? ", collected);
        switch (collectCmdState) {
            case IDLE:
                if (gamepad2.left_bumper) {
                    collectCmdState = collectCmd.BEFORE_COLLECT;
                }
                break;
            case BEFORE_COLLECT:
                collectorTimer.reset();
                collectCmdState = collectCmd.COLLECT;
                break;
            case COLLECT:
                if (!gamepad2.left_bumper) {
                    collectCmdState = collectCmd.BEFORE_EJECT;
                }
                if (collected && collectorTimer.seconds() > (Math.PI / 10)) {
                    collectorTimer.reset();
                    collectCmdState = collectCmd.SENSOR_DELAY;
                }
                break;
            case SENSOR_DELAY:
                collected = false;
                if (collectorTimer.seconds() > DELAY_TIME) {
                    collectCmdState = collectCmd.BEFORE_EJECT;
                }
                break;
            case BEFORE_EJECT:
                collectorTimer.reset();
                collectCmdState = collectCmd.EJECT;
                break;
            case EJECT:
                if (collectorTimer.seconds() > EJECT_TIME) {
                    collectCmdState = collectCmd.IDLE;
                }
                break;
        }

        // Collector commands
        switch (collectCmdState) {
            case IDLE:
                collectorArm.setPosition(COLLECTOR_UP);
                collector.setPower(0);
                break;
            case BEFORE_COLLECT:
            case COLLECT:
            case SENSOR_DELAY:
                collectorArm.setPosition(COLLECTOR_DOWN);
                collector.setPower(1);
                break;
            case BEFORE_EJECT:
                collectorArm.setPosition(COLLECTOR_UP);
                collector.setPower(1);
                break;
            case EJECT:
                collectorArm.setPosition(COLLECTOR_UP);
                collector.setPower(-1);
                break;
        }
        RobotLog.d("," + "Collector" + "," + getRuntime() + "," +
                gamepad2.left_bumper + "," + collected + "," +
                collectorArm.getPosition() + "," + collector.getPower() + "," +
                collectorTimer.seconds() + "," + collectCmdState);

        // Debug when requested
        if (DEBUG) {
            /*telemetry.addData("Collector Input", "R %.0f/%s",
                    range, inRange ? "+" : "-");*/
            telemetry.addData("Collector Output", "C %.2f/%d, A %.2f",
                    collector.getPower(), collector.getCurrentPosition(), collectorArm.getPosition());
        }
    }

    @Override
    public void stop() {
    }

    public void collect() {
        collectCmdState = collectCmd.BEFORE_COLLECT;
    }

    public void autoCollect() {
        collectCmdState = collectCmd.BEFORE_COLLECT;
        loop();
    }

    enum collectCmd {
        IDLE,
        BEFORE_COLLECT,
        COLLECT,
        SENSOR_DELAY,
        BEFORE_EJECT,
        EJECT
    }
}