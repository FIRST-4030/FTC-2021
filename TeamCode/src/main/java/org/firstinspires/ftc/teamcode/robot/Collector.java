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
import com.qualcomm.ftccommon.configuration.EditLegacyModuleControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

@Config
@TeleOp(name = "Collector", group = "Test")
public class Collector extends OpMode {
    // Hardware
    private Servo arm = null;
    private DcMotor collector = null;
    //private DistanceSensor distance = null;
    private TouchSensor touchSensor = null;

    // Config
    public static boolean DEBUG = false;
    public static double ARM_UP = 0.65;
    public static double ARM_DOWN = 0.90;
    public static double SPEED = 1;
    public static int DISTANCE = 10;
    public static double EJECT_TIME = 7;

    // Members
    private boolean enabled = false;
    private InputHandler in;
    private ElapsedTime runtime = new ElapsedTime();
    private AUTO_STATE state = AUTO_STATE.DONE;

    @Override
    public void init() {
        // Collector
        try {
            collector = hardwareMap.get(DcMotor.class, "Collector");
            collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            arm = hardwareMap.get(Servo.class, "CollectorArm");
            arm.setPosition(ARM_UP);

            touchSensor = hardwareMap.get(TouchSensor.class, "DC");
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

        arm.setPosition(ARM_UP);
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
        telemetry.addData("collector pos", arm.getPosition());
        telemetry.addData("collector pos", Math.round(arm.getPosition() * 100.0)/100.0);
        telemetry.addData("State", state);
        if (gamepad2.left_bumper && (Math.round(arm.getPosition() * 100.0)/100.0 == ARM_DOWN)) {
            telemetry.log().add("Going to up now");
            state = AUTO_STATE.UP;
            runtime.reset();
        } else if (gamepad2.left_bumper && (Math.round(arm.getPosition() * 100.0)/100.0 == ARM_UP)) {
            telemetry.log().add("Going to down now");
            state = AUTO_STATE.DOWN;
        }

        switch (state) {
            case DOWN:
                telemetry.log().add("In down now");
                arm.setPosition(ARM_DOWN);
                collector.setPower(SPEED);
                state = AUTO_STATE.DONE;
                break;
            case UP:
                telemetry.log().add("In up now");
                if (runtime.seconds() > 1.5 && runtime.seconds() < EJECT_TIME) {
                    arm.setPosition(ARM_UP);
                    collector.setPower(-SPEED);
                } else if (runtime.seconds() < 1.5 && runtime.seconds() < EJECT_TIME){
                    arm.setPosition(ARM_DOWN);
                    collector.setPower(SPEED);
                } else {
                    arm.setPosition(ARM_UP);
                    collector.setPower(0);
                    state = AUTO_STATE.DONE;
                }
                break;
            case DONE:
                break;
        }

        // Debug when requested
        if (DEBUG) {
            /*telemetry.addData("Collector Input", "R %.0f/%s",
                    range, inRange ? "+" : "-");*/
            telemetry.addData("Collector Output", "C %.2f/%d, A %.2f",
                    collector.getPower(), collector.getCurrentPosition(), arm.getPosition());
        }
    }

    @Override
    public void stop() {
    }

    enum AUTO_STATE implements OrderedEnum {
        DOWN,
        UP,
        DONE;

        public AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}