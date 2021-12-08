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
    private DistanceSensor distance = null;

    // Config
    public static boolean DEBUG = false;
    public static double ARM_UP = 0.37;
    public static double ARM_DOWN = 0.9;
    public static double SPEED = -1;
    public static int DISTANCE = 7;
    public static double EJECT_TIME = 2;

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

            distance = hardwareMap.get(DistanceSensor.class, "DC");

            in = Globals.input(this);
            in.register("COLLECT", GAMEPAD.driver2, PAD_KEY.right_bumper);

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

        // Distance
        double range = distance.getDistance(DistanceUnit.MM);
        boolean inRange = (range <= DISTANCE);

        /*// Collector
        double spin = -gamepad2.left_stick_y;
        collector.setPower(Range.clip(spin, SPEED, -SPEED));*/

        // Arm
        if ((inRange || gamepad2.right_bumper) && arm.getPosition() == ARM_DOWN) {
            state = AUTO_STATE.UP;
        } else if (gamepad2.right_bumper && arm.getPosition() == ARM_UP) {
            state = AUTO_STATE.DOWN;
        }

        switch (state) {
            case DOWN:
                arm.setPosition(ARM_DOWN);
                collector.setPower(SPEED);
                state = AUTO_STATE.DONE;
                break;
            case UP:
                runtime.reset();
                if (runtime.seconds() < EJECT_TIME) {
                    arm.setPosition(ARM_UP);
                    collector.setPower(-SPEED);
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
            telemetry.addData("Collector Input", "R %.0f/%s",
                    range, inRange ? "+" : "-");
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