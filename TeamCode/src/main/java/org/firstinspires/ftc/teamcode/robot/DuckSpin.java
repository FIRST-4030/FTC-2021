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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

@Config
@TeleOp(name = "DuckSpin", group = "Test")
public class DuckSpin extends OpMode {
    // Hardware
    private DcMotor duck = null;

    // Config
    public static double speedMin = 0.63;  // min duck spinner speed (0 - 1.0)
    public static double speedMax = 0.88;  // max duck spinner speed (0 - 1.0)
    public static double rampTime = 1.25;  // duck spinner ramp time (seconds, >0)
    public static double autoSpeedMin = 0.46;
    public static double autoSpeedMax = 0.66;
    public static double autoRampTime = 1.65;

    // Members
    public static boolean DEBUG = false;
    private InputHandler in = null;
    private boolean enabled = false;
    private double speed = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private AUTO_STATE state = AUTO_STATE.DONE;

    @Override
    public void init() {
        // Pull in Globals
        in = Globals.input(this);

        // Duck spinner
        try {
            duck = hardwareMap.get(DcMotor.class, "duck");
            duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": " +
                    "Could not initialize");
        }

        // Inputs
        in.register("DUCK_RED", GAMEPAD.driver1, PAD_KEY.b);
        in.register("DUCK_BLUE", GAMEPAD.driver1, PAD_KEY.a);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    public void auto(boolean red) {
        state = red ? AUTO_STATE.TELEOP_RED : AUTO_STATE.TELEOP_BLUE;
    }

    public boolean isDone() {
        return (state == AUTO_STATE.DONE);
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        // Inputs
        in.loop();

        // Override the current auto state with driver commands
        // Technically we should only trigger on button-down, not repeatedly while held
        if (in.down("DUCK_RED")) {
            auto(true);
        } else if (in.down("DUCK_BLUE")) {
            auto(false);
        }

        // Run the auto cycle (including translated driver commands)
        switch (state) {
            case TELEOP_RED:
                // Start backward
                speed = -speedMin;
                timer.reset();
                state = AUTO_STATE.TELEOP_SPIN; // Jump to a specific state
                break;
            case TELEOP_BLUE:
                // Start forward
                speed = speedMin;
                timer.reset();
                state = AUTO_STATE.TELEOP_SPIN; // Jump to a specific state
                break;
            case TELEOP_SPIN:
                // Run the spin cycle
                if (speed != 0 && timer.seconds() < rampTime) {
                    speed = (speedMin + (timer.seconds() / rampTime) *
                            (speedMax - speedMin)) * Math.signum(speed);
                } else {
                    speed = 0;
                    state = state.next(); // Jump to the next state in the enum list
                }
                duck.setPower(speed);
                break;
            case AUTO_RED:
                // Start backward
                speed = -autoSpeedMin;
                timer.reset();
                state = AUTO_STATE.AUTO_SPIN; // Jump to a specific state
                break;
            case AUTO_BLUE:
                // Start forward
                speed = autoSpeedMin;
                timer.reset();
                state = AUTO_STATE.AUTO_SPIN; // Jump to a specific state
                break;
            case AUTO_SPIN:
                // Run the spin cycle
                if (speed != 0 && timer.seconds() < autoRampTime) {
                    speed = (autoSpeedMin + (timer.seconds() / autoRampTime) *
                            (autoSpeedMax - autoSpeedMin)) * Math.signum(speed);
                } else {
                    speed = 0;
                    state = state.next(); // Jump to the next state in the enum list
                }
                duck.setPower(speed);
                break;
            case DONE:
                break;
        }

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Duck State", state);
            telemetry.addData("Duck Output", "%.2f/%d",
                    duck.getPower(), duck.getCurrentPosition());
        }

        telemetry.addData("speedMin:", speedMin);
        telemetry.addData("speedMax:", speedMax);
        // Moving the servo position and number should increase
        if (gamepad1.dpad_up) {
            speedMin += 0.01;
            speedMin = Math.min(1.0f, speedMin);
            speedMax += 0.01;
            speedMax = Math.min(1.0f, speedMax);
            // Moving the servo position and number should decrease
        } else if (gamepad1.dpad_down) {
            speedMin -= 0.01;
            speedMin = Math.max(0.0f, speedMin);
            speedMax -= 0.01;
            speedMax = Math.max(0.0f, speedMax);
        }
    }

    @Override
    public void stop() {
    }

    enum AUTO_STATE implements OrderedEnum {
        AUTO_RED,
        AUTO_BLUE,
        AUTO_SPIN,
        TELEOP_RED,
        TELEOP_BLUE,
        TELEOP_SPIN,
        DONE;

        public AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}