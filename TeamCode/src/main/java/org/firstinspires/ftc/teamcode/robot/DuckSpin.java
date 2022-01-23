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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;

@Config
//@Disabled
@TeleOp(name = "DuckSpin", group = "Test")
public class DuckSpin extends OpMode {
    // Hardware
    private DcMotor duck = null;
    private DcMotor duck2 = null;

    // Config
    public static double teleopMin = 0.72;
    public static double teleopMax = 1;
    public static double teleopRamp = 1.2;
    public static double autoMin = 0.445;
    public static double autoMax = 0.645;
    public static double autoRamp = 1.71;

    public static double pow = 13;
    private boolean started;
    private boolean done;

    // Members
    public static boolean DEBUG = false;
    private InputHandler in = null;
    private boolean enabled = false;
    private double speed = 0.0;
    public static double speedMin = teleopMin;  // min duck spinner speed (0 - 1.0)
    public static double speedMax = teleopMax;  // max duck spinner speed (0 - 1.0)
    public static double rampTime = teleopRamp;  // duck spinner ramp time (seconds, >0)
    private final ElapsedTime timer = new ElapsedTime();
    private AUTO_STATE state = AUTO_STATE.IDLE;

    @Override
    public void init() {
        // Pull in Globals
        Globals.opmode = this;
        in = Globals.input(this);

        // Duck spinner
        try {
            duck = hardwareMap.get(DcMotor.class, "duck");
            duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            duck2 = hardwareMap.get(DcMotor.class, "duck2");
            duck2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void teleop(boolean red) {
        loop();
        speedMin = teleopMin;
        speedMax = teleopMax;
        rampTime = teleopRamp;
        state = red ? AUTO_STATE.RED : AUTO_STATE.BLUE;
    }

    public void auto(boolean red) {
        loop();
        speedMin = autoMin;
        speedMax = autoMax;
        rampTime = autoRamp;
        if (state == AUTO_STATE.IDLE) {
            state = red ? AUTO_STATE.RED : AUTO_STATE.BLUE;
        }
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
            teleop(true);
        } else if (in.down("DUCK_BLUE")) {
            teleop(false);
        }

        // Run the auto cycle (including translated driver commands)
        switch (state) {
            case IDLE:
                break;
            case RED:
                // Start backward
                speed = -speedMin;
                timer.reset();
                state = AUTO_STATE.SPIN_RED; // Jump to a specific state
                break;
            case BLUE:
                // Start forward
                speed = speedMin;
                timer.reset();
                state = AUTO_STATE.SPIN_BLUE; // Jump to a specific state
                break;
            case SPIN_BLUE:
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
            case SPIN_RED:
                // Run the spin cycle
                if (speed != 0 && timer.seconds() < rampTime) {
                    speed = (speedMin + (timer.seconds() / rampTime) *
                            (speedMax - speedMin)) * Math.signum(speed);
                } else {
                    speed = 0;
                    state = state.next(); // Jump to the next state in the enum list
                }
                duck2.setPower(speed);
                break;
            case NEW_SPIN1:
                duckRampS(-teleopMin, -teleopMax, teleopRamp, true);
                if (done && duck.getPower() == 0) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case NEW_SPIN2:
                duckRampS(-teleopMin, -teleopMax, teleopRamp, false);
                if (done && duck.getPower() == 0) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case NEW_SPIN3:
                duckRampInvS(-teleopMin, -teleopMax, teleopRamp, true);
                if (done && duck.getPower() == 0) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case NEW_SPIN4:
                duckRampInvS(-teleopMin, -teleopMax, teleopRamp, false);
                if (done && duck.getPower() == 0) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case NEW_SPIN5:
                duckRampPoly(-teleopMin, -teleopMax, teleopRamp, pow);
                if (done && duck.getPower() == 0) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case DONE:
                duck.setPower(0);
                duck2.setPower(0);
                break;
        }

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Duck State", state);
            telemetry.addData("Duck Output", "%.2f/%d",
                    duck.getPower(), duck.getCurrentPosition());
        }

        telemetry.addData("speedMin:", teleopMin);
        telemetry.addData("speedMax:", teleopMax);
        /*
         * TODO: This is a great place to use the InputHandler.auto() method
         ** It's specifically meant for this sort of incremental manual control
         ** auto() lets this bit of code trigger repeatedly while the key is held
         ** but also limits the repeat rate so it doesn't go too fast
         ** if (in.auto("SPEED_UP")) {
         **   speedMax = Math.min(1.0f, speedMax + 0.01);
         **   speedMin = Math.min(1.0f, speedMin + 0.01);
         ** }
         ** You can adjust the key-repeat rate with a call like this:
         ** in.get("SPEED_UP").setAutoDelay(50);
         *
         * You should also be able to adjust speedMax/speedMin directly in the dashboard
         * Tuning controls on the gamepad can be useful, but the dashboard requires less code
         */

        if (gamepad1.x) {
            state = AUTO_STATE.NEW_SPIN1;
        }
        if (gamepad1.y) {
            state = AUTO_STATE.NEW_SPIN2;
        }
        if (gamepad1.left_bumper) {
            state = AUTO_STATE.NEW_SPIN3;
        }
        if (gamepad1.right_bumper) {
            state = AUTO_STATE.NEW_SPIN4;
        }
        if (gamepad1.dpad_up) {
            state = AUTO_STATE.NEW_SPIN5;
        }
    }

    @Override
    public void stop() {
    }

    // duck spin s curve ramp taken from sin curve
    // put in speedMin and speedMax, -1 to 1
    // time is the total time of one routine
    // boolean full: true means a half period of sin curve
    //               false means a quarter period of sin curve
    public void duckRampS(double speedMin, double speedMax, double time, boolean full) {
        if (time == 0) {
            return;
        }

        speedMin = Math.max(-1, speedMin);
        speedMin = Math.min(1, speedMin);
        speedMax = Math.max(-1, speedMax);
        speedMax = Math.min(1, speedMax);
        double y = 0;

        if (full) {
            double a = Math.PI / Math.abs(time);
            double b = Math.abs(speedMax - speedMin) / 2;
            double c = Math.abs(time) / 2;
            double d = Math.abs(speedMax + speedMin) / 2;
            double x = timer.seconds();
            if (timer.seconds() <= time) {
                //y = 3 * Math.pow(timer.seconds() / time, 2) - 2 * Math.pow(timer.seconds() / time, 3);
                y = b * Math.sin(a * (x - c)) + d;
            } else {
                y = 0;
            }
        } else {
            double a = Math.PI / (2 * Math.abs(time));
            double b = Math.abs(speedMax - speedMin);
            double x = timer.seconds();
            if (timer.seconds() <= time) {
                y = b * Math.sin(a * x) + speedMin;
            } else {
                y = 0;
            }
        }

        if (!done && started) {
            // speed is calculated using the curve defined above
            if (speedMax > 0) {
                duck.setPower(y);
                duck2.setPower(y);
            } else {
                duck.setPower(-y);
                duck2.setPower(-y);
            }
            done = (timer.seconds() > time);
        }

        if (!started) {
            timer.reset();
            started = true;
            done = false;
        } else if (done) {
            duck.setPower(0);
            duck2.setPower(0);
            started = false;
        }

        telemetry.log().add(getClass().getSimpleName() + "::duckRampS(): Motors in use");
        telemetry.addData("Vel1", duck.getPower());
        telemetry.addData("Vel2", duck2.getPower());
    }

    // duck spin inverse s curve ramp taken from arcsin curve
    // put in speedMin and speedMax, -1 to 1
    // time is the total time of one routine
    // boolean full: true means a half period of inverse sin curve
    //               false means a quarter period of inverse sin curve
    public void duckRampInvS(double speedMin, double speedMax, double time, boolean full) {
        if (time == 0) {
            return;
        }

        speedMin = Math.max(-1, speedMin);
        speedMin = Math.min(1, speedMin);
        speedMax = Math.max(-1, speedMax);
        speedMax = Math.min(1, speedMax);
        double y = 0;

        if (!full) {
            double a = 2 * Math.abs(speedMax - speedMin) / Math.PI;
            double b = 1 / time;
            double x = timer.seconds();
            if (timer.seconds() <= time) {
                y = a * Math.asin(b * x) + speedMin;
            } else {
                y = 0;
            }
        } else {
            double a = 2 / time;
            double b = Math.abs(speedMax - speedMin) / Math.PI;
            double c = Math.abs(time) / 2;
            double d = Math.abs(speedMax + speedMin) / 2;
            double x = timer.seconds();
            if (timer.seconds() <= time) {
                y = b * Math.asin(a * (x - c)) + d;
            } else {
                y = 0;
            }
        }

        if (!done && started) {
            // speed is calculated using the curve defined above
            if (speedMax > 0) {
                duck.setPower(y);
                duck2.setPower(y);
            } else {
                duck.setPower(-y);
                duck2.setPower(-y);
            }
            done = (timer.seconds() > time);
        }

        if (!started) {
            timer.reset();
            started = true;
            done = false;
        } else if (done) {
            duck.setPower(0);
            duck2.setPower(0);
            started = false;
        }

        telemetry.log().add(getClass().getSimpleName() + "::duckRampInvS(): Motors in use");
        telemetry.addData("Vel1", duck.getPower());
        telemetry.addData("Vel2", duck2.getPower());
    }

    // duck spin inverse s curve ramp taken from arcsin curve
    // put in speedMin and speedMax, -1 to 1
    // time is the total time of one routine
    // boolean full: true means a half period of inverse sin curve
    //               false means a quarter period of inverse sin curve
    public void duckRampPoly(double speedMin, double speedMax, double time, double pow) {
        if (time == 0) {
            return;
        }

        speedMin = Math.max(-1, speedMin);
        speedMin = Math.min(1, speedMin);
        speedMax = Math.max(-1, speedMax);
        speedMax = Math.min(1, speedMax);
        double y;

        double x = timer.seconds();
        if (timer.seconds() <= time) {
            y = Math.pow(time, -pow) * (speedMax - speedMin) * Math.pow(x, pow) + speedMin;
        } else {
            y = 0;
        }

        if (!done && started) {
            // speed is calculated using the curve defined above
            duck.setPower(y);
            duck2.setPower(y);
            done = (timer.seconds() > time);
        }

        if (!started) {
            timer.reset();
            started = true;
            done = false;
        } else if (done) {
            duck.setPower(0);
            duck2.setPower(0);
            started = false;
        }

        telemetry.log().add(getClass().getSimpleName() + "::duckRampInvS(): Motors in use");
        telemetry.addData("Vel1", duck.getPower());
        telemetry.addData("Vel2", duck2.getPower());
    }

    enum AUTO_STATE implements OrderedEnum {
        IDLE,
        RED,
        BLUE,
        SPIN_BLUE,
        SPIN_RED,
        NEW_SPIN1,
        NEW_SPIN2,
        NEW_SPIN3,
        NEW_SPIN4,
        NEW_SPIN5,
        DONE;

        public AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }
}