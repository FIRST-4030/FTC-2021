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

import org.apache.commons.math3.analysis.function.Abs;
import org.firstinspires.ftc.teamcode.gamepad.GAMEPAD;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.gamepad.PAD_KEY;
import org.firstinspires.ftc.teamcode.utils.OrderedEnum;
import org.firstinspires.ftc.teamcode.utils.OrderedEnumHelper;


//@Disabled
@Config
@TeleOp(name = "Depositor", group = "Test")
public class Depositor extends OpMode {
    // Hardware
    private DcMotor belt = null;
    private Servo low = null;
    private Servo mid = null;
    private Servo tilt = null;
    private Servo high = null;
    private TouchSensor sensor = null;

    // Config
    public static boolean DEBUG = false;
    public static double BELT_SPEED = 0.6;
    public static double TILT_BACK = 0.47;
    public static double TILT_FORWARD = 0.18;
    public static double LOW_OPEN = 0.98;
    public static double LOW_CLOSE = 0.56;
    public static double MID_OPEN = 0.69;
    public static double MID_CLOSE = 0.24;
    public static double HIGH_OPEN = 0.13;
    public static double HIGH_INIT = 0.55;
    public static int INIT_PREP_POS = 390;
    public static int LOW_PREP_POS = 560;
    public static int MID_PREP_POS = 690;
    public static int HIGH_PREP_POS = 840;
    public static int BELT_POSITION_DEADBAND = 15;
    public int num = 0;
    public boolean sensorTriggered = false;
    public boolean prepPosSet = false;

    // Members
    private boolean enabled = false;
    private AUTO_STATE state = AUTO_STATE.DONE;
    private AUTO_STATE oldState = AUTO_STATE.DONE;
    private DOOR_USED required_Door = DOOR_USED.NONE;
    private InputHandler in;

    @Override
    public void init() {

        try {
            belt = hardwareMap.get(DcMotor.class, "Depbelt");
            belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            low = hardwareMap.get(Servo.class, "Deplow");
            low.setPosition(LOW_CLOSE);
            mid = hardwareMap.get(Servo.class, "Depmid");
            mid.setPosition(MID_CLOSE);
            high = hardwareMap.get(Servo.class, "Dephigh");
            high.setPosition(HIGH_INIT);
            tilt = hardwareMap.get(Servo.class, "Deptilt");
            sensor = hardwareMap.get(TouchSensor.class, "DS");

            Globals.opmode = this;
            in = Globals.input(this);
            sensorTriggered = false;
            prepPosSet = false;
            num = 0;

            in.register("LOW", GAMEPAD.driver2, PAD_KEY.x);
            in.register("MID", GAMEPAD.driver2, PAD_KEY.y);
            in.register("HIGH", GAMEPAD.driver2, PAD_KEY.b);
            in.register("REVERSE", GAMEPAD.driver2, PAD_KEY.a);
            in.register("TILT_FORWARD", GAMEPAD.driver2, PAD_KEY.dpad_left);
            in.register("TILT_BACK", GAMEPAD.driver2, PAD_KEY.dpad_right);

            enabled = true;
        } catch (Exception e) {
            telemetry.log().add(getClass().getSimpleName() + ": " +
                    "Could not initialize" + e.getMessage());
        }
    }

    @Override
    public void init_loop() {
        if (!enabled) {
            return;
        }

        if (sensor.isPressed()) {
            sensorTriggered = true;
        }
        if (sensorTriggered) {
            belt.setPower(0);
            belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            belt.setPower(0.3);
        }
    }

    @Override
    public void start() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }
        low.setPosition(LOW_CLOSE);
        mid.setPosition(MID_CLOSE);
        high.setPosition(HIGH_OPEN);
        tilt.setPosition(TILT_FORWARD);
    }

    @Override
    public void loop() {
        // Skip processing if we're disabled
        if (!enabled) {
            return;
        }

        // actually process the inputs from the game pads
        in.loop();

        telemetry.addData("isPressed?", sensor.isPressed() ? "Yes" : "No");
        telemetry.addData("beltBusy? ", belt.isBusy() ? "Yes" : "No");
        telemetry.addData("state", state);
        telemetry.addData("required door", required_Door);
        telemetry.addData("beltPos", belt.getCurrentPosition());
        telemetry.addData("beltPosCmd", belt.getTargetPosition());

        // this senses the magnetic prox switch, and reinitializes the belt encoder
        // and sets the flag that the switch has been hit
        // Note that the flag isn't reset until the state changes

        if (Math.abs(belt.getCurrentPosition() % 927 - belt.getTargetPosition()) <= BELT_POSITION_DEADBAND) {

        }

        // if the state changed, set sensorTriggered to false
        if (state != oldState) {
            oldState = state;
            sensorTriggered = false;
            prepPosSet = false;
        }
        switch (state) {
            // Tilt forward, move flipper to start position
            case TILTED_FORWARD:
                low.setPosition(LOW_CLOSE);
                mid.setPosition(MID_CLOSE);
                high.setPosition(HIGH_OPEN);
                tilt.setPosition(TILT_FORWARD);
                if (!prepPosSet) {
                    switch (required_Door) {
                        case LOW_DOOR:
                        case MID_DOOR:
                            belt.setTargetPosition(((belt.getCurrentPosition() / 927) * 927) + INIT_PREP_POS);
                            break;
                        case HIGH_DOOR:
                            break;
                    }
                    belt.setPower(BELT_SPEED);
                    prepPosSet = true;
                }
                if (belt.getCurrentPosition() >= belt.getTargetPosition()) {
                    belt.setPower(0);
                    state = AUTO_STATE.DONE;
                }
                required_Door = DOOR_USED.NONE;
                break;
            case DOOR_PREP:      // Move the flipper to below the required door
                // prepPosSet acts as a flag to make sure that the position command is set only once
                if (!prepPosSet) {
                    switch (required_Door) {
                        case LOW_DOOR:
                            belt.setTargetPosition((belt.getCurrentPosition() / 927 * 927) + LOW_PREP_POS);
                            break;
                        case MID_DOOR:
                            belt.setTargetPosition((belt.getCurrentPosition() / 927 * 927) + MID_PREP_POS);
                            break;
                        case HIGH_DOOR:
                            belt.setTargetPosition((belt.getCurrentPosition() / 927 * 927) + HIGH_PREP_POS);
                            break;
                    }
                    belt.setPower(BELT_SPEED);
                    prepPosSet = true;
                }
                if (belt.getCurrentPosition() >= belt.getTargetPosition()) {
                    state = AUTO_STATE.DONE;
                }
                break;
            case DOOR_OPEN:      // Open required door and run conveyor until the magnetic switch is active
                if (!prepPosSet) {
                    switch (required_Door) {
                        case LOW_DOOR:
                            low.setPosition((LOW_OPEN));
                            belt.setTargetPosition((((belt.getCurrentPosition() / 927) + 1) * 927));
                            break;
                        case MID_DOOR:
                            mid.setPosition(MID_OPEN);
                            belt.setTargetPosition((((belt.getCurrentPosition() / 927) + 1) * 927));
                            break;
                        case HIGH_DOOR:
                            high.setPosition(HIGH_OPEN);
                            belt.setTargetPosition((((belt.getCurrentPosition() / 927) + 1) * 927) + INIT_PREP_POS);
                            break;
                    }
                    belt.setPower(BELT_SPEED);
                    prepPosSet = true;
                }
                if (belt.getCurrentPosition() >= belt.getTargetPosition()) {
                    state = AUTO_STATE.TILTED_FORWARD;
                }
                break;
            case TILTED_BACK:
                tilt.setPosition(TILT_BACK);
                state = AUTO_STATE.DONE;
                break;
            case REVERSE_RUN:      // Run the belt in reverse for a set time (for TeleOp only)
                belt.setPower(-BELT_SPEED);
                break;
            case DONE:
                tilt.setPosition(TILT_FORWARD);
                low.setPosition(low.getPosition());
                mid.setPosition(mid.getPosition());
                high.setPosition(high.getPosition());
                belt.setPower(0);
                break;
        }


        /* if (in.down("REVERSE")) {
            state = AUTO_STATE.REVERSE_RUN;
        } else if (in.up("REVERSE")) {
            state = AUTO_STATE.DONE;
        } */
        if (state == AUTO_STATE.DONE) {
            if (gamepad2.x) {
                if (required_Door == DOOR_USED.LOW_DOOR) {
                    telemetry.addData("action: ", "going to low door open");
                    state = AUTO_STATE.DOOR_OPEN;
                } else {
                    setDoor(DOOR_USED.LOW_DOOR);
                    telemetry.addData("action: ", "going to low door prep");
                    state = AUTO_STATE.DOOR_PREP;
                }
            }
            if (gamepad2.y) {
                if (required_Door == DOOR_USED.MID_DOOR) {
                    telemetry.addData("action: ", "going to mid door open");
                    state = AUTO_STATE.DOOR_OPEN;
                } else {
                    setDoor(DOOR_USED.MID_DOOR);
                    telemetry.addData("action: ", "going to mid door prep");
                    state = AUTO_STATE.DOOR_PREP;
                }
            }
            if (gamepad2.b) {
                if (required_Door == DOOR_USED.HIGH_DOOR) {
                    telemetry.addData("action: ", "going to high door open");
                    state = AUTO_STATE.DOOR_OPEN;
                } else {
                    setDoor(DOOR_USED.HIGH_DOOR);
                    telemetry.addData("action: ", "going to high door prep");
                    state = AUTO_STATE.DOOR_PREP;
                }
            }
            if (gamepad2.dpad_left) {
                telemetry.addData("action: ", "going to tilt forward");
                state = AUTO_STATE.TILTED_FORWARD;
            }
            if (gamepad2.dpad_right) {
                telemetry.addData("action: ", "going to tilt backward");
                state = AUTO_STATE.TILTED_BACK;
            }
            if (gamepad2.a) {
                mid.setPosition(MID_OPEN);
            } else {
                mid.setPosition(MID_CLOSE);
            }
            if (gamepad2.right_trigger > 0) {
                high.setPosition(HIGH_OPEN + (gamepad2.right_trigger * 0.24));
            } else {
                high.setPosition(HIGH_OPEN);
            }
            if (gamepad2.left_trigger > 0) {
                low.setPosition(LOW_CLOSE + (gamepad2.left_trigger * (1 - LOW_CLOSE)));
            } else {
                low.setPosition(LOW_CLOSE);
            }
            belt.setPower(gamepad2.right_stick_y * 0.8);
        }

        // Debug when requested
        if (DEBUG) {
            telemetry.addData("Depositor Output",
                    "B %.2f/%d, L %.2f, M %.2f, T %.2f",
                    belt.getPower(), belt.getCurrentPosition(),
                    low.getPosition(), mid.getPosition(), tilt.getPosition());
            telemetry.addData("mode", state);
        }
    }

    enum AUTO_STATE implements OrderedEnum {
        TILTED_FORWARD,     // Tilt forward, move flipper to start position
        DOOR_PREP,          // Move the flipper to below the required door
        DOOR_OPEN,          // Open the required door and run conveyor a small amount
        TILTED_BACK,        // Tilt back
        REVERSE_RUN,        // Run the belt in reverse for a set time (for TeleOp only)
        DONE;               // Power down all servos

        public AUTO_STATE next() {
            return OrderedEnumHelper.next(this);
        }
    }

    enum DOOR_USED {
        LOW_DOOR,
        MID_DOOR,
        HIGH_DOOR,
        NONE
    }

    public void deposit() {
        state = AUTO_STATE.DOOR_OPEN;
        loop();
    }

    public void prep() {
        state = AUTO_STATE.DOOR_PREP;
        loop();
    }

    public boolean isDone() {
        return (state == AUTO_STATE.DONE || state == AUTO_STATE.TILTED_FORWARD);
    }

    @Override
    public void stop() {
        belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belt.setTargetPosition(belt.getCurrentPosition());
        state = AUTO_STATE.DONE;
    }

    public void setDoor(DOOR_USED newDoor) {
        required_Door = newDoor;
    }

    public DOOR_USED doorUsed() {
        return required_Door;
    }
}