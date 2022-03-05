package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class LoopUtil extends OpMode {

    private ElapsedTime loop_timer;
    private boolean timeStart = false, running = false, fixed_update_execute = false;
    private double delta_time, loop_start_time, loop_end_time, update_wait_time, fixed_update_wait_time, unprocessed_time,
            frameTime, frames, fps,
            MIN_WAIT_LIMIT, UPDATE_CAP;

    @Override
    public void init() {
        loop_timer = new ElapsedTime();

        delta_time = 0;
        loop_start_time = 0;
        loop_end_time = 0;

        update_wait_time = 0;
        fixed_update_wait_time = 0;

        frameTime = 0;
        frames = 0;
        fps = 0;

        MIN_WAIT_LIMIT = 0.1;
        UPDATE_CAP = 1.0 / 60.0;
    }

    @Override
    public void loop() {
        if (!timeStart){ //replicate the start of a game loop
            first_loop();
        }

        loop_start_time = loop_timer.milliseconds();
        delta_time = loop_end_time - loop_start_time;
        loop_end_time = loop_start_time;
        unprocessed_time += delta_time;
        frameTime += delta_time;

        while (unprocessed_time >= UPDATE_CAP){
            unprocessed_time -= UPDATE_CAP;

            if (frameTime >= 1.0){
                frameTime = 0;
                fps = frames;
                frames = 0;

                if (fixed_update_wait_time < MIN_WAIT_LIMIT) {
                    fixed_update();
                } else {
                    update_wait_time -= delta_time;
                }
            }
        }

        if (update_wait_time < MIN_WAIT_LIMIT) {
            update();
        } else {
            update_wait_time -= delta_time;
        }
    }

    private void first_loop(){
        loop_timer.reset();
        timeStart = true;
        running = true;
    }

    public abstract void update();
    public abstract void fixed_update();

    public void update_wait_for(double ms){
        update_wait_time = ms;
    }

    public void fixed_update_wait_for(double ms){
        fixed_update_wait_time = ms;
    }

    public double getDeltaTime(){
        return delta_time;
    }

    public double getCurrentTime(){
        return loop_timer.milliseconds();
    }
    public boolean isRunning(){
        return running;
    }
}
