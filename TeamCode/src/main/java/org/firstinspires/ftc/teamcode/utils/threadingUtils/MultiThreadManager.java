package org.firstinspires.ftc.teamcode.utils.threadingUtils;

import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class MultiThreadManager {

    private Runnable[] tasks;
    private Future[] tracker;
    private ExecutorService executor;

    public MultiThreadManager(Runnable... input_tasks) {
        this.tasks = input_tasks;
        this.executor = Executors.newScheduledThreadPool(this.tasks.length);
        this.tracker = new Future[this.tasks.length];
    }

    public void dispose(){
        for (Future future: this.tracker) {
            future.cancel(true);
        }
        executor.shutdown();
    }

    public void update(){
        for (int i = 0; i < tasks.length; i++) {
            if ((tracker[i] == null) || (tracker[i].isDone())) tracker[i] = executor.submit(tasks[i]);
        }
    }
}