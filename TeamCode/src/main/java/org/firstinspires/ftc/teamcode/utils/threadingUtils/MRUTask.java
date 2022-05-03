package org.firstinspires.ftc.teamcode.utils.threadingUtils;

import java.util.HashMap;
import java.util.concurrent.ConcurrentHashMap;

public abstract class MRUTask implements Runnable{

    //mimic making separate static value for each child class
    private static ConcurrentHashMap<Class, Boolean>
            isDone = new ConcurrentHashMap<>(),
            isRunning = new ConcurrentHashMap<>();

    private Class staticAllocator = getClass();

    public void setupTask(){
        isDone.put(this.getClass(), true);
        isRunning.put(this.getClass(), false);
    }

    public abstract void init();
    public abstract void update();

    @Override
    public void run() {
        if(getIsRunning()) {
            setIsDone(false);
            update();
            setIsDone(true);
        }
    }


    //start of the child static stuff
    private void setIsDone(boolean doneState){
        isDone.replace(staticAllocator, doneState);
    }

    private boolean getIsDone(){
        return isDone.get(staticAllocator);
    }

    public static boolean isDone(Class childClass){
        return isDone.get(childClass);
    }

    private void setIsRunning(boolean runningState){
        isRunning.replace(staticAllocator, runningState);
    }

    private boolean getIsRunning(){
        return isRunning.get(staticAllocator);
    }

    public static boolean isRunning(Class childClass){
        return isRunning.get(childClass);
    }
    //end of child static variable shenanigans


    private void waitMs(long msWaitTime){
        long timeStart = System.currentTimeMillis();
        long delta = timeStart;
        while(delta < msWaitTime){
            delta = System.currentTimeMillis() - timeStart;
        }
    }

    private void waitNano(long nanoWaitTime){
        long timeStart = System.nanoTime();
        long delta = timeStart;
        while (delta < nanoWaitTime){
            delta = System.nanoTime() - timeStart;
        }
    }
}
