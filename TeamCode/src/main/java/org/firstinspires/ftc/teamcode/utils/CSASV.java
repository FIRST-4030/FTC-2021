package org.firstinspires.ftc.teamcode.utils;

import java.util.concurrent.ConcurrentHashMap;

/**
 * Child Specific Abstract Static Variable
 * @param <T>
 */
public class CSASV<T>{

    private ConcurrentHashMap<Class, T> csashm;

    public CSASV(){
        csashm = new ConcurrentHashMap<>();
    }

    public void register(Class root, T variable){
        if (csashm.containsKey(root)) {return; }
        csashm.put(root, variable);
    }

    public void deregister(Class root){
        if (csashm.containsKey(root)){
            csashm.remove(root);
        } else {return;}
    }

    public void dispose(){
        csashm.clear();
    }

    public synchronized T get(Class root){
        if (csashm.containsKey(root)){
            return csashm.get(root);
        } else {
            return null;
        }
    }

    public synchronized void set(Class root, T variable){
        if (csashm.containsKey(root)){
            csashm.replace(root, variable);
        } else { return; }
    }


}
