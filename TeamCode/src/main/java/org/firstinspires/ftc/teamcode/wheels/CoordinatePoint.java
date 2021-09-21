package org.firstinspires.ftc.teamcode.wheels;

public class CoordinatePoint {
    private float m_x;
    private float m_y;

    public CoordinatePoint (float xpos, float ypos){
        m_x = xpos;
        m_y = ypos;
    }

    public float x (){
        return m_x;
    }
    public float y (){
        return m_y;
    }
}
