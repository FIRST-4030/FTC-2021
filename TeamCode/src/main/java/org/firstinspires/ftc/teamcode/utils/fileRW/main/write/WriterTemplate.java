package org.firstinspires.ftc.teamcode.utils.fileRW.main.write;

public abstract class WriterTemplate {

    private int y_pos;

    protected void incrY(){y_pos += 1;}

    protected void decrY(){y_pos -= 1;}

    protected void setY(int nY){this.y_pos = nY;}

    protected int getY(){return this.y_pos;}
}
