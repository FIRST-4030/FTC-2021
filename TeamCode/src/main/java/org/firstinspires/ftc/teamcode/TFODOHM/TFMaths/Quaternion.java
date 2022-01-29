package org.firstinspires.ftc.teamcode.TFODOHM.TFMaths;

public class Quaternion {

    private float i, j, k, w;

    public Quaternion(float i, float j, float k, float w){

    }

    public float length(){
        return (float) Math.sqrt(i * i + j * j + k * k + w * w);
    }

    public void normalize(){
        float length = this.length();
        i /= length;
        j /= length;
        k /= length;
        w /= length;
    }

    public Quaternion normalized(){
        float length = this.length();
        float ni = this.i / length;
        float nj = this.j / length;
        float nk = this.k / length;
        float nw = this.w / length;

        return new Quaternion(ni, nj, nk, nw);
    }

    public Quaternion conjugate(){
        return new Quaternion(-i, -j, -k, w);
    }

    public Quaternion mul(Quaternion b){
        float ni =  i * b.getW() + j * b.getK() - k * b.getJ() + w * b.getI();
        float nj = -i * b.getK() + j * b.getW() + k * b.getI() + w * b.getJ();
        float nk = -i * b.getJ() + j * b.getI() + k * b.getW() + w * b.getK();
        float nw = -i * b.getI() - j * b.getJ() - k * b.getK() + w * b.getW();

        return new Quaternion(ni, nj, nk, nw);
    }

    public Quaternion mul(Vector3f b){

        float ni =                 j * b.getZ() - k * b.getY() + w * b.getX();
        float nj = -i * b.getZ()                + k * b.getX() + w * b.getY();
        float nk =  i * b.getY() - j * b.getX()                + w * b.getZ();
        float nw = -i * b.getX() - j * b.getY() - k * b.getZ();

        return new Quaternion(ni, nj, nk, nw);
    }

    public float getI() {
        return i;
    }

    public void setI(float i) {
        this.i = i;
    }

    public float getJ() {
        return j;
    }

    public void setJ(float j) {
        this.j = j;
    }

    public float getK() {
        return k;
    }

    public void setK(float k) {
        this.k = k;
    }

    public float getW() {
        return w;
    }

    public void setW(float w) {
        this.w = w;
    }

    public Matrix4f getAsFrameRotationMatrix(){
        float a, b, c, d;
        a = w; b = i; c = j; d = k;
        return null;
    }

    public Matrix4f getAsPointRotationMatrix(){
        return null;
    }
}
