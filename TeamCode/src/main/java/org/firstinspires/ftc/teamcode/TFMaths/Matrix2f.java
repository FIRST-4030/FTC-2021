package org.firstinspires.ftc.teamcode.TFODOMH.TFMaths;

import java.io.IOException;

public class Matrix2f {

    private float[] m;

    public Matrix2f(){
        m = new float[] {1.0f, 0.0f,
                         0.0f, 1.0f};
    }

    public Matrix2f(float[] matrix){
        if (matrix.length == 4) {
            m = matrix;
        } else {
            IOException e = new IOException("Array length is not 4!");
            e.printStackTrace();
        }
    }

    public float[] getAsFloatArray(){
        return m;
    }

    public Vector3f matMul(Vector3f a){
        float nX = a.getX() * m[0] + a.getY() * m[1];
        float nY = a.getX() * m[2] + a.getY() * m[3];
        return new Vector3f(nX, nY, a.getZ());
    }

    public static Matrix2f matMul(Matrix2f a, Matrix2f b){
        float[] am = a.getAsFloatArray();
        float[] bm = b.getAsFloatArray();
        float e1, e2, e3, e4;
        e1 = am[0] * bm[0] + am[1] * bm[2];
        e2 = am[0] * bm[1] + am[1] * bm[3];
        e3 = am[2] * bm[0] + am[3] * bm[2];
        e4 = am[2] * bm[1] + am[3] * bm[3];
        return new Matrix2f(new float[]{e1,e2,e3,e4});
    }

    public static Matrix2f add(Matrix2f a, Matrix2f b){
        float[] am = a.getAsFloatArray();
        float[] bm = b.getAsFloatArray();
        return new Matrix2f(new float[] {am[0] + bm[0], am[1] + bm[1], am[2] + bm[2], am[3] + bm[3]});
    }

    public static Matrix2f mul(Matrix2f a, float factor){
        float[] am = a.getAsFloatArray();
        return new Matrix2f(new float[] {am[0] * factor, am[1] * factor, am[2] * factor, am[3] * factor});
    }

    public float det(){
        return m[0] * m[3] - m[1] * m[2];
    }

    public Matrix2f getAsInverse(){
        float det = this.det();
        if (det == 0){ return null;}
        return new Matrix2f(new float[]{ m[3]/det, -m[1]/det, -m[2]/det, m[1]/det});
    }

    public void invert(){
        float det = this.det();
        if (det == 0){return;}
        float[] nm = {m[3]/det, -m[1]/det, -m[2]/det, m[0]/det};
        m = nm;
    }
}
