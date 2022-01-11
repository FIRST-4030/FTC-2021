package org.firstinspires.ftc.teamcode.TFODOMH.TFMaths;

public class TFMathExtension {

    /**
     * Liner Interpolation: 'a' & 'b' are the start & end points (order matters), and 't' is the scalar value: 0 gives 'a', 1 gives 'b', anything between gives a point in between 'a' & 'b'
     * @param a
     * @param b
     * @param t
     * @return
     */
    public static float lerp(float a, float b, float t){
        return a + (b - a) * t;
    }

    /**
     * Liner Interpolation: 'a' & 'b' are the start & end points (order matters), and 't' is the scalar value: 0 gives 'a', 1 gives 'b', anything between gives a point in between 'a' & 'b'
     * @param a
     * @param b
     * @param t
     * @return
     */
    public static Vector3f lerp(Vector3f a, Vector3f b, float t){
        float nX = a.getX() + (b.getX() - a.getX()) * t;
        float nY = a.getY() + (b.getY() - a.getY()) * t;
        float nZ = a.getZ() + (b.getZ() - a.getZ()) * t;

        return new Vector3f(nX, nY, nZ);
    }

    /**
     * quick clamping solution. Clamps a value between a min and max
     * @param min
     * @param max
     * @param value
     * @return
     */
    public static float clamp(float min, float max, float value){
        return Math.min(max, Math.max(min, value));
    }

    /**
     * Returns the intersection between two given 2d line segments (Line 1 is determined by l1 & l2, while Line 2 is determined by l3 & l4).
     * It just uses Vector3f for convenience.
     * @param l1
     * @param l2
     * @param l3
     * @param l4
     * @return new Vector3f(ix, iy, 1);
     */
    public static Vector3f llInt2d(Vector3f l1, Vector3f l2, Vector3f l3, Vector3f l4){
        float x1 = l1.getX();
        float x2 = l2.getX();

        float y1 = l1.getY();
        float y2 = l2.getY();

        float x3 = l3.getX();
        float x4 = l4.getX();

        float y3 = l3.getY();
        float y4 = l4.getY();

        float denominator = ((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4));

        if (denominator == 0) {return null;}

        float e1 = quick2fArrDet(new float[]{x1, y1, x2, y2});
        float e3 = quick2fArrDet(new float[]{x3, y3, x4, y4});
        float e4 = quick2fArrDet(new float[]{x3,  1, x4,  1});

        float e2x = quick2fArrDet(new float[]{x1, 1, x2, 1});
        float e2y = quick2fArrDet(new float[]{y1, 1, y2, 1});

        float ix = quick2fArrDet(new float[]{e1, e2x, e3, e4}) / denominator;
        float iy = quick2fArrDet(new float[]{e1, e2y, e3, e4}) / denominator;

        return new Vector3f(ix, iy, 1);
    }

    /**
     * returns the determinant of a 4 length array without casting it into a Matrix2f object
     * <br> [a,b] is arranged like {a, b, c, d} in a 1d array
     * <br> [c,d]
     *
     * @param a
     * @return a * c - b * d
     */
    public static float quick2fArrDet(float[] a){
        return a[0] * a[3] - a[1] * a[2];
    }

    /**
     * returns the determinant of a 9 length array without casting it into a Matrix3f object
     * <br> [a,b,c] is arranged like {a, b, c, d, e, f, g, h, i} in a 1d array
     * <br> [d,e,f]
     * <br> [g,h,i]
     *
     * @param a
     * @return a * |e,f,h,i| - b * |d,f,g,i| + c * |d,e,g,h|
     */
    public static float quick3fArrDet(float[] a){
        float[] min1 = {a[4], a[5], a[7], a[8]};
        float[] min2 = {a[3], a[5], a[6], a[8]};
        float[] min3 = {a[3], a[4], a[6], a[7]};
        return quick2fArrDet(min1) * a[0] - quick2fArrDet(min2) * a[1] + quick2fArrDet(min3) * a[2];
    }

    /**
     * bypass using Vector2f length method
     * @return float length
     */
    public static float quickLengthF(float x1, float y1, float x2, float y2){
        return (float) Math.sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) - (y2 - y1)));
    }

    /**
     * bypass using Vector2f length method
     * @return double length
     */
    public static double quickLengthD(float x1, float y1, float x2, float y2){
        return Math.sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) - (y2 - y1)));
    }

    /**
     * This method converts the sensor width, sensor height, and focal length of a camera into horizontal & vertical FOV
     * @param width
     * @param height
     * @param focal_len
     * @return
     */
    public static double[] findFOV(double width, double height, double focal_len){
        double hFov, vFov;
        hFov = 2 * Math.atan2(width, 2 * focal_len);
        vFov = 2 * Math.atan2(height, 2 * focal_len);

        return new double[] {hFov, vFov};
    }
}