package org.firstinspires.ftc.teamcode.TFODOHM.ODMain;

import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Matrix4f;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Plane3f;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.TFMathExtension;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Vector2f;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Vector3f;
import org.firstinspires.ftc.teamcode.TFODOHM.TFMaths.Vector4f;

public class CameraLens {
    //For the isBusy() method
    private boolean busy = false;

    //Camera Attributes
    private double hFOV = 1, vFOV = 1; //initialize with default values in just in case
    private final double zFar = 1000; //set the far plane/ will stretch the Z coordinate
    private final Plane3f xzPlane = Plane3f.XZ_PLANE; //define the plane for clipping the Camera Position to Calculated Vector 

    //NDC Img Attributes
    private Vector4f imgRight = new Vector4f(1, 0, 0, 1);
    private Vector4f imgBottom = new Vector4f(0, 1, 0, 1);
    private Vector4f imgCenter = new Vector4f(0, 0, 1, 1);

    //Describing transforms, most importantly, the rotation and translation
    private Matrix4f rotation = new Matrix4f();
    private Vector3f translation = new Vector3f();

    //Final output Matrix4f
    private Matrix4f imgToLocal = new Matrix4f();

    //Preset FOV value for the listed cameras (only the C270)
    public static final double[] C270_FOV = TFMathExtension.findFOV(3.58, 2.02, 4.11);

    public CameraLens(double[] fov){
        this.hFOV = fov[0];
        this.vFOV = fov[1];
    }

    public CameraLens(double[] fov, Vector3f lensPos, Matrix4f lensRot){
        this.hFOV = fov[0];
        this.vFOV = fov[1];

        this.translation = lensPos;
        this.rotation = lensRot;
    }

    public void setupCameraMatrix(){
        busy = true;

        //find half of the fov or aov
        double hHFOV = this.hFOV * 0.5;
        double hVFOV = this.vFOV * 0.5;

        //find the x and y scale for the camera fov
        double xScale = Math.tan(hHFOV) * zFar;
        double yScale = Math.tan(hVFOV) * zFar;

        //set the appropriate vectors with proper (X, Y, Z) scale
        this.imgRight.setX((float) xScale);
        this.imgBottom.setY((float) -yScale);
        this.imgCenter.setZ((float) this.zFar);

        //rotate those previous vectors
        this.imgRight = this.rotation.matMul(this.imgRight);
        this.imgBottom = this.rotation.matMul(this.imgBottom);
        this.imgCenter = this.rotation.matMul(this.imgCenter);

        //using a property of Matrices (columns bending the x, y, z... space)
        this.imgToLocal = new Matrix4f(
                new float[]{this.imgRight.getX(), this.imgBottom.getX(), this.imgCenter.getX(), this.translation.getX(),
                            this.imgRight.getY(), this.imgBottom.getY(), this.imgCenter.getY(), this.translation.getY(),
                            this.imgRight.getZ(), this.imgBottom.getZ(), this.imgCenter.getZ(), this.translation.getZ(),
                                               0,                     0,                     0,                       1}
        );

        busy = false;
    }

    public Vector3f findImgToLocal(Vector2f imgCoordinate){
        busy = true;
        Vector3f preClippedVector = imgToLocal.matMul(new Vector4f(imgCoordinate.getX(), imgCoordinate.getY(), 1, 1)).getAsVec3f();
        Vector3f output = this.xzPlane.getVector3fInt(this.translation, preClippedVector);
        busy = false;
        return output;
    }

    public Vector3f findImgToLocal(float imgX, float imgY){
        busy = true;
        Vector3f preClippedVector = imgToLocal.matMul(new Vector4f(imgX, imgY, 1, 1)).getAsVec3f();
        Vector3f output = this.xzPlane.getVector3fInt(this.translation, preClippedVector);
        busy = false;
        return output;
    }

    public void setTranslation(Vector3f newPosition){
        this.translation = newPosition;
        this.setupCameraMatrix();
    }

    public void setRotation(Matrix4f newRotation){
        this.rotation = newRotation;
        this.setupCameraMatrix();
    }

    public double getZFar(){
        return this.zFar;
    }

    public double getHFOV(){
        return this.hFOV;
    }

    public double getVFOV(){
        return this.vFOV;
    }

    public Vector3f getTranslation(){
        return this.translation;
    }

    public Vector4f getImgRight(){
        return this.imgRight;
    }

    public Vector4f getImgBottom(){
        return this.imgBottom;
    }

    public Vector4f getImgCenter(){
        return this.imgCenter;
    }

    public Plane3f getXZPlane(){
        return this.xzPlane;
    }

    public Matrix4f getRotation(){
        return this.rotation;
    }

    public Matrix4f getImgToLocal(){
        return this.imgToLocal;
    }

    @Override
    public String toString(){
        return  "CameraLens Attributes: " +
                "\nHFov: " + this.hFOV +
                "\nVFov: " + this.vFOV +
                "\nImgRight: " + this.imgRight +
                "\nImgBottom: " + this.imgBottom +
                "\nImgCenter: " + this.imgCenter +
                "\nTranslation: " + this.translation.toString() +
                "\nRotation: \n" + this.rotation.toString() +
                "\nImgToLocalMatrix: \n" + this.imgToLocal.toString();
    }

    public boolean isBusy() {
        return busy;
    }
}
