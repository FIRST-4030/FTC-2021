/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.vuforia;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.config.BOT;
import org.firstinspires.ftc.teamcode.robot.config.VuforiaFTCConfig;
import org.firstinspires.ftc.teamcode.utils.Heading;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class VuforiaFTC {
    private static final boolean DEBUG = true;

    // Short names for external constants
    private static final AxesReference AXES_REFERENCE = AxesReference.EXTRINSIC;
    private static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    // Cartesian heading constants
    private static final int HEADING_OFFSET = -Heading.FULL_CIRCLE / 4;

    // Dynamic things we need to remember
    private boolean running = false;
    private boolean capture = false;
    private final Telemetry telemetry;
    private final HardwareMap map;
    private VuforiaFTCConfig config;
    public VuforiaLocalizer vuforia = null;
    private int trackingTimeout = 100;
    private VuforiaTrackables targetsRaw;
    private final List<VuforiaTrackable> targets = new ArrayList<>();

    // The actual data we care about
    private long timestamp = 0;
    private final int[] location = new int[3];
    private final int[] orientation = new int[3];
    private final HashMap<String, Boolean> targetVisible = new HashMap<>();
    private final HashMap<String, Integer> targetAngle = new HashMap<>();
    private final HashMap<String, Integer> targetIndex = new HashMap<>();
    private ImageFTC image = null;

    public VuforiaFTC(HardwareMap map, Telemetry telemetry, BOT bot) {
        this.telemetry = telemetry;
        this.map = map;
    }

    public void init() {
        config = new VuforiaFTCConfig();
        targetsRaw = config.init(map);
        targets.addAll(targetsRaw);

        // Per-target hashmaps, by name
        for (VuforiaTrackable t : targets) {
            targetIndex.put(t.getName(), targets.indexOf(t));
            targetVisible.put(t.getName(), false);
            targetAngle.put(t.getName(), 0);
        }

        // Expose the vuforia object for external use
        vuforia = config.vuforia;
    }

    // True if we initialized Vuforia
    public boolean isAvailable() {
        return (vuforia != null);
    }

    // Start tracking
    public void start() {
        if (!isAvailable()) {
            return;
        }
        targetsRaw.activate();
        running = true;
    }

    // This doesn't completely disable Vuforia, but it stops most tracking tasks
    public void stop() {
        if (!isRunning()) {
            return;
        }
        targetsRaw.deactivate();
        running = false;
    }

    public boolean isRunning() {
        return isAvailable() && running;
    }

    public void track() {
        if (!isRunning()) {
            return;
        }

        for (VuforiaTrackable trackable : targets) {
            // Per-target visibility (somewhat imaginary but still useful)
            targetVisible.put(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible());

            // Angle to target, if available
            OpenGLMatrix newPose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
            if (newPose != null) {
                Orientation poseOrientation = Orientation.getOrientation(newPose, AXES_REFERENCE, AxesOrder.XYZ, ANGLE_UNIT);
                targetAngle.put(trackable.getName(), (int) poseOrientation.secondAngle);
            }

            /*
             * Update the location and orientation track
             *
             * We poll for each trackable so this happens in the loop, but the overall tracking
             * is aggregated among all targets with a defined pose and location. The current
             * field of view will dictate the quality of the track and if one or more targets
             * are present they will be the primary basis for tracking but tracking persists
             * even when the view does not include a target, and is self-consistent when the
             * view includes multiple targets
             */
            OpenGLMatrix newLocation = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (newLocation != null) {
                // Extract our location from the matrix
                for (int i = 0; i < location.length; i++) {
                    location[i] = (int) newLocation.get(i, 3);
                }

                // Calculate the orientation of our view
                Orientation newOrientation = Orientation.getOrientation(newLocation, AXES_REFERENCE, AxesOrder.XYZ, ANGLE_UNIT);
                orientation[0] = (int) newOrientation.firstAngle;
                orientation[1] = (int) newOrientation.secondAngle;
                orientation[2] = (int) newOrientation.thirdAngle;

                // Timestamp the update
                timestamp = System.currentTimeMillis();
            }
        }
    }

    public void display(Telemetry telemetry) {
        if (!isRunning()) {
            return;
        }

        // Is the location track valid?
        telemetry.addData("Valid", isStale() ? "No" : "Yes");

        // List of visible targets (if any)
        StringBuilder visibleStr = new StringBuilder();
        for (String target : targetVisible.keySet()) {
            if (getVisible(target)) {
                if (visibleStr.length() > 0) {
                    visibleStr.append(", ");
                }
                visibleStr.append(target);
            }
        }
        if (visibleStr.length() == 0) {
            visibleStr = new StringBuilder("<None>");
        }
        telemetry.addData("Visible", visibleStr.toString());

        // Angle to each visible target (if any)
        for (String target : targetVisible.keySet()) {
            if (getVisible(target)) {
                telemetry.addData(target + " ∠", getTargetAngle(target) + "°");
            }
        }

        // Raw data from the last location and orientation fix
        telemetry.addData("X/Y Heading", getX() + "/" + getY() + " " + getHeading() + "°");
    }

    /**
     * @return True if frame capture is enabled
     */
    public boolean capturing() {
        return isRunning() && capture;
    }

    /**
     * Enable frame capture -- not done by default because it consumes resources
     */
    public void enableCapture() {
        if (!isRunning()) {
            return;
        }
        vuforia.enableConvertFrameToBitmap();
        capture = true;
    }

    /*
     * Grab the next available frame, if capture is enabled
     */
    public void capture() {
        this.capture(null);
    }

    /*
     * Grab the next available frame, if capture is enabled, optionally saving to disk
     */
    public void capture(final String filename) {
        if (!capturing()) {
            return;
        }

        // Clear the buffer first so upper layers can monitor image != null to ensure freshness
        clearImage();

        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap == null) {
                    telemetry.log().add(this.getClass().getSimpleName() + ": No frame captured");
                    return;
                }
                image = new ImageFTC(bitmap);
                if (filename != null) {
                    if (!image.savePNG(filename)) {
                        telemetry.log().add(this.getClass().getSimpleName() + ": Unable to save file: " + filename);
                    }
                }
            }
        }));
    }

    /**
     * @return The most recent available frame, if any
     */
    public ImageFTC getImage() {
        return image;
    }

    /**
     * Clear the captured frame buffer
     */
    public void clearImage() {
        image = null;
    }

    /**
     * Getters
     */
    public HashMap<String, Boolean> getVisible() {
        return targetVisible;
    }

    /**
     * @param target Name of the target of interest.
     * @return True if the target was actively tracked in the last round of VuforiaFTC processing
     */
    public boolean getVisible(String target) {
        return targetVisible.get(target);
    }

    public HashMap<String, Integer> getTargetAngle() {
        return targetAngle;
    }

    /**
     * @param target Name of the target of interest. Valid targets will also be visible per
     *               {@link #getVisible(String)} getVisible(target)}
     * @return The angle to the target's plane relative to the plane of the phone's image sensor
     * (i.e. 0° is dead-on, negative sign denotes right-of-center)
     */
    public int getTargetAngle(String target) {
        return targetAngle.get(target);
    }

    /**
     * @param target Name of the target of interest.
     * @return The Vuforia targetable index for the named target.
     */
    public int getTargetIndex(String target) {
        return targetIndex.get(target);
    }

    /**
     * @param index Targets index.
     * @return Live VuforiaTrackable for the indexed target.
     */
    public VuforiaTrackable getTrackable(int index) {
        return targets.get(index);
    }

    /**
     * @param name Targets name.
     * @return Live VuforiaTrackable for the named target.
     */
    public VuforiaTrackable getTrackable(String name) {
        return targets.get(getTargetIndex(name));
    }

    /**
     * @return System.currentTimeMillis() as reported at the time of the last location update
     */
    public long getTimestamp() {
        return timestamp;
    }

    /**
     * @return True when the last location update was more than trackingTimeout milliseconds ago
     */
    public boolean isStale() {
        return (timestamp + trackingTimeout < System.currentTimeMillis());
    }

    public int[] getLocation() {
        return location;
    }

    public int[] getOrientation() {
        return orientation;
    }

    /**
     * @return The X component of the robot's last known location relative to the field center.
     * Negative values denote blue alliance side of field.
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int getX() {
        return location[0];
    }

    /**
     * @return The Y component of the robot's last known location relative to the field center.
     * Negative sign denotes audience side of field.
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int getY() {
        return location[1];
    }

    /**
     * @return The robot's last known heading relative to the field.
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int getHeading() {
        int heading = orientation[2];
        if (orientation[0] < 0) {
            heading -= Heading.FULL_CIRCLE / 2;
        }
        return Heading.normalize(cartesianToCardinal(heading));
    }

    /**
     * @param x X component of destination in the field plane
     * @param y Y component of destination in the field plane
     * @return Bearing from the current location to {x,y} with respect to field north
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int bearing(float x, float y) {
        return bearing(new int[]{getX(), getY()}, new int[]{(int) x, (int) y});
    }

    /**
     * @param dest X,Y array of destination in the field plane
     * @return Bearing from the current location to {x,y} with respect to field north
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int bearing(int[] dest) {
        return bearing(dest[0], dest[1]);
    }

    /**
     * @param index Target index. Syntax helper for {@link #bearing(float, float)} bearing(int, int)}
     * @return Bearing from the current location to {x,y} with respect to field north
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int bearing(int index) {
        float[] target = targetsRaw.get(index).getFtcFieldFromTarget().getData();
        return bearing(target[0], target[1]);
    }

    /**
     * @param x X component of destination in the field plane
     * @param y Y component of destination in the field plane
     * @return Distance from the current location to {x,y} with respect to field units (millimeters)
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int distance(float x, float y) {
        return distance(new int[]{getX(), getY()}, new int[]{(int) x, (int) y});
    }

    /**
     * @param dest X,Y array of destination in the field plane
     * @return Distance from the current location to {x,y} with respect to field units (millimeters)
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int distance(int[] dest) {
        return distance(dest[0], dest[1]);
    }

    /**
     * @param index Target index. Syntax helper for {@link #distance(float, float)} distance(int, int)}
     * @return Distance from the current location to {x,y} with respect to field units (millimeters)
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int distance(int index) {
        float[] target = targetsRaw.get(index).getFtcFieldFromTarget().getData();
        return distance(target[0], target[1]);
    }

    public void setTrackingTimeout(int timeout) {
        trackingTimeout = timeout;
    }

    public int getTrackingTimeout() {
        return trackingTimeout;
    }

    /**
     * Helpers
     */

    // Bearing from x1,y1 to x2,y2 in degrees
    // Motion from south to north is correlated with increasing Y components in field locations
    private int bearing(int[] src, int[] dest) {
        double bearing = Math.atan2(dest[1] - src[1], dest[0] - src[0]);
        bearing = Math.toDegrees(bearing);
        return Heading.normalize(cartesianToCardinal((int) bearing));
    }

    // Distance from x1,y1 to x2,y2 in field location units (millimeters)
    private int distance(int[] src, int[] dest) {
        return (int) Math.hypot((dest[1] - src[1]), (dest[0] - src[0]));
    }

    private int cartesianToCardinal(int heading) {
        return Heading.FULL_CIRCLE - (heading + HEADING_OFFSET);
    }
}
