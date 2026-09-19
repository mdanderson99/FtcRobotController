package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * AprilTagLimelight
 *
 * Detects FTC AprilTags using a Limelight 3A smart camera and streams
 * per-tag telemetry (ID, angle offsets, distance/pose) plus the robot's
 * field pose to the Driver Station.
 *
 * ---------------------------------------------------------------------
 * ONE-TIME SETUP (do this before running this OpMode)
 * ---------------------------------------------------------------------
 * 1. Wire the Limelight 3A to the Control Hub via USB-C. It shows up as
 *    a network/USB device, not as a webcam.
 * 2. In the Driver Station / Control Hub's Robot Configuration, add the
 *    Limelight 3A and name it exactly "limelight" (matches the
 *    hardwareMap.get(...) call below).
 * 3. Connect to the camera's web interface (http://limelight.local:5801
 *    when on the same network) and, on pipeline slot 0:
 *      - Set the pipeline type to "AprilTag"
 *      - Confirm the tag family / field map matches the current FTC
 *        season (the 3A ships with the current season's map preloaded)
 * 4. (Optional, for field-relative robot pose) Measure your camera's
 *    position/orientation relative to the robot's center, enter those
 *    offsets on the pipeline's Advanced tab, and enable "Full 3D".
 *    Skip this if you only need angle-to-tag (tx/ty), not field pose.
 * ---------------------------------------------------------------------
 */
@TeleOp(name = "AprilTag Limelight Telemetry", group = "Vision")
public class AprilTagLimelight extends LinearOpMode {

    // Pipeline slot configured as "AprilTag" in the Limelight web UI
    private static final int APRILTAG_PIPELINE = 0;

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Tighten the Driver Station telemetry refresh rate
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start(); // must be called before getLatestResult() returns data

        telemetry.addData(">", "Limelight ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Pipeline", "Index: %d  Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
            telemetry.addData("Camera", "Temp: %.1fC  FPS: %d",
                    status.getTemp(), (int) status.getFps());

            LLResult result = limelight.getLatestResult();

            if (result == null) {
                telemetry.addLine("No result yet - check USB connection / device name.");
                telemetry.update();
                continue;
            }

            if (!result.isValid()) {
                telemetry.addLine("No AprilTags currently visible.");
                telemetry.update();
                continue;
            }

            telemetry.addData("Latency (ms)", "capture: %.1f  targeting: %.1f",
                    result.getCaptureLatency(), result.getTargetingLatency());

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            telemetry.addData("Tags Detected", tags.size());

            for (LLResultTypes.FiducialResult tag : tags) {
                telemetry.addLine(" ");
                telemetry.addData("Tag ID", "%d  (family: %s)",
                        tag.getFiducialId(), tag.getFamily());
                telemetry.addData("  tx / ty (deg)", "%.2f / %.2f",
                        tag.getTargetXDegrees(), tag.getTargetYDegrees());

                // Robot's pose relative to this one tag (useful for auto-aim / distance)
                Pose3D robotPoseWrtTag = tag.getRobotPoseTargetSpace();
                if (robotPoseWrtTag != null) {
                    telemetry.addData("  Robot pose rel. to tag", robotPoseWrtTag.toString());
                }
            }

            // Field-relative robot pose fused from all visible tags (MegaTag1).
            // Requires "Full 3D" + camera offsets to be configured in step 4 above.
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                telemetry.addLine(" ");
                telemetry.addData("Robot Field Pose", botpose.toString());
            }

            telemetry.update();
        }

        limelight.stop();
    }
}