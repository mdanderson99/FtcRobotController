//Variable Hood Mecanum Teleop

package org.firstinspires.ftc.teamcode;
/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 */


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "my1_31_2026CameraTeleop", group = "StarterBot")
//@Disabled
public class my1_31_2026CameraTeleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    double LAUNCHER_TARGET_VELOCITY = 1300;
    double LAUNCHER_MIN_VELOCITY = 1270;
    boolean clearToShoot = false;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo hood = null;

    ElapsedTime feederTimer = new ElapsedTime();
    private boolean cameraSettingsApplied = false;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        leftFront = hardwareMap.get(DcMotor.class, "left_Front");
        rightFront = hardwareMap.get(DcMotor.class, "right_Front");
        leftBack = hardwareMap.get(DcMotor.class, "left_Back");
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        hood = hardwareMap.get(Servo.class, "hood");

        // Motor directions
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        telemetry.addData("Status", "Initialized");

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        try {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            telemetry.addData("Error", "Camera could not be initialized");
        }
    }

    @Override
    public void init_loop() {
        checkAndSetCameraControls();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        checkAndSetCameraControls();

        boolean targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        if (aprilTag != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    }
                }
            }
        }

        // 1. Logic & Calculations
        if (targetFound) {
            double tagRotationDegrees = desiredTag.ftcPose.pitch; 
            double horizontalError = desiredTag.ftcPose.range * tan(toRadians(tagRotationDegrees));
            
            if (abs(horizontalError) < 2.61) {
                clearToShoot = true;
            } else {
                clearToShoot = false;
            }

            if (abs(horizontalError) >= 2.61 && (desiredTag.id == 20 || desiredTag.id == 24)) {
                clearToShoot = false;
            }
        } else {
            clearToShoot = false;
        }

        // 2. Drive Controls
        double x = -gamepad1.left_stick_y;
        double y =  gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        leftFront.setPower(x + y + rotation);
        rightFront.setPower(x - y - rotation);
        leftBack.setPower(x - y + rotation);
        rightBack.setPower(x + y - rotation);

        // 3. Launcher Controls
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) {
            launcher.setVelocity(STOP_SPEED);
        }

        // 4. Hood Controls
        if (gamepad1.left_bumper && gamepad1.left_trigger > 0) {
            hood.setPosition(0.525);
            LAUNCHER_TARGET_VELOCITY = 1250;
            LAUNCHER_MIN_VELOCITY = 1200;
        }   else if (gamepad1.left_bumper) {
            hood.setPosition(0.575);
            LAUNCHER_TARGET_VELOCITY = 1300;
            LAUNCHER_MIN_VELOCITY = 1255;
        }   else if (gamepad1.left_trigger > 0) {
            hood.setPosition(0.625);
            LAUNCHER_TARGET_VELOCITY = 1350;
            LAUNCHER_MIN_VELOCITY = 1300;
        }   else {
            hood.setPosition(0.525);
            LAUNCHER_TARGET_VELOCITY = 1250;
            LAUNCHER_MIN_VELOCITY = 1200;
        }

        launch(gamepad1.right_bumper);

        // 5. Telemetry
        telemetry.addData("Camera State", visionPortal != null ? visionPortal.getCameraState() : "NULL");
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Horiz Error", "%5.1f inches", desiredTag.ftcPose.range * tan(toRadians(desiredTag.ftcPose.pitch)));
        } else {
            telemetry.addData("Target", "Not Found");
        }

        telemetry.addData("Clear to shoot", clearToShoot);
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        telemetry.addData("Hood Position", hood.getPosition());
        
        telemetry.update();
    }

    @Override
    public void stop() {
        // Crucial for releasing the camera back to the system
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

    private void checkAndSetCameraControls() {
        if (visionPortal != null && !cameraSettingsApplied && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl != null) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                if (gainControl != null) {
                    gainControl.setGain(250);
                    cameraSettingsApplied = true;
                }
            }
        }
    }
}
