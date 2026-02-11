//my1_10_2026Teleop.java

        package org.firstinspires.ftc.teamcode;
/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 * (standard license text unchanged)
 */

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "my1_10_2026Teleop", group = "StarterBot")
@Disabled
public class my1_10_2026Teleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1300;
    final double LAUNCHER_MIN_VELOCITY = 1270;
    //lower this bc ppl have beef with 1500 ðŸ¥€
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    ElapsedTime feederTimer = new ElapsedTime();

    // Data Logging Stuff - We'll make it store all the same stuff as telemetry
    //DataLog datalog;

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

        //leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    public void loop() {

        // â­ FRONT/BACK FLIPPED â­
        // (Only line changed)
        double x = -gamepad1.left_stick_y;   // â† forward/back reversed
        double y =  gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        double leftFrontPower  = x + y + rotation;
        double rightFrontPower = x - y - rotation;
        double leftBackPower   = x - y + rotation;
        double rightBackPower  = x + y - rotation;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) {
            launcher.setVelocity(STOP_SPEED);
        }

        launch(gamepad1.rightBumperWasPressed());

        telemetry.addData("State", launchState);
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f), rightBack (%.2f)",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() { }

    double softenedDrive(double x) {
        return 0.7 * Math.pow(x, 3) + 0.3 * x;
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
}