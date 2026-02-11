package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

/*
 * Copyright (c) 2025 Base 10 Assets, LLC
 * All rights reserved.
 *
 * FTC Autonomous OpMode for the 2025-2026 DECODE season.
 * This robot launches 3 balls and drives off the launch line.
 */
@Disabled
@Autonomous(name="my12_13_2025Auto", group="StarterBot")
public class my12_13_2025Auto extends OpMode {

    // Constants for launcher and drive behavior
    final double FEED_TIME = 0.20; // Time to run feeder servos per shot
    final double LAUNCHER_TARGET_VELOCITY = 1200; // Launcher motor velocity target
    final double LAUNCHER_MIN_VELOCITY = 1150; // Minimum velocity to fire a shot
    final double TIME_BETWEEN_SHOTS = 2; // Delay between shots
    final double DRIVE_SPEED = -0.5; // Maximum drive speed
    final double WHEEL_DIAMETER_MM = 102;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double DRIVE_DISTANCE = 29;
    double driveDistance = DRIVE_DISTANCE;
    final int SHOTS_TO_FIRE = 3;
    int shotsToFire = SHOTS_TO_FIRE;

    // Timers
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime selectionTimer = new ElapsedTime();

    // Hardware
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    private enum LaunchState {IDLE, PREPARE, LAUNCH}
    private LaunchState launchState;

    private enum AutonomousState {LAUNCH, WAIT_FOR_LAUNCH, DRIVING_AWAY_FROM_GOAL, COMPLETE}
    private AutonomousState autonomousState;

    private enum Alliance {RED, BLUE}
    private Alliance alliance = Alliance.RED;

    private enum AutoMode {DRIVE_SHOOT, DRIVE, SHOOT, DO_NOTHING};
    private AutoMode autoMode = AutoMode.DRIVE_SHOOT;

    private boolean driveInProgress = false;
    private final int TOLERANCE = 20; // Encoder tolerance

    @Override
    public void init() {
        autonomousState = AutonomousState.LAUNCH;
        launchState = LaunchState.IDLE;

        // Map hardware
        leftFront  = hardwareMap.get(DcMotor.class, "left_Front");
        rightFront = hardwareMap.get(DcMotor.class, "right_Front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_Back");
        rightBack  = hardwareMap.get(DcMotor.class, "right_Back");
        launcher   = hardwareMap.get(DcMotorEx.class,"launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder= hardwareMap.get(CRServo.class, "right_feeder");

        // Directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Zero power behavior
        leftFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // Launcher PIDF
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,0,0,10));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        // Alliance selection
        if (gamepad1.b) alliance = Alliance.BLUE;
        else if (gamepad1.x) alliance = Alliance.RED;

        if (gamepad1.y && selectionTimer.seconds() > 1) {
            int whichOne = autoMode.ordinal() + 1;
            selectionTimer.reset();
            if (whichOne >= autoMode.values().length) whichOne = 0;
            autoMode = AutoMode.values()[whichOne];
        }

        switch (autoMode) {
            case DRIVE_SHOOT: shotsToFire = SHOTS_TO_FIRE; driveDistance = DRIVE_DISTANCE; break;
            case DRIVE:       shotsToFire = 0; driveDistance = DRIVE_DISTANCE; break;
            case SHOOT:       shotsToFire = SHOTS_TO_FIRE; driveDistance = 0; break;
            case DO_NOTHING:  shotsToFire = 0; driveDistance = 0; break;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Press Y", "to toggle AutoMode");
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected AutoMode", autoMode);
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (autonomousState) {
            case LAUNCH:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;
            case WAIT_FOR_LAUNCH:
                if(launch(false)) {
                    shotsToFire--;
                    if(shotsToFire > 0) autonomousState = AutonomousState.LAUNCH;
                    else {
                        resetDriveEncoders();
                        launcher.setVelocity(0);
                        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                    }
                }
                break;
            case DRIVING_AWAY_FROM_GOAL:
                if(drive(DistanceUnit.INCH, driveDistance, DRIVE_SPEED, alliance == Alliance.RED)) {
                    resetDriveEncoders();
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f), rightBack (%.2f)",
                leftFront.getPower(), rightFront.getPower(), leftBack.getPower(), rightBack.getPower());
        telemetry.update();
    }
    private void resetDriveEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopAllDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    // Launch one ball with state machine control
    boolean launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if(shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if(launcher.getVelocity() > LAUNCHER_MIN_VELOCITY){
                    launchState = LaunchState.LAUNCH;
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if(feederTimer.seconds() > FEED_TIME){
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
                break;
        }
        return false;
    }

    // Drive with ramped acceleration and simple encoder check
    boolean drive(DistanceUnit unit, double distanceInInches, double speed, boolean isRed) {
        double targetMM = distanceInInches * 25.4; // Convert inches to mm
        int targetTicks = (int)(targetMM * TICKS_PER_MM);

        // Initialize drive only once
        if(!driveInProgress) {
            driveInProgress = true;
            driveTimer.reset();
            resetDriveEncoders();
        }

        // Ramp up acceleration over 0.4 seconds
        double ramp = Math.min(1.0, driveTimer.seconds()/0.4);
        double power = speed;// * ramp;

        // Set motor powers based on alliance
        if(isRed){
            leftFront.setPower(power);
            rightBack.setPower(power);
        }
        else {
            rightFront.setPower(power);
            leftBack.setPower(power);
        }

        // Check if target position reached using MATH (I know, math is so scary >:))
        boolean done;
        int pos1, pos2;
        if(isRed) {
            pos1 = leftFront.getCurrentPosition();
            pos2 = rightBack.getCurrentPosition();
        }
        else {
            pos1 = rightFront.getCurrentPosition();
            pos2 = leftBack.getCurrentPosition();
        }
        //done = Math.abs(targetTicks - pos1)<TOLERANCE && Math.abs(targetTicks - pos2)<TOLERANCE;
        done = Math.abs(targetTicks - pos1) < TOLERANCE && Math.abs(targetTicks - pos2) < TOLERANCE;

        if(!done)
            driveTimer.reset(); // Reset timer if not within tolerance

        // Telemetry for debugging
        telemetry.addData("TargetTicks", targetTicks);
        telemetry.addData("Ticks", "pos1 (%d), pos2 (%d)", pos1, pos2);
        telemetry.addData("Power", power);
        telemetry.addLine(isRed ? "Alliance RED" : "Alliance BLUE");

        // Stop motors once finished and ramped hold achieved
        //if(done && driveTimer.seconds()>20){
        if(done) {
            stopAllDrive();
            driveInProgress=false;
            return true;
        }
        /*if(!done && driveTimer.seconds()>5){
        throw new ArithmeticException("HEY HEY HEY HEY HEY 67!!!!");
        }*/
        return false;
    }

}
