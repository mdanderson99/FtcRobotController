package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "shooterRigTestTeleop", group = "TeleOp")
public class shooterRigTestTeleop extends OpMode {

    private DcMotorEx shooterMotor;
    // Target velocity in ticks per second
    private final double TARGET_VELOCITY = 1300.0;

    @Override
    public void init() {
        // Initialize the shooter motor as a DcMotorEx to use PIDF control
        // Ensure the name "shooter" matches your configuration on the robot controller
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Set the motor mode to use encoders for velocity control
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients (P, I, D, F)
        // These values (300, 0, 0, 10) provide stable velocity control for launchers
        PIDFCoefficients pidf = new PIDFCoefficients(300.0, 0.0, 0.0, 10.0);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Set the motor to brake when power is zero
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // gamepad1.a: spin up the shooter to the target velocity
        if (gamepad1.a) {
            shooterMotor.setVelocity(TARGET_VELOCITY);
        }
        // gamepad1.b: stop the shooter
        else if (gamepad1.b) {
            shooterMotor.setVelocity(0.0);
        }

        telemetry.addData("Target Velocity", TARGET_VELOCITY);
        telemetry.addData("Current Velocity", shooterMotor.getVelocity());
        telemetry.update();
    }
}
