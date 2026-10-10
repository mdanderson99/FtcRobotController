package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Test Teleop", group = "TeleOp")
public class motorTestTeleop extends OpMode {

    private DcMotorEx motor = null;

    @Override
    public void init() {
        // Initialize the motor
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // gamepad1.a: spin counterclockwise
        if (gamepad1.a) {
            motor.setPower(-0.4);
        }
        // gamepad1.b: spin clockwise
        else if (gamepad1.b) {
            motor.setPower(0.4);
        }
        // gamepad1.x: stop
        else if (gamepad1.x) {
            motor.setPower(0.0);
        }

        telemetry.addData("Motor Power", "%.2f", motor.getPower());
        telemetry.addData("Motor Velocity", "%.2f", motor.getVelocity());
        telemetry.update();
    }
}
