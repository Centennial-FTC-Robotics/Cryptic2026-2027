package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TeleOpStupid")
public class TeleOpStupid extends OpMode {

    private DcMotorEx flywheelMotor;

    private float flywheelPower = 0.1f,k;
    private final float powerStep = 0.05f;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // Kill switch: either y or a shuts everything down
        if (gamepad1.y || gamepad1.a) {
            flywheelMotor.setPower(0);
            requestOpModeStop();
            return;
        }

        if (gamepad1.bWasPressed()) flywheelPower += powerStep;
        if (gamepad1.xWasPressed()) flywheelPower -= powerStep;

        flywheelPower = Math.max(0.0f, Math.min(1.0f, flywheelPower));

        flywheelMotor.setPower(flywheelPower);

        telemetry.addData("Flywheel Power", "%.2f", flywheelPower);
        telemetry.addData("Flywheel Velocity (ticks per sec)", "%.0f", flywheelMotor.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheelMotor.setPower(0);
    }
}