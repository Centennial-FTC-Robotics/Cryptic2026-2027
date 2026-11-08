package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "TeleOp-Stupid")
public class TeleOpStupid extends OpMode {

    public static double targetVelocity = 0;
    public static double toleranceFraction = 0.05;

    public static double[] rightFootPositions = { 0.14, 0.0 };        // { Extended, Retracted }
    public static double[] leftFootPositions = { 0.14, 0.0 };         // { Extended, Retracted }
    // public static double[] wedgePositions = { 1.0, 0.5 };          // { Extended, Retracted } - disabled for now
    public static double[] compressionPositions = { 1.0, 0.0, 0.5 };  // { Pollen, Nectar, Blocking }
    public static double[] camPositions = { 0.0, 0.4 };               // { Pollen, Nectar }

    private final double motorMaxRpm = 6000;
    private final double ticksPerRevolution = 28;

    private boolean nectarMode = false;

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheelMotor, stage1IntakeMotor, stage2IntakeMotor;
    private Servo rightFootServo, leftFootServo, compressionServo, camServo;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        stage1IntakeMotor = hardwareMap.get(DcMotorEx.class, "stage1Intake");
        stage2IntakeMotor = hardwareMap.get(DcMotorEx.class, "stage2Intake");

        rightFootServo = hardwareMap.get(Servo.class, "rightFoot");
        leftFootServo = hardwareMap.get(Servo.class, "leftFoot");
        compressionServo = hardwareMap.get(Servo.class, "compressionServo");
        camServo = hardwareMap.get(Servo.class, "camServo");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stage1IntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // Drivetrain
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // Stage 1 intake (triggers)
        if (gamepad1.right_trigger > 0.1) { stage1IntakeMotor.setPower(1); }
        else if (gamepad1.left_trigger > 0.1) { stage1IntakeMotor.setPower(-1); stage2IntakeMotor.setPower(-0.5); }
        else stage1IntakeMotor.setPower(0); stage2IntakeMotor.setPower(0);

        // A - shoot (flywheel at speed + feet down, intakes run once flywheel is up to speed)
        // double targetVelocity = 2800 * flywheelDrivingPower;
        // double velocityError = Math.abs(leftFront.getVelocity() - targetVelocity);

        /*
        if (gamepad1.a) {
            targetVelocity = 2800 * flywheelShootPower;

            flywheelMotor.setVelocity(targetVelocity);
            rightFootServo.setPosition(rightFootPositions[0]);
            leftFootServo.setPosition(leftFootPositions[0]);

            if (velocityError <= targetVelocity * toleranceFraction) {
                stage1IntakeMotor.setPower(1);
                stage2IntakeMotor.setPower(1);
            } else stage2IntakeMotor.setPower(0);
        }

        rightFootServo.setPosition(rightFootPositions[1]);
        leftFootServo.setPosition(leftFootPositions[1]);
        stage2IntakeMotor.setPower(0);
        flywheelMotor.setVelocity(targetVelocity);
        */

        if (gamepad1.a) {
            targetVelocity = 2800 * 0.4;

            rightFootServo.setPosition(rightFootPositions[0]);
            leftFootServo.setPosition(leftFootPositions[0]);

            if (flywheelMotor.getVelocity() / targetVelocity >= 0.95) {
                stage1IntakeMotor.setPower(0.5);
                stage2IntakeMotor.setPower(0.5);
            }
        } else {
            targetVelocity = 2800 * 0;

            rightFootServo.setPosition(rightFootPositions[1]);
            leftFootServo.setPosition(leftFootPositions[1]);
        }

        flywheelMotor.setVelocity(targetVelocity);

        if (gamepad2.xWasPressed()) nectarMode = !nectarMode;

        compressionServo.setPosition(nectarMode ? compressionPositions[1] : compressionPositions[0]);
        camServo.setPosition(nectarMode ? camPositions[1] : camPositions[0]);

        // Debug
        telemetry.addData("nectarMode", nectarMode);
        telemetry.addData("rightFootCmd", rightFootServo.getPosition());
        telemetry.addData("leftFootCmd", leftFootServo.getPosition());
        telemetry.addData("compressionCmd", compressionServo.getPosition());
        telemetry.addData("camCmd", camServo.getPosition());
        telemetry.addData("flywheelTarget", targetVelocity);
        telemetry.addData("flywheelVelocity", flywheelMotor.getVelocity());
        // telemetry.addData("velocityError", velocityError);
        telemetry.addData("flywheelPosition", leftFront.getCurrentPosition());
        telemetry.update();


    }
}