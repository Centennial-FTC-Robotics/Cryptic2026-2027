package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Configurable
@TeleOp(name = "TeleOp-Stupid")
public class TeleOpStupid extends OpMode {

    public static double flywheelPowerCoefficient = 0.02;
    public static double flywheelPowerBase = 0.77;
    public static double flywheelSwitchVoltage = 10;
    public static double targetVelocity = 1080;
    public static double toleranceFractionHigh = 0.02;

    public static double[] rightFootPositions = { 0.16, 0.0 };        // { Extended, Retracted }
    public static double[] leftFootPositions = { 0.16, 0.0 };         // { Extended, Retracted }
    public static double[] compressionPositions = { 1.0, 0.0, 0.5 };  // { Pollen, Nectar, Blocking }
    public static double[] camPositions = { 0.0, 0.4 };               // { Pollen, Nectar }

    public static double inPerTick = 0.00193071191602763;
    public static double forwardPodY = 0;
    public static double strafePodX = 0;

    public static double driverForwardDeg = 180;

    public static double resetX = 141.5;
    public static double resetY = 141.5;
    public static double resetHeadingDeg = 180;

    public static double centerX = 70.75;
    public static double centerY = 70.75;
    public static double aimOffsetDeg = 0;
    public static double aimKP = 1.0;
    public static double aimKI = 0.0;
    public static double aimKD = 0.08;
    public static double aimIntegralMax = 0.3;
    public static double aimMinPower = 0.5;
    public static double aimMaxPower = 1.0;
    public static double aimToleranceDeg = 0.5;

    private boolean nectarMode = false;
    private boolean autoAiming = false;
    private double aimIntegral, aimLastError, aimLastTime;
    private boolean aimFirstLoop = true;

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheelMotor, stage1IntakeMotor, stage2IntakeMotor;
    private Servo rightFootServo, leftFootServo, compressionServo, camServo;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private TelemetryManager panelsTelemetry;
    private VoltageSensor batterySensor;

    private double headingOffsetRad;
    private double rawX, rawY, rawHeading;
    private double worldX, worldY, worldHeading;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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

        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        batterySensor = hardwareMap.voltageSensor.iterator().next();

        stage1IntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stage1IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        double mmPerTick = inPerTick * 25.4;
        pinpoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        pinpoint.setOffsets(forwardPodY, strafePodX, DistanceUnit.INCH);   // (forward pod sideways, strafe pod forward)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,    // parallel
                GoBildaPinpointDriver.EncoderDirection.REVERSED);                        // perpendicular
        pinpoint.resetPosAndIMU();

        rawX = 0;
        rawY = 0;
        rawHeading = 0;
        setFieldPose(resetX, resetY, Math.toRadians(resetHeadingDeg));
        headingOffsetRad = Math.toRadians(resetHeadingDeg);
    }

    private void updatePinpoint() {
        pinpoint.update();
        if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            rawX = pinpoint.getPosX(DistanceUnit.INCH);
            rawY = pinpoint.getPosY(DistanceUnit.INCH);
            rawHeading = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);
        }
    }

    private void setFieldPose(double x, double y, double headingRad) {
        worldHeading = headingRad - rawHeading;
        worldX = x - (Math.cos(worldHeading) * rawX - Math.sin(worldHeading) * rawY);
        worldY = y - (Math.sin(worldHeading) * rawX + Math.cos(worldHeading) * rawY);
    }

    private double fieldX() { return worldX + Math.cos(worldHeading) * rawX - Math.sin(worldHeading) * rawY; }
    private double fieldY() { return worldY + Math.sin(worldHeading) * rawX + Math.cos(worldHeading) * rawY; }

    @Override
    public void loop() {
        updatePinpoint();

        if (gamepad1.options) {
            imu.resetYaw();
            headingOffsetRad = Math.toRadians(driverForwardDeg);
        }

        if (gamepad1.shareWasPressed()) {
            setFieldPose(resetX, resetY, Math.toRadians(resetHeadingDeg));
            imu.resetYaw();
            headingOffsetRad = Math.toRadians(resetHeadingDeg);
        }

        double poseX = fieldX();
        double poseY = fieldY();
        double poseHeading = worldHeading + rawHeading;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double fieldHeading = AngleUnit.normalizeRadians(botHeading + headingOffsetRad);
        double driveHeading = fieldHeading - Math.toRadians(driverForwardDeg);

        double inputAxial = -gamepad1.left_stick_y;
        double inputLateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        if (gamepad1.xWasPressed()) {
            autoAiming = true;
            aimIntegral = 0;
            aimFirstLoop = true;
        }
        if (Math.abs(gamepad1.right_stick_x) > 0.1) autoAiming = false;

        double aimError = 0;
        double targetHeading = Math.atan2(centerY - poseY, centerX - poseX) + Math.toRadians(aimOffsetDeg);
        if (autoAiming) {
            aimError = AngleUnit.normalizeRadians(targetHeading - fieldHeading);

            double now = System.nanoTime() / 1e9;
            double dt = aimFirstLoop ? 0 : Math.max(now - aimLastTime, 1e-3);

            if (Math.abs(aimError) <= Math.toRadians(aimToleranceDeg)) {
                autoAiming = false;
            } else {
                aimIntegral = Math.max(-aimIntegralMax, Math.min(aimIntegralMax, aimIntegral + aimError * dt));
                double aimDerivative = aimFirstLoop ? 0 : (aimError - aimLastError) / dt;

                double aimOutput = aimKP * aimError + aimKI * aimIntegral + aimKD * aimDerivative
                        + Math.signum(aimError) * aimMinPower;
                aimOutput = Math.max(-aimMaxPower, Math.min(aimMaxPower, aimOutput));

                yaw = -aimOutput;
            }

            aimLastError = aimError;
            aimLastTime = now;
            aimFirstLoop = false;
        }

        double lateral = inputLateral * Math.cos(-driveHeading) - inputAxial * Math.sin(-driveHeading);
        double axial = inputLateral * Math.sin(-driveHeading) + inputAxial * Math.cos(-driveHeading);

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

        if (gamepad1.right_trigger > 0.1) { stage1IntakeMotor.setPower(1); }
        else if (gamepad1.left_trigger > 0.1) { stage1IntakeMotor.setPower(-1); stage2IntakeMotor.setPower(-1); }
        else {
            stage1IntakeMotor.setPower(0);
            stage2IntakeMotor.setPower(0);
        }

        if (batterySensor.getVoltage() > flywheelSwitchVoltage) flywheelMotor.setPower(flywheelPowerBase);
        else flywheelMotor.setPower(flywheelPowerBase + batterySensor.getVoltage() * flywheelPowerCoefficient);

        double flywheelVelocity = flywheelMotor.getVelocity();
        boolean velocityInRange = flywheelVelocity >= targetVelocity
                && flywheelVelocity <= targetVelocity * (1 + toleranceFractionHigh);

        if (gamepad1.a) {

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            rightFootServo.setPosition(rightFootPositions[0]);
            leftFootServo.setPosition(leftFootPositions[0]);

            if (velocityInRange) {
                stage1IntakeMotor.setPower(1);
                stage2IntakeMotor.setPower(1);
            }
        } else {
            rightFootServo.setPosition(rightFootPositions[1]);
            leftFootServo.setPosition(leftFootPositions[1]);
        }

        if (gamepad2.xWasPressed()) nectarMode = !nectarMode;

        compressionServo.setPosition(nectarMode ? compressionPositions[1] : compressionPositions[0]);
        camServo.setPosition(nectarMode ? camPositions[1] : camPositions[0]);

        // Debug
        // telemetry.addData("Nectar Mode: ", nectarMode);
        telemetry.addData("Bot Heading (Degrees): ", Math.toDegrees(botHeading));
        // telemetry.addData("Right Foot Position: ", rightFootServo.getPosition());
        // telemetry.addData("Left Foot Position: ", leftFootServo.getPosition());
        telemetry.addData("Flywheel Power", flywheelMotor.getPower());
        telemetry.addData("Flywheel Current Velocity: ", flywheelVelocity);
        telemetry.addData("Battery Voltage", batterySensor.getVoltage());
        telemetry.addData("flywheelPosition", leftFront.getCurrentPosition());

        panelsTelemetry.debug("Pose X: ", poseX);
        panelsTelemetry.debug("Pose Y: ", poseY);
        panelsTelemetry.debug("Pose Heading (Degrees): ", Math.toDegrees(poseHeading));
        panelsTelemetry.debug("Field Heading (Degrees): ", Math.toDegrees(fieldHeading));
        panelsTelemetry.debug("Pinpoint Status: ", String.valueOf(pinpoint.getDeviceStatus()));
        // panelsTelemetry.debug("Auto Aiming: ", autoAiming);
        // panelsTelemetry.debug("AutoAim Target Degrees: ", Math.toDegrees(targetHeading));
        // panelsTelemetry.debug("Auto Aim Target Error: ", Math.toDegrees(aimError));
        panelsTelemetry.update(telemetry);

    }
}