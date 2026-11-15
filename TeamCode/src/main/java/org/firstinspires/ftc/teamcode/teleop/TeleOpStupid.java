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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

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

    // Pinpoint setup copied from last year (Cryptic2025-2026 PinpointLocalizer and MecanumDrive)
    public static double inPerTick = 0.00193071191602763;
    public static double forwardPodY = 0;   // inches, sideways offset of the forward pod (left is positive)
    public static double strafePodX = 0;    // inches, forward offset of the strafe pod (forward is positive)

    // Field heading that stick forward drives toward (-90 = toward the red alliance, down on the map)
    public static double driverForwardDeg = -90;

    // Pose the odometry is set to when the reset button is pressed (Pedro field coordinates, inches / degrees)
    // Blue alliance: back against the top wall, top left corner, facing down toward the hive (-y)
    public static double resetX = 9;
    public static double resetY = 135;
    public static double resetHeadingDeg = -90;

    // Auto aim (X button)
    public static double centerX = 72;          // field center, Pedro 144 x 144 field
    public static double centerY = 72;
    public static double aimOffsetDeg = 0;      // angle from the bot's front to the shooter direction
    public static double aimKP = 1.0;           // power per radian of error
    public static double aimKI = 0.0;
    public static double aimKD = 0.08;
    public static double aimIntegralMax = 0.3;  // clamp on the integral term (radian seconds)
    public static double aimMinPower = 0.12;    // added in the direction of the error to overcome friction
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

    private double headingOffsetRad;                 // field heading that IMU yaw 0 corresponds to
    private double rawX, rawY, rawHeading;           // latest Pinpoint reading (Pinpoint frame)
    private double worldX, worldY, worldHeading;     // Pinpoint frame to field frame, like last year's txWorldPinpoint

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

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stage1IntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stage1IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // IMU for field centric driving - set these to match how the Control Hub is mounted
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // goBILDA Pinpoint - same setup as last year
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        double mmPerTick = inPerTick * 25.4;
        pinpoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        pinpoint.setOffsets(forwardPodY, strafePodX, DistanceUnit.INCH);   // (forward pod sideways, strafe pod forward)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,    // parallel
                GoBildaPinpointDriver.EncoderDirection.REVERSED);                        // perpendicular
        pinpoint.resetPosAndIMU();

        // The field pose is kept as a software offset on top of the Pinpoint, like last year's localizer
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

        // Options - bot is facing the driver forward direction (toward the red alliance): re-zero the IMU heading only
        if (gamepad1.options) {
            imu.resetYaw();
            headingOffsetRad = Math.toRadians(driverForwardDeg);
        }

        // Share - bot is at the reset pose: zero the odometry position and the heading together
        if (gamepad1.shareWasPressed()) {
            setFieldPose(resetX, resetY, Math.toRadians(resetHeadingDeg));
            imu.resetYaw();
            headingOffsetRad = Math.toRadians(resetHeadingDeg);
        }

        double poseX = fieldX();
        double poseY = fieldY();
        double poseHeading = worldHeading + rawHeading;   // Pinpoint heading, telemetry only

        // Drivetrain (field centric) - one heading for driving and aiming: IMU yaw in the field frame
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double fieldHeading = AngleUnit.normalizeRadians(botHeading + headingOffsetRad);
        double driveHeading = fieldHeading - Math.toRadians(driverForwardDeg);   // 0 when facing driver forward

        double inputAxial = -gamepad1.left_stick_y;
        double inputLateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // X - single press auto aim at the field center, cancelled by turning with the right stick
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

                yaw = -aimOutput;   // positive yaw turns clockwise, positive error needs counterclockwise
            }

            aimLastError = aimError;
            aimLastTime = now;
            aimFirstLoop = false;
        }

        double lateral = inputLateral * Math.cos(-driveHeading) - inputAxial * Math.sin(-driveHeading);
        double axial = inputLateral * Math.sin(-driveHeading) + inputAxial * Math.cos(-driveHeading);

        lateral *= 1.1;   // counteracts imperfect strafing

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
        else if (gamepad1.left_trigger > 0.1) { stage1IntakeMotor.setPower(-1); stage2IntakeMotor.setPower(-1); }
        else {
            stage1IntakeMotor.setPower(0);
            stage2IntakeMotor.setPower(0);
        }

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
            targetVelocity = 2800 * 0.45;

            rightFootServo.setPosition(rightFootPositions[0]);
            leftFootServo.setPosition(leftFootPositions[0]);

            if (flywheelMotor.getVelocity() / targetVelocity >= 0.975) {
                stage1IntakeMotor.setPower(1);
                stage2IntakeMotor.setPower(1);
            }
        } else {
            targetVelocity = 0;

            rightFootServo.setPosition(rightFootPositions[1]);
            leftFootServo.setPosition(leftFootPositions[1]);
        }

        flywheelMotor.setVelocity(targetVelocity);

        if (gamepad2.xWasPressed()) nectarMode = !nectarMode;

        compressionServo.setPosition(nectarMode ? compressionPositions[1] : compressionPositions[0]);
        camServo.setPosition(nectarMode ? camPositions[1] : camPositions[0]);

        // Debug
        telemetry.addData("nectarMode", nectarMode);
        telemetry.addData("botHeadingDeg", Math.toDegrees(botHeading));
        telemetry.addData("fieldHeadingDeg", Math.toDegrees(fieldHeading));
        telemetry.addData("pinpointStatus", pinpoint.getDeviceStatus());
        telemetry.addData("poseX", poseX);
        telemetry.addData("poseY", poseY);
        telemetry.addData("poseHeadingDeg", Math.toDegrees(poseHeading));
        telemetry.addData("autoAiming", autoAiming);
        telemetry.addData("aimTargetDeg", Math.toDegrees(targetHeading));
        telemetry.addData("aimErrorDeg", Math.toDegrees(aimError));
        telemetry.addData("rightFootCmd", rightFootServo.getPosition());
        telemetry.addData("leftFootCmd", leftFootServo.getPosition());
        telemetry.addData("compressionCmd", compressionServo.getPosition());
        telemetry.addData("camCmd", camServo.getPosition());
        telemetry.addData("flywheelTarget", targetVelocity);
        telemetry.addData("flywheelVelocity", flywheelMotor.getVelocity());
        // telemetry.addData("velocityError", velocityError);
        telemetry.addData("flywheelPosition", leftFront.getCurrentPosition());
        // Panels telemetry (also forwards to the Driver Station)
        panelsTelemetry.debug("poseX", poseX);
        panelsTelemetry.debug("poseY", poseY);
        panelsTelemetry.debug("poseHeadingDeg", Math.toDegrees(poseHeading));
        panelsTelemetry.debug("fieldHeadingDeg", Math.toDegrees(fieldHeading));
        panelsTelemetry.debug("pinpointStatus", String.valueOf(pinpoint.getDeviceStatus()));
        panelsTelemetry.debug("autoAiming", autoAiming);
        panelsTelemetry.debug("aimTargetDeg", Math.toDegrees(targetHeading));
        panelsTelemetry.debug("aimErrorDeg", Math.toDegrees(aimError));
        panelsTelemetry.update(telemetry);

    }
}