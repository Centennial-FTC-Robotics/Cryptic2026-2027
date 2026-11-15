package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "TeleOp-Smart")
public class TeleOpSmart extends OpMode {

    private enum DriveState { NORMAL, SLOW;}
    private enum FlywheelState { STATIC, RUNNING, DRIVING;}
        private DriveState driveState = DriveState.NORMAL;
        private FlywheelState flywheelState = FlywheelState.DRIVING;

    private enum Stage1IntakeState { STATIC, INTAKE, REVERSE, FEED;}
    private enum Stage2IntakeState { STATIC, TRANSFER, REVERSE;}
        private Stage1IntakeState stage1IntakeState = Stage1IntakeState.STATIC;
        private Stage2IntakeState stage2IntakeState = Stage2IntakeState.STATIC;

    private enum FeetState { RETRACTED, EXTENDED;}
    private enum WedgeState { RETRACTED, EXTENDED;}
        private FeetState feetState = FeetState.RETRACTED;
        private WedgeState wedgeState = WedgeState.RETRACTED;

    private enum CompressionState { POLLEN, NECTAR, BLOCKING;}
    private enum CamState { POLLEN, NECTAR;}
        private CompressionState compressionState = CompressionState.POLLEN;
        private CamState camState = CamState.POLLEN;

    public static double[] driveMultipliers = { 1.0, 0.4 };             // { Normal, Slow }

    public static double[] flywheelPower = { 0.0, 0.4, 0.0 };           // { Static, Running, Driving }
    public static double[] stage1IntakePower = { 0.0, 1.0, -1.0, 1 }; // { Static, Intake, Reverse, Feed }
    public static double[] stage2IntakePower = { 0.0, 1, -1 };     // { Static, Transfer, Reverse }

    public static double[] rightFootPositions = { 0.14, 0.0 };          // { Extended, Retracted }
    public static double[] leftFootPositions = { 0.14, 0.0 };           // { Extended, Retracted }
    public static double[] wedgePositions = { 1.0, 0.5, 0.75 };         // { Extended, Retracted, Waypoint }
    public static double[] compressionPositions = { 1.0, 0.0, 0.5 };    // { Pollen, Nectar, Blocking }
    public static double[] camPositions = { 0.0, 0.4 };                 // { Pollen, Nectar }

    public static double wedgeWaypointDelaySec = 0.3;
    public static double toleranceFraction = 0.05;

    private final double motorMaxRpm = 6000;
    private final double ticksPerRevolution = 28;

    private double wedgeToggleTime = -1000;

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheelMotor, stage1IntakeMotor, stage2IntakeMotor;
    private Servo rightFootServo, leftFootServo, wedgeServo, compressionServo, camServo;

    private double getFlywheelTargetVelocity() { return flywheelPower[1] * motorMaxRpm * ticksPerRevolution / 60.0; }
    private boolean flywheelAtSpeed() { return flywheelMotor.getVelocity() >= getFlywheelTargetVelocity() * (1.0 - toleranceFraction); }

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
        wedgeServo = hardwareMap.get(Servo.class, "wedgeServo");
        compressionServo = hardwareMap.get(Servo.class, "compressionServo");
        camServo = hardwareMap.get(Servo.class, "camServo");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        stage1IntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) driveState = (driveState == DriveState.SLOW) ? DriveState.NORMAL : DriveState.SLOW;

        if (gamepad1.dpadDownWasPressed()) {
            wedgeState = (wedgeState == WedgeState.EXTENDED) ? WedgeState.RETRACTED : WedgeState.EXTENDED;
            wedgeToggleTime = getRuntime();
        }

        if (gamepad2.xWasPressed()) {
            boolean nectar = camState == CamState.POLLEN;
            camState = nectar ? CamState.NECTAR : CamState.POLLEN;
            compressionState = nectar ? CompressionState.NECTAR : CompressionState.POLLEN;
        }

        stage1IntakeState = Stage1IntakeState.STATIC;
        stage2IntakeState = Stage2IntakeState.STATIC;

        if (gamepad1.right_trigger > 0.1 || wedgeState == WedgeState.EXTENDED) stage1IntakeState = Stage1IntakeState.INTAKE;
        else if (gamepad1.left_trigger > 0.1) { stage1IntakeState = Stage1IntakeState.REVERSE; stage2IntakeState = Stage2IntakeState.REVERSE; }

        if (gamepad1.a) {
            flywheelState = FlywheelState.RUNNING;
            feetState = FeetState.EXTENDED;

            if (flywheelAtSpeed()) {
                stage1IntakeState = Stage1IntakeState.FEED;
                stage2IntakeState = Stage2IntakeState.TRANSFER;
            }
        } else { flywheelState = FlywheelState.DRIVING; feetState = FeetState.RETRACTED; }

        updateStateMachine();
        updateTelemetry();
    }

    private void updateStateMachine() {
        double driveScale = driveMultipliers[driveState == DriveState.SLOW ? 1 : 0];

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

        leftFront.setPower(leftFrontPower * driveScale);
        rightFront.setPower(rightFrontPower * driveScale);
        leftBack.setPower(leftBackPower * driveScale);
        rightBack.setPower(rightBackPower * driveScale);

        switch (flywheelState) {
            case STATIC:    flywheelMotor.setVelocity(flywheelPower[0] * motorMaxRpm * ticksPerRevolution / 60.0); break;
            case RUNNING:   flywheelMotor.setVelocity(getFlywheelTargetVelocity()); break;
            case DRIVING:   flywheelMotor.setVelocity(flywheelPower[2] * motorMaxRpm * ticksPerRevolution / 60.0); break;
        }

        switch (stage1IntakeState) {
            case STATIC:    stage1IntakeMotor.setPower(stage1IntakePower[0]); break;
            case INTAKE:    stage1IntakeMotor.setPower(stage1IntakePower[1]); break;
            case REVERSE:   stage1IntakeMotor.setPower(stage1IntakePower[2]); break;
            case FEED:      stage1IntakeMotor.setPower(stage1IntakePower[3]); break;
        }

        switch (stage2IntakeState) {
            case STATIC:    stage2IntakeMotor.setPower(stage2IntakePower[0]); break;
            case TRANSFER:  stage2IntakeMotor.setPower(stage2IntakePower[1]); break;
            case REVERSE:   stage2IntakeMotor.setPower(stage2IntakePower[2]); break;
        }

        switch (feetState) {
            case EXTENDED:
                rightFootServo.setPosition(rightFootPositions[0]);
                leftFootServo.setPosition(leftFootPositions[0]);
                break;
            case RETRACTED:
                rightFootServo.setPosition(rightFootPositions[1]);
                leftFootServo.setPosition(leftFootPositions[1]);
                break;
        }

        double wedgeTarget = (wedgeState == WedgeState.EXTENDED) ? wedgePositions[0] : wedgePositions[1];
        boolean atWaypoint = getRuntime() - wedgeToggleTime < wedgeWaypointDelaySec;
        wedgeServo.setPosition(atWaypoint ? wedgePositions[2] : wedgeTarget);

        switch (compressionState) {
            case POLLEN:    compressionServo.setPosition(compressionPositions[0]); break;
            case NECTAR:    compressionServo.setPosition(compressionPositions[1]); break;
            case BLOCKING:  compressionServo.setPosition(compressionPositions[2]); break;
        }

        switch (camState) {
            case POLLEN:    camServo.setPosition(camPositions[0]); break;
            case NECTAR:    camServo.setPosition(camPositions[1]); break;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("driveState", driveState);
        telemetry.addData("nectarMode", camState == CamState.NECTAR);
        telemetry.addData("wedgeState", wedgeState);
        telemetry.addData("rightFootCmd", rightFootServo.getPosition());
        telemetry.addData("leftFootCmd", leftFootServo.getPosition());
        telemetry.addData("wedgeCmd", wedgeServo.getPosition());
        telemetry.addData("compressionCmd", compressionServo.getPosition());
        telemetry.addData("camCmd", camServo.getPosition());
        telemetry.addData("flywheelTarget", getFlywheelTargetVelocity());
        telemetry.addData("flywheelVelocity", flywheelMotor.getVelocity());
        telemetry.addData("flywheelPosition", flywheelMotor.getCurrentPosition());
        telemetry.update();
    }
}