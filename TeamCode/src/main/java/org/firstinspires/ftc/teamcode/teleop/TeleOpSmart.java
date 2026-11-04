package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "TeleOp-Smart")
public class TeleOpSmart extends OpMode {

    private ElapsedTime timeRunning = new ElapsedTime();

    private enum FlywheelState { STATIC, RUNNING, DRIVING }
    private enum Stage1IntakeState { STATIC, INTAKE, REVERSE }
    private enum Stage2IntakeState { STATIC, TRANSFER, REVERSE }
    private enum FeetState { RETRACTED, EXTENDED }
    private enum WedgeState { RETRACTED, EXTENDED }
    private enum CompressionState { POLLEN, NECTAR, BLOCKING }
    private enum CamState { POLLEN, NECTAR }

    private FlywheelState flywheelState = FlywheelState.DRIVING;
    private Stage1IntakeState stage1IntakeState = Stage1IntakeState.STATIC;
    private Stage2IntakeState stage2IntakeState = Stage2IntakeState.STATIC;
    private FeetState feetState = FeetState.RETRACTED;
    private WedgeState wedgeState = WedgeState.RETRACTED;
    private CompressionState compressionState = CompressionState.POLLEN;
    private CamState camState = CamState.POLLEN;

    public static float[] flywheelPower = { 0.0f, 0.7f, 0.5f };      // { Static, Running, Driving }

    public static double[] rightFootPositions = { 1.0, 0.0 };         // { Extended, Retracted }
    public static double[] leftFootPositions = { 1.0, 0.0 };          // { Extended, Retracted }
    public static double[] wedgePositions = { 1.0, 0.0 };             // { Extended, Retracted }
    public static double[] compressionPositions = { 1.0, 0.0, 0.5 };  // { Pollen, Nectar, Blocking }
    public static double[] camPositions = { 1.0, 0.0 };

    public static double toleranceFraction = 0.05;

    private final double motorMaxRpm = 6000;
    private final double ticksPerRevolution = 28;

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;

    private DcMotorEx flywheelMotor;
    private DcMotorEx stage1IntakeMotor;
    private DcMotorEx stage2IntakeMotor;

    private Servo rightFootServo;
    private Servo leftFootServo;
    private Servo wedgeServo;
    private Servo compressionServo;
    private Servo camServo;

    private double getFlywheelTargetVelocity() { return flywheelPower[1] * motorMaxRpm * ticksPerRevolution / 60.0; }

    private double getTolerance() { return getFlywheelTargetVelocity() * toleranceFraction; }

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

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double max;
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double leftFrontDOUBLE = axial + lateral + yaw;
        double rightFrontDOUBLE = axial - lateral - yaw;
        double leftBackDOUBLE = axial - lateral + yaw;
        double rightBackDOUBLE = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontDOUBLE), Math.abs(rightFrontDOUBLE));
        max = Math.max(max, Math.abs(leftBackDOUBLE));
        max = Math.max(max, Math.abs(rightBackDOUBLE));

        if (max > 1.0) {
            leftFrontDOUBLE /= max;
            rightFrontDOUBLE /= max;
            leftBackDOUBLE /= max;
            rightBackDOUBLE /= max;
        }

        leftFront.setPower(leftFrontDOUBLE);
        rightFront.setPower(rightFrontDOUBLE);
        leftBack.setPower(leftBackDOUBLE);
        rightBack.setPower(rightBackDOUBLE);

        if (gamepad1.bWasPressed()) {
            wedgeState = (wedgeState == WedgeState.EXTENDED) ? WedgeState.RETRACTED : WedgeState.EXTENDED;
        }

        if (gamepad1.right_trigger > 0.1 || wedgeState == WedgeState.EXTENDED) stage1IntakeState = Stage1IntakeState.INTAKE;
        else if (gamepad1.left_trigger > 0.1) stage1IntakeState = Stage1IntakeState.REVERSE;
        else stage1IntakeState = Stage1IntakeState.STATIC;

        if (gamepad1.a) {
            flywheelState = FlywheelState.RUNNING;
            feetState = FeetState.EXTENDED;

            if (Math.abs(flywheelMotor.getVelocity() - getFlywheelTargetVelocity()) <= getTolerance()) {
                stage1IntakeState = Stage1IntakeState.INTAKE;
                stage2IntakeState = Stage2IntakeState.TRANSFER;
            }
        }

        if (gamepad1.aWasReleased()) {
            flywheelState = FlywheelState.DRIVING;
            feetState = FeetState.RETRACTED;
            stage1IntakeState = Stage1IntakeState.STATIC;
            stage2IntakeState = Stage2IntakeState.STATIC;
        }

        updateStateMachine();
    }

    public void updateStateMachine() {
        switch (flywheelState) {
            case STATIC:    flywheelMotor.setPower(flywheelPower[0]); break;
            case RUNNING:   flywheelMotor.setVelocity(getFlywheelTargetVelocity()); break;
            case DRIVING:   flywheelMotor.setPower(flywheelPower[2]); break;
        }

        switch (stage1IntakeState) {
            case INTAKE:    stage1IntakeMotor.setPower(1);  break;
            case REVERSE:   stage1IntakeMotor.setPower(-1); break;
            case STATIC:    stage1IntakeMotor.setPower(0);  break;
        }

        switch (stage2IntakeState) {
            case TRANSFER:  stage2IntakeMotor.setPower(1);  break;
            case REVERSE:   stage2IntakeMotor.setPower(-1); break;
            case STATIC:    stage2IntakeMotor.setPower(0);  break;
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

        switch (wedgeState) {
            case EXTENDED:  wedgeServo.setPosition(wedgePositions[0]); break;
            case RETRACTED: wedgeServo.setPosition(wedgePositions[1]); break;
        }

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
}