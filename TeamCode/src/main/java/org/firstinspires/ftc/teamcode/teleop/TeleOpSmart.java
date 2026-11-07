package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
@TeleOp(name = "TeleOp-Smart")
public class TeleOpSmart extends OpMode {

    // Top level mode. Only one can be active, and it decides what every subsystem is allowed to do.
    private enum RobotMode { DRIVE, SPIN_UP, FEED }

    private enum FlywheelState { STATIC, RUNNING, DRIVING }
    private enum Stage1IntakeState { STATIC, INTAKE, REVERSE }
    private enum Stage2IntakeState { STATIC, TRANSFER, REVERSE }
    private enum FeetState { RETRACTED, EXTENDED }
    private enum WedgeState { RETRACTED, EXTENDED }
    private enum CompressionState { POLLEN, NECTAR, BLOCKING }
    private enum CamState { POLLEN, NECTAR }

    private RobotMode robotMode = RobotMode.DRIVE;

    private FlywheelState flywheelState = FlywheelState.DRIVING;
    private Stage1IntakeState stage1IntakeState = Stage1IntakeState.STATIC;
    private Stage2IntakeState stage2IntakeState = Stage2IntakeState.STATIC;
    private FeetState feetState = FeetState.RETRACTED;
    private WedgeState wedgeState = WedgeState.RETRACTED;
    private CompressionState compressionState = CompressionState.POLLEN;
    private CamState camState = CamState.POLLEN;

    public static float[] flywheelPower = { 0.0f, 0.7f, 0.5f }; // { Static, Running, Driving }

    public static double[] rightFootPositions = { 1.0, 0.0 }; // { Extended, Retracted }
    public static double[] leftFootPositions = { 1.0, 0.0 }; // { Extended, Retracted }
    public static double[] wedgePositions = { 1.0, 0.0 }; // { Extended, Retracted }
    public static double[] compressionPositions = { 1.0, 0.0, 0.5 }; // { Pollen, Nectar, Blocking }
    public static double[] camPositions = { 1.0, 0.0 }; // { Pollen, Nectar }

    // Feeding starts inside startToleranceFraction and only stops outside stopToleranceFraction.
    public static double startToleranceFraction = 0.05;
    public static double stopToleranceFraction = 0.10;

    // Time after pressing a before feeding is allowed, so the wedge and feet finish moving first.
    public static double deployDelayMs = 300;

    // Feet are extended while shooting, so driving is blocked to avoid dragging them.
    public static boolean lockDriveWhileShooting = true;

    public static double triggerThreshold = 0.1;

    // Telemetry settings. Verbose adds motor currents, which cost extra hub reads.
    public static boolean enablePanelsTelemetry = true;
    public static boolean verboseTelemetry = true;
    public static double lowBatteryVolts = 11.5;
    public static double slowLoopMs = 50;

    private final double motorMaxRpm = 6000;
    private final double ticksPerRevolution = 28;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private VoltageSensor voltageSensor;
    private TelemetryManager panelsTelemetry;

    // Values captured once per loop so telemetry and logic see the same numbers.
    private double flywheelVelocity;
    private boolean shootInput;
    private boolean wedgeToggleInput;
    private boolean intakeInput;
    private boolean reverseInput;
    private double leftFrontCommand;
    private double rightFrontCommand;
    private double leftBackCommand;
    private double rightBackCommand;

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

    private double getFlywheelTargetVelocity() {
        return flywheelPower[1] * motorMaxRpm * ticksPerRevolution / 60.0;
    }

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

        // Bulk caching makes each loop read the hubs once instead of once per call.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        loopTimer.reset();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // Read every input once. bWasPressed must be consumed every loop even if it is ignored.
        boolean shootHeld = gamepad1.a;
        boolean wedgeTogglePressed = gamepad1.bWasPressed();
        boolean intakeRequested = gamepad1.right_trigger > triggerThreshold;
        boolean reverseRequested = gamepad1.left_trigger > triggerThreshold;

        flywheelVelocity = flywheelMotor.getVelocity();
        shootInput = shootHeld;
        wedgeToggleInput = wedgeTogglePressed;
        intakeInput = intakeRequested;
        reverseInput = reverseRequested;

        updateRobotMode(shootHeld);
        resolveStates(wedgeTogglePressed, intakeRequested, reverseRequested);
        updateDrive();
        applyOutputs();
        updateTelemetry();
    }

    // Decides the one active mode. No subsystem state is touched here.
    private void updateRobotMode(boolean shootHeld) {
        if (!shootHeld) {
            robotMode = RobotMode.DRIVE;
            return;
        }

        if (robotMode == RobotMode.DRIVE) {
            robotMode = RobotMode.SPIN_UP;
            shootTimer.reset();
        }

        double targetVelocity = getFlywheelTargetVelocity();
        double velocityError = Math.abs(flywheelVelocity - targetVelocity);
        boolean deployed = shootTimer.milliseconds() >= deployDelayMs;

        if (robotMode == RobotMode.SPIN_UP && deployed && velocityError <= targetVelocity * startToleranceFraction) {
            robotMode = RobotMode.FEED;
        } else if (robotMode == RobotMode.FEED && velocityError > targetVelocity * stopToleranceFraction) {
            robotMode = RobotMode.SPIN_UP;
        }
    }

    // Every subsystem state is set here from the mode, so no two commands can disagree.
    private void resolveStates(boolean wedgeTogglePressed, boolean intakeRequested, boolean reverseRequested) {
        switch (robotMode) {
            case DRIVE:
                flywheelState = FlywheelState.DRIVING;
                feetState = FeetState.RETRACTED;

                if (wedgeTogglePressed) {
                    wedgeState = (wedgeState == WedgeState.EXTENDED) ? WedgeState.RETRACTED : WedgeState.EXTENDED;
                }

                // Reverse has top priority so a jam can always be cleared, stage 1 and stage 2 together.
                if (reverseRequested) {
                    stage1IntakeState = Stage1IntakeState.REVERSE;
                    stage2IntakeState = Stage2IntakeState.REVERSE;
                } else if (intakeRequested || wedgeState == WedgeState.EXTENDED) {
                    stage1IntakeState = Stage1IntakeState.INTAKE;
                    stage2IntakeState = Stage2IntakeState.STATIC;
                } else {
                    stage1IntakeState = Stage1IntakeState.STATIC;
                    stage2IntakeState = Stage2IntakeState.STATIC;
                }
                break;

            case SPIN_UP:
                flywheelState = FlywheelState.RUNNING;
                feetState = FeetState.EXTENDED;
                wedgeState = WedgeState.RETRACTED;
                stage1IntakeState = Stage1IntakeState.STATIC;
                stage2IntakeState = Stage2IntakeState.STATIC;
                break;

            case FEED:
                flywheelState = FlywheelState.RUNNING;
                feetState = FeetState.EXTENDED;
                wedgeState = WedgeState.RETRACTED;
                stage1IntakeState = Stage1IntakeState.INTAKE;
                stage2IntakeState = Stage2IntakeState.TRANSFER;
                break;
        }

        // Compression and cam have no driver input yet, so they stay at their default positions.
        compressionState = CompressionState.POLLEN;
        camState = CamState.POLLEN;
    }

    private void updateDrive() {
        if (robotMode != RobotMode.DRIVE && lockDriveWhileShooting) {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFrontCommand = 0;
            rightFrontCommand = 0;
            leftBackCommand = 0;
            rightBackCommand = 0;
            return;
        }

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

        leftFrontCommand = leftFrontPower;
        rightFrontCommand = rightFrontPower;
        leftBackCommand = leftBackPower;
        rightBackCommand = rightBackPower;
    }

    private void applyOutputs() {
        switch (flywheelState) {
            case STATIC:    flywheelMotor.setPower(flywheelPower[0]); break;
            case RUNNING:   flywheelMotor.setVelocity(getFlywheelTargetVelocity()); break;
            case DRIVING:   flywheelMotor.setPower(flywheelPower[2]); break;
        }

        switch (stage1IntakeState) {
            case INTAKE:    stage1IntakeMotor.setPower(1); break;
            case REVERSE:   stage1IntakeMotor.setPower(-1); break;
            case STATIC:    stage1IntakeMotor.setPower(0); break;
        }

        switch (stage2IntakeState) {
            case TRANSFER:  stage2IntakeMotor.setPower(1); break;
            case REVERSE:   stage2IntakeMotor.setPower(-1); break;
            case STATIC:    stage2IntakeMotor.setPower(0); break;
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

    // Sends to Panels when enabled, otherwise to the Driver Station. Panels forwards to the Driver Station on update.
    private void log(String key, Object value) {
        if (enablePanelsTelemetry && panelsTelemetry != null) {
            panelsTelemetry.addData(key, value);
        } else {
            telemetry.addData(key, value);
        }
    }

    private String getFeedStatus() {
        switch (robotMode) {
            case DRIVE:
                return "Idle";
            case SPIN_UP:
                return shootTimer.milliseconds() < deployDelayMs ? "Deploying feet and wedge" : "Spinning up";
            case FEED:
                return "Feeding";
        }
        return "Unknown";
    }

    private String getWarnings(double loopMs, double voltage) {
        StringBuilder warnings = new StringBuilder();

        if (voltage > 0 && voltage < lowBatteryVolts) warnings.append("LOW BATTERY; ");
        if (loopMs > slowLoopMs) warnings.append("SLOW LOOP; ");

        // These two should never trigger. If they do, an interlock is broken.
        if (robotMode != RobotMode.DRIVE && wedgeState == WedgeState.EXTENDED) {
            warnings.append("INTERLOCK: wedge down while shooting; ");
        }
        if (lockDriveWhileShooting && feetState == FeetState.EXTENDED
                && (leftFrontCommand != 0 || rightFrontCommand != 0 || leftBackCommand != 0 || rightBackCommand != 0)) {
            warnings.append("INTERLOCK: driving with feet out; ");
        }
        if (stage2IntakeState == Stage2IntakeState.TRANSFER && flywheelState != FlywheelState.RUNNING) {
            warnings.append("INTERLOCK: feeding without flywheel running; ");
        }

        if (verboseTelemetry) {
            if (flywheelMotor.isOverCurrent()) warnings.append("FLYWHEEL OVERCURRENT; ");
            if (stage1IntakeMotor.isOverCurrent()) warnings.append("STAGE 1 OVERCURRENT; ");
            if (stage2IntakeMotor.isOverCurrent()) warnings.append("STAGE 2 OVERCURRENT; ");
        }

        return warnings.length() == 0 ? "none" : warnings.toString();
    }

    private void updateTelemetry() {
        double targetVelocity = getFlywheelTargetVelocity();
        double velocityError = flywheelVelocity - targetVelocity;
        double velocityPercent = targetVelocity > 0 ? flywheelVelocity / targetVelocity * 100.0 : 0;
        double startBand = targetVelocity * startToleranceFraction;
        double stopBand = targetVelocity * stopToleranceFraction;
        double deployRemainingMs = robotMode == RobotMode.DRIVE ? 0 : Math.max(0, deployDelayMs - shootTimer.milliseconds());
        double voltage = voltageSensor != null ? voltageSensor.getVoltage() : 0;
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // Status
        log("Mode", robotMode);
        log("Feed status", getFeedStatus());
        log("Warnings", getWarnings(loopMs, voltage));

        // Flywheel
        log("Flywheel state", flywheelState);
        log("Flywheel velocity (ticks per sec)", flywheelVelocity);
        log("Flywheel target (ticks per sec)", targetVelocity);
        log("Flywheel error (ticks per sec)", velocityError);
        log("Flywheel percent of target", String.format("%.1f", velocityPercent));
        log("Within start band (feed allowed)", Math.abs(velocityError) <= startBand);
        log("Within stop band (feed holds)", Math.abs(velocityError) <= stopBand);
        log("Deploy delay remaining (ms)", String.format("%.0f", deployRemainingMs));

        // Mechanisms
        log("Stage 1 intake", stage1IntakeState);
        log("Stage 2 intake", stage2IntakeState);
        log("Feet", feetState);
        log("Wedge", wedgeState);
        log("Compression", compressionState);
        log("Cam", camState);
        log("Drive locked", robotMode != RobotMode.DRIVE && lockDriveWhileShooting);

        // Inputs
        log("Input shoot (a)", shootInput);
        log("Input wedge toggle (b)", wedgeToggleInput);
        log("Input intake (right trigger)", intakeInput);
        log("Input reverse (left trigger)", reverseInput);

        // Drive commands
        log("Drive LF / RF", String.format("%.2f / %.2f", leftFrontCommand, rightFrontCommand));
        log("Drive LB / RB", String.format("%.2f / %.2f", leftBackCommand, rightBackCommand));

        // System
        log("Battery (V)", String.format("%.2f", voltage));
        log("Loop time (ms)", String.format("%.1f", loopMs));

        if (verboseTelemetry) {
            log("Flywheel current (A)", String.format("%.2f", flywheelMotor.getCurrent(CurrentUnit.AMPS)));
            log("Stage 1 current (A)", String.format("%.2f", stage1IntakeMotor.getCurrent(CurrentUnit.AMPS)));
            log("Stage 2 current (A)", String.format("%.2f", stage2IntakeMotor.getCurrent(CurrentUnit.AMPS)));
        }

        if (enablePanelsTelemetry && panelsTelemetry != null) {
            panelsTelemetry.update(telemetry);
        } else {
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        flywheelMotor.setPower(0);
        stage1IntakeMotor.setPower(0);
        stage2IntakeMotor.setPower(0);
    }
}