package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.line;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * BLUE RIGHT auto.
 *
 * Coordinates are Pedro field coordinates: 144 by 144 inches, origin in the bottom left of the
 * Panels field view, heading 0 = facing +x, positive heading = counter clockwise.
 * The blue alliance wall is the TOP of the map (y = 144) and the blue driver faces down (-y).
 * "Right" for the blue driver is therefore the LEFT side of the map (small x).
 *
 * Everything marked MEASURE is a best guess from your description. Change those numbers live in the
 * Panels Configurables tab while the OpMode is in INIT and watch the Field tab update.
 *
 * This is a LinearOpMode on purpose: the routine below reads top to bottom like a recipe.
 */
@Configurable
@Autonomous(name = "Blue Right Auto", group = "Auto")
public class BlueRightAuto extends LinearOpMode {

    // =========================================================================================
    // FIELD POSITIONS (inches and degrees)
    // =========================================================================================

    // Start: back flat against the blue wall, one tile in from the right corner, facing the field.
    // TeleOp reset is x 9, y 135 (corner), so one tile (24 in) over is x 33.
    public static double startX = 33;
    public static double startY = 135;
    public static double startHeadingDeg = 180;

    // Shooting spot: a little forward and a little to the right of the start. MEASURE
    // Its heading is computed automatically so the shooter points at the hive.
    public static double shootX = 28;
    public static double shootY = 122;

    // What we aim at (same numbers as TeleOp-Stupid). shooterOffsetDeg is the angle from the
    // front of the bot to the direction the shooter fires, same as aimOffsetDeg in TeleOp-Stupid.
    public static double hiveX = 70.75;
    public static double hiveY = 70.75;
    public static double shooterOffsetDeg = 0;

    // Garden: the four pollen on the ground in the corner. The bot turns 90 degrees right of its
    // start heading (to face the side wall) and drives into the wall. MEASURE
    public static double gardenWallX = 9;        // 9 = center of an 18 inch bot with its front on the wall
    public static double gardenY = 132;
    public static double gardenHeadingDeg = 180; // start heading -90, turned 90 degrees right

    // Flower on the right side wall. MEASURE ALL THREE
    // standoff = where the bot stops to drop the wedge, contact = where it ends up pushed in
    public static double flowerStandoffX = 28;
    public static double flowerContactX = 141.5;
    public static double flowerY = 100;
    public static double flowerHeadingDeg = 180; // front of the bot faces the wall with the flower

    // The other flower and the other shooting spot are these same numbers mirrored across x = 70.75.
    public static double fieldCenterX = 70.75;

    // =========================================================================================
    // TIMING (seconds)
    // =========================================================================================
    public static double spinupMinSec = 0.6;     // always wait at least this long before feeding
    public static double spinupMaxSec = 2.0;     // give up waiting for flywheel speed after this
    public static double shootFeedSec = 2.5;     // how long the feed runs to empty four pollen
    public static double pickupDwellSec = 1.0;   // time spent pushing into the garden wall
    public static double flowerDwellSec = 3.0;   // time spent pushing into a flower with intake running
    public static double feetSettleSec = 0.25;
    public static double wedgeSettleSec = 0.3;
    public static double wedgeWaypointDelaySec = 0.3; // same as TeleOp-Smart

    public static double turnToleranceDeg = 3;
    public static double turnSettleSec = 0.15;
    public static double turnTimeoutSec = 2.0;
    public static double pathTimeoutSec = 6.0;
    public static double wallPathTimeoutSec = 3.0;

    // Heading finishes turning at this fraction of the path, so the bot arrives already aimed
    public static double turnEndT = 0.85;

    public static boolean shootPreload = true;

    // =========================================================================================
    // MECHANISMS (values copied from TeleOp-Stupid and TeleOp-Smart, keep them in sync)
    // =========================================================================================
    public static double flywheelTargetVelocity = 2800 * 0.45;  // ticks per second, shoot speed
    public static double flywheelReadyFraction = 0.975;

    public static double intakePower = 1.0;       // stage 1 while collecting
    public static double feedStage1Power = 1.0;
    public static double feedStage2Power = 1.0;

    public static boolean useFeetWhenShooting = true;
    public static double[] footPositions = { 0.14, 0.0 };         // { Extended, Retracted }
    public static double[] wedgePositions = { 1.0, 0.5, 0.75 };   // { Extended, Retracted, Waypoint }
    public static double compressionPollen = 1.0;
    public static double camPollen = 0.0;

    // =========================================================================================
    // INTERNALS
    // =========================================================================================
    private Follower follower;
    private TelemetryManager panelsTelemetry;

    private DcMotorEx flywheel, stage1Intake, stage2Intake;
    private Servo rightFoot, leftFoot, wedge, compression, cam;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private String step = "init";

    private Pose startPose, shootA, gardenTurn, gardenEnd;
    private Pose flowerStandoffA, flowerContactA;
    private Pose shootB, flowerStandoffB, flowerContactB;
    private Path startToShoot, shootToGarden, gardenToShoot;
    private Path shootToFlowerA, flowerAIn, flowerAToShootB;
    private Path shootBToFlowerB, flowerBIn, flowerBToShootB;
    private final List<Path> allPaths = new ArrayList<>();
    private final List<Pose> keyPoses = new ArrayList<>();

    @Override
    public void runOpMode() {
        initMechanisms();
        follower = Constants.create(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // INIT loop: paths are rebuilt every loop so edits in Panels show up right away.
        // The follower is idle here, so the drivetrain does not move.
        while (!isStarted() && !isStopRequested()) {
            buildPaths();
            follower.setPose(startPose);
            follower.update();
            PanelsDraw.draw(allPaths, keyPoses, startPose); // PREVIEW
            panelsTelemetry.debug("status", "INIT - open the Panels Field tab to see the planned route");
            panelsTelemetry.debug("startPose", startPose.toString());
            panelsTelemetry.update(telemetry);
        }
        if (isStopRequested()) return;

        buildPaths();
        follower.setPose(startPose);
        follower.update();
        autoTimer.reset();

        // ---------------------------------------------------------------------------------
        // 1. Preload: start -> shooting spot, shoot
        // ---------------------------------------------------------------------------------
        startFlywheel();
        drive("1 start to shoot spot", startToShoot, pathTimeoutSec);
        if (shootPreload) shoot("1 shoot preload");

        // ---------------------------------------------------------------------------------
        // 2. Garden: turn 90 right, intake while driving into the wall, come back, shoot
        // ---------------------------------------------------------------------------------
        turnTo("2 turn right to the garden", gardenTurn);
        intakeOn();
        drive("2 drive into the garden wall", shootToGarden, wallPathTimeoutSec);
        waitSec("2 intaking at the wall", pickupDwellSec);
        startFlywheel();
        drive("2 back to shoot spot", gardenToShoot, pathTimeoutSec);
        shoot("2 shoot garden pollen");

        // ---------------------------------------------------------------------------------
        // 3. Flower A (right side wall), then shoot from the mirrored spot on the other side
        // ---------------------------------------------------------------------------------
        flowerPickup("3", shootToFlowerA, flowerAIn);
        startFlywheel();
        drive("3 flower A to mirrored shoot spot", flowerAToShootB, pathTimeoutSec);
        shoot("3 shoot flower A pollen");

        // ---------------------------------------------------------------------------------
        // 4. Flower B (the mirrored flower), then back to the mirrored shoot spot
        // ---------------------------------------------------------------------------------
        flowerPickup("4", shootBToFlowerB, flowerBIn);
        startFlywheel();
        drive("4 flower B back to shoot spot", flowerBToShootB, pathTimeoutSec);
        shoot("4 shoot flower B pollen");

        stopEverything();
        step = "done";
        while (opModeIsActive()) tick();
    }

    // =========================================================================================
    // PATHS
    // =========================================================================================
    private void buildPaths() {
        PoseFactory right = PoseFactory.degrees();
        PoseFactory left = right.mirrorX(fieldCenterX);   // x becomes 144 - x, heading is mirrored too

        startPose = right.of(startX, startY, startHeadingDeg);
        shootA = aimedAtHive(right.of(shootX, shootY, 0));
        gardenTurn = shootA.withHeading(Math.toRadians(gardenHeadingDeg));
        gardenEnd = right.of(gardenWallX, gardenY, gardenHeadingDeg);

        flowerStandoffA = right.of(flowerStandoffX, flowerY, flowerHeadingDeg);
        flowerContactA = right.of(flowerContactX, flowerY, flowerHeadingDeg);

        shootB = aimedAtHive(left.of(shootX, shootY, 0));
        flowerStandoffB = left.of(flowerStandoffX, flowerY, flowerHeadingDeg);
        flowerContactB = left.of(flowerContactX, flowerY, flowerHeadingDeg);

        // line(from, to) then pick how the heading behaves:
        //   .linear(from, to, endT) turns smoothly from the first heading to the second
        //   .constant(pose)         holds one heading the whole way
        startToShoot = line(startPose, shootA).linear(startPose, shootA, turnEndT);
        shootToGarden = line(gardenTurn, gardenEnd).constant(gardenEnd);
        gardenToShoot = line(gardenEnd, shootA).linear(gardenEnd, shootA, turnEndT);

        shootToFlowerA = line(shootA, flowerStandoffA).linear(shootA, flowerStandoffA, turnEndT);
        flowerAIn = line(flowerStandoffA, flowerContactA).constant(flowerContactA);
        flowerAToShootB = line(flowerContactA, shootB).linear(flowerContactA, shootB, turnEndT);

        shootBToFlowerB = line(shootB, flowerStandoffB).linear(shootB, flowerStandoffB, turnEndT);
        flowerBIn = line(flowerStandoffB, flowerContactB).constant(flowerContactB);
        flowerBToShootB = line(flowerContactB, shootB).linear(flowerContactB, shootB, turnEndT);

        allPaths.clear();
        allPaths.add(startToShoot);
        allPaths.add(shootToGarden);
        allPaths.add(gardenToShoot);
        allPaths.add(shootToFlowerA);
        allPaths.add(flowerAIn);
        allPaths.add(flowerAToShootB);
        allPaths.add(shootBToFlowerB);
        allPaths.add(flowerBIn);
        allPaths.add(flowerBToShootB);

        keyPoses.clear();
        keyPoses.add(startPose);
        keyPoses.add(shootA);
        keyPoses.add(gardenEnd);
        keyPoses.add(flowerStandoffA);
        keyPoses.add(flowerContactA);
        keyPoses.add(shootB);
        keyPoses.add(flowerStandoffB);
        keyPoses.add(flowerContactB);
    }

    /** Same math as the auto aim in TeleOp-Stupid: face the hive, plus the shooter offset. */
    private Pose aimedAtHive(Pose position) {
        double toHive = Math.atan2(hiveY - position.y(), hiveX - position.x());
        return position.withHeading(toHive + Math.toRadians(shooterOffsetDeg));
    }

    // =========================================================================================
    // ACTIONS - these are the building blocks the routine above is made from
    // =========================================================================================

    /** Follow a path until it finishes (or the timeout hits). */
    private void drive(String name, Path path, double timeoutSec) {
        step = name;
        follower.follow(path);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < timeoutSec) tick();
    }

    /** Turn in place to the heading of the given pose. */
    private void turnTo(String name, Pose target) {
        step = name;
        follower.hold(target);
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < turnTimeoutSec) {
            tick();
            double errorDeg = Math.toDegrees(Math.abs(Angle.normalizeSigned(target.heading() - follower.pose().heading())));
            if (errorDeg > turnToleranceDeg) settled.reset();
            else if (settled.seconds() > turnSettleSec) break;
        }
    }

    /** Stand still (the follower keeps holding the last pose) for some seconds. */
    private void waitSec(String name, double seconds) {
        step = name;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) tick();
    }

    /** Wait for the flywheel to spin up, then feed everything through. */
    private void shoot(String name) {
        step = name + " (spin up)";
        if (useFeetWhenShooting) {
            follower.stop();       // feet plant the rear wheels, so do not fight with the drivetrain
            setFeet(true);
        }
        startFlywheel();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < spinupMaxSec) {
            tick();
            if (timer.seconds() >= spinupMinSec && flywheelReady()) break;
        }

        step = name + " (feeding)";
        stage1Intake.setPower(feedStage1Power);
        stage2Intake.setPower(feedStage2Power);
        waitSec(step, shootFeedSec);

        stage1Intake.setPower(0);
        stage2Intake.setPower(0);
        flywheel.setVelocity(0);

        if (useFeetWhenShooting) {
            setFeet(false);
            waitSec(name + " (feet up)", feetSettleSec);
        }
    }

    /** Drive to the flower, drop the wedge, push in with the intake running, then lift the wedge. */
    private void flowerPickup(String label, Path toStandoff, Path pushIn) {
        drive(label + " drive to flower", toStandoff, pathTimeoutSec);
        wedgeDown();
        intakeOn();
        drive(label + " push into flower", pushIn, wallPathTimeoutSec);
        waitSec(label + " intaking from flower", flowerDwellSec);
        wedgeUp();
    }

    // Wedge uses the same two-step move as TeleOp-Smart: go to the waypoint first, then the target.
    private void wedgeDown() {
        wedge.setPosition(wedgePositions[2]);
        waitSec("wedge waypoint", wedgeWaypointDelaySec);
        wedge.setPosition(wedgePositions[0]);
        waitSec("wedge down", wedgeSettleSec);
    }

    private void wedgeUp() {
        wedge.setPosition(wedgePositions[2]);
        waitSec("wedge waypoint", wedgeWaypointDelaySec);
        wedge.setPosition(wedgePositions[1]);
        waitSec("wedge up", wedgeSettleSec);
    }

    private void setFeet(boolean extended) {
        double position = footPositions[extended ? 0 : 1];
        rightFoot.setPosition(position);
        leftFoot.setPosition(position);
    }

    private void startFlywheel() { flywheel.setVelocity(flywheelTargetVelocity); }

    private boolean flywheelReady() {
        return flywheel.getVelocity() >= flywheelTargetVelocity * flywheelReadyFraction;
    }

    private void intakeOn() {
        stage1Intake.setPower(intakePower);
        stage2Intake.setPower(0);
    }

    private void stopEverything() {
        flywheel.setVelocity(0);
        stage1Intake.setPower(0);
        stage2Intake.setPower(0);
        setFeet(false);
        wedge.setPosition(wedgePositions[1]);
        follower.stop();
    }

    /** Runs every loop: move the robot, redraw the field, update telemetry. */
    private void tick() {
        follower.update();
        Pose pose = follower.pose();

        PanelsDraw.draw(allPaths, keyPoses, pose); // PREVIEW

        panelsTelemetry.debug("step", step);
        panelsTelemetry.debug("elapsedSec", autoTimer.seconds());
        panelsTelemetry.debug("x", pose.x());
        panelsTelemetry.debug("y", pose.y());
        panelsTelemetry.debug("headingDeg", Math.toDegrees(pose.heading()));
        panelsTelemetry.debug("flywheelVelocity", flywheel.getVelocity());
        panelsTelemetry.debug("flywheelTarget", flywheelTargetVelocity);
        panelsTelemetry.update(telemetry);
    }

    private void initMechanisms() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        stage1Intake = hardwareMap.get(DcMotorEx.class, "stage1Intake");
        stage2Intake = hardwareMap.get(DcMotorEx.class, "stage2Intake");

        rightFoot = hardwareMap.get(Servo.class, "rightFoot");
        leftFoot = hardwareMap.get(Servo.class, "leftFoot");
        wedge = hardwareMap.get(Servo.class, "wedgeServo");
        compression = hardwareMap.get(Servo.class, "compressionServo");
        cam = hardwareMap.get(Servo.class, "camServo");

        // same directions as TeleOp-Stupid
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        stage1Intake.setDirection(DcMotor.Direction.REVERSE);
        stage1Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // starting positions
        setFeet(false);
        wedge.setPosition(wedgePositions[1]);
        compression.setPosition(compressionPollen);
        cam.setPosition(camPollen);
    }
}