package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Autonomous(name = "Blue-Right-Autonomous")
public class BlueRightAuto extends OpMode {

    private DcMotorEx stage1IntakeMotor, stage2IntakeMotor, flywheelMotor;
    private Servo leftFootServo, rightFootServo, wedgeServo;

    public static double shooterTargetVelocity = 1000;
    public static double toleranceFractionHigh = 0.02;
    public static double spinUpDelaySec = 2.5;
    public static double stableTimeSec = 0;

    public static double shootingDuration = 10;
    public static double[] footPositions = {0.12, 0.0}; // { Extended, Retracted }

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(83, 128, 270);

    private final Pose shot1Pose = poseFactory.of(84, 116, 270);
    private final Pose shot2Pose = poseFactory.of(84, 20, 90);

    private final Pose intake0Pose = poseFactory.of(120, 129, 0);
    private final Pose intake1Pose = poseFactory.of(132, 129, 0);
    private final Pose intake2Pose = poseFactory.of(95, 16, 270);
    private final Pose intake2MidPose = poseFactory.of(95, 24, 270);

    private final Pose drive1Pose = poseFactory.of(78, 120, 90);
    private final Pose drive2Pose = poseFactory.of(78, 16, 90);

    private final Pose park = poseFactory.of(128, 42, 180);

    private Path score1()   { return line(startPose, shot1Pose).linear(startPose, shot1Pose); }
    private Path intake0()  { return line(shot1Pose, intake0Pose).linear(shot1Pose, intake0Pose); }
    private Path intake1() { return line(intake0Pose, intake1Pose).constant(intake1Pose); }

    private Path drive1()   { return line(intake1Pose, drive1Pose).constant(shot2Pose); }
    private Path drive2()   { return line(drive1Pose, drive2Pose).constant(shot2Pose); }

    private Path score3()   { return line(drive2Pose, shot2Pose).constant(shot2Pose); }
    private Path intake2()  { return curve(shot2Pose, intake2MidPose, intake2Pose).constant(intake2Pose); }
    private Path intake3()  { return line(intake2Pose, intake2MidPose).constant(intake2Pose); }

    private Path score4()   { return line(intake2MidPose, shot2Pose).linear(intake2MidPose, shot2Pose); }

    private Path park()     { return line(shot2Pose, park).linear(shot2Pose, park); }

    private Command runIntake() {
        return Command.build()
                .setStart(() -> { stage1IntakeMotor.setPower(1); })
                .setEnd(endCondition -> { stage1IntakeMotor.setPower(0); });
    }

    private Command shootSequence() {

        final double[] shooterStartTime = {0};
        final double[] stableStartTime = {-1};

        final boolean[] shooterReady = {false};

        return Command.build()

                .setStart(() -> {
                    shooterStartTime[0] = System.nanoTime() / 1e9;
                    stableStartTime[0] = -1;
                    shooterReady[0] = false;

                    flywheelMotor.setVelocity(shooterTargetVelocity);

                    leftFootServo.setPosition(footPositions[0]);
                    rightFootServo.setPosition(footPositions[0]);
                })

                .setExecute(() -> {
                    double now = System.nanoTime() / 1e9;

                    flywheelMotor.setVelocity(shooterTargetVelocity);
                    double currentVelocity = flywheelMotor.getVelocity();
                    double velocityError = Math.abs(shooterTargetVelocity - currentVelocity);
                    double enterTolerance = shooterTargetVelocity * toleranceFractionHigh;
                    double exitTolerance = enterTolerance * 1.5;

                    if (velocityError <= enterTolerance && currentVelocity >= shooterTargetVelocity && stableStartTime[0] < 0) stableStartTime[0] = now;
                    else if (velocityError > exitTolerance) stableStartTime[0] = -1;

                    shooterReady[0] =
                            (now - shooterStartTime[0] >= spinUpDelaySec)
                                    && stableStartTime[0] >= 0
                                    && (now - stableStartTime[0] >= stableTimeSec);

                    if (shooterReady[0]) {
                        stage1IntakeMotor.setPower(1);
                        stage2IntakeMotor.setPower(1);

                    } else {
                        stage1IntakeMotor.setPower(0);
                        stage2IntakeMotor.setPower(0);
                    }
                })

                .setDone(() -> System.nanoTime() / 1e9 - shooterStartTime[0] >= shootingDuration)

                .setEnd(endCondition -> {
                    stage1IntakeMotor.setPower(0);
                    stage2IntakeMotor.setPower(0);

                    flywheelMotor.setVelocity(0);

                    leftFootServo.setPosition(footPositions[1]);
                    rightFootServo.setPosition(footPositions[1]);
                });
    }

    private Command autonomousRoutine() {
        return sequential(
                follow(follower, score1()),
                shootSequence(),

                follow(follower, intake0()),
                race(follow(follower, intake1()), runIntake()), race(runIntake(), waitMs(500)),

                follow(follower, drive1()),
                follow(follower, drive2()),
                follow(follower, score3()),
                shootSequence(),

                // tfollow(follower, intake2()),

                // follow(follower, intake3()),
                // follow(follower, score4()),

                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        stage1IntakeMotor = hardwareMap.get(DcMotorEx.class, "stage1Intake");
        stage2IntakeMotor = hardwareMap.get(DcMotorEx.class, "stage2Intake");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        leftFootServo = hardwareMap.get(Servo.class, "leftFoot");
        rightFootServo = hardwareMap.get(Servo.class, "rightFoot");
        wedgeServo = hardwareMap.get(Servo.class, "wedgeServo");

        stage1IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        schedule(autonomousRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}