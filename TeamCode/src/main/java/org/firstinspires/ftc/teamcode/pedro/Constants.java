package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;

import com.pedropathing.controllers.Controller;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;

import com.pedropathing.math.Vector2D;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");

        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(7.1594196229469125);
        c.yPodOffset.set(-1.952544985793707);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.43425378620536087);
                Controller secondaryTranslationalForward = Controller.proportional(0.1604451705280092);
                Controller primaryTranslationalLateral = Controller.proportional(0.6731003118303919);
                Controller secondaryTranslationalLateral = Controller.proportional(0.24869257964975267);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013724133601022119));
                c.brake.set(Controller.proportionalFeedforward(0.0116655135608688));

                c.headingFeedback.set(Controller.proportional(7.0219505398483895));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05290164827179887, 0.0072423210600353624));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10494567005003946, 0.05834124074880395));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016201290702587186, 0.0019382703237077578));

                c.maxAchievableForwardVelocity.set(72.76315456287934);
                c.maxAchievableStrafeVelocity.set(55.4553089599504);
                c.naturalForwardDeceleration.set(36.90488142836422);
                c.naturalStrafeDeceleration.set(87.76638190549609);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}