package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.10)
            /*.forwardZeroPowerAcceleration(-42)
            .lateralZeroPowerAcceleration(-61)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)*/
            //.headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.035, 0.01))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /*public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(.5)
            .strafePodX(-2)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);*/

    /*public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardPodY(-2)
            .strafePodX(.5)
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardEncoder_HardwareMapName("FL")
            .strafeEncoder_HardwareMapName("BL")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .forwardTicksToInches(.00198453)
            .strafeTicksToInches(.00198927)
            ;*/

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            /*.forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)*/
            .leftPodY(3.75)
            .rightPodY(3.75)
            .strafePodX(2.5)
            .leftEncoder_HardwareMapName("outtake")
            .rightEncoder_HardwareMapName("intake")
            .strafeEncoder_HardwareMapName("frontLeft")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            /*.forwardTicksToInches(0)
            .strafeTicksToInches(0)
            .turnTicksToInches(0)*/
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //.xVelocity(76.5)
            //.yVelocity(61)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                //.twoWheelLocalizer(localizerConstants)
                //.pinpointLocalizer(localizerConstants)
                .threeWheelLocalizer(localizerConstants)
                .build()
                ;
    }
}
