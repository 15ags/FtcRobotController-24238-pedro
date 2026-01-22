package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .forwardZeroPowerAcceleration(-60.60893690224966)
            .lateralZeroPowerAcceleration(-80.74899864480138)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0.00003, 0.01, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0.07, 0.09, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009, 0.001, 0.0015, 0.6, 0.01))
            .centripetalScaling(0.0007);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.00138)
            .strafeTicksToInches(0.00137)
            .turnTicksToInches(0.00139)
            .leftPodY(-2.25)
            .rightPodY(1.75)
            .strafePodX(0)
            .leftEncoder_HardwareMapName("LBMotor")
            .rightEncoder_HardwareMapName("RBMotor")
            .strafeEncoder_HardwareMapName("RFMotor")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.8)
            .xVelocity(57.004353833727535)
            .yVelocity(29.61383936362166)
            .rightFrontMotorName("RFMotor")
            .rightRearMotorName("RBMotor")
            .leftRearMotorName("LBMotor")
            .leftFrontMotorName("LFMotor")
            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}
