package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(12);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RFMotor")
            .rightRearMotorName("RBMotor")
            .leftRearMotorName("LBMotor")
            .leftFrontMotorName("LFMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    /*public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .robotLength(9.5)
            .robotWidth(13.5)
            .forwardTicksToInches(0.02944875)
            .strafeTicksToInches(0.02944875)
            .turnTicksToInches(0.02944875)
            .rightFrontMotorName("RFMotor")
            .rightRearMotorName("RBMotor")
            .leftRearMotorName("LBMotor")
            .leftFrontMotorName("LFMotor")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.REVERSE)
            .rightRearEncoderDirection(Encoder.FORWARD);*/

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(1)
            .rightPodY(-1)
            .strafePodX(-2.5)
            .leftEncoder_HardwareMapName("leftFront")
            .rightEncoder_HardwareMapName("rightRear")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
