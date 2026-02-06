package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.ScoringMotors;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="teleop", group="Iterative OpMode")
public class teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveBase driveBase;
    private ScoringMotors scoringMotors;
    //public static Follower follower;
    static TelemetryManager telemetryM;
    private Limelight3A limelight;
    private IMU imu;
    boolean inZoneAuto = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        driveBase = new DriveBase(hardwareMap);

        scoringMotors = new ScoringMotors(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        telemetryM.addLine("initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        boolean driverIdle = Math.abs(gamepad2.left_stick_y) < 0.1;

        driveBase.baseTele(gamepad1);
        if (!driverIdle) {
            scoringMotors.scoringMotorsTele(gamepad2);
        }

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose_mt2 = llResult.getBotpose_MT2();
            telemetryM.addData("Tx", llResult.getTx());
            telemetryM.addData("Ty", llResult.getTy());
            telemetryM.addData("Ta", llResult.getTa());
            if (botPose_mt2 != null) {
                double x = botPose_mt2.getPosition().x;
                double y = botPose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                boolean inRedZone = x > 2.4 && y > 2.4;
                boolean inBlueZone = x > 2.4 && y > 1.2;

                if ((inRedZone||inBlueZone) && driverIdle && !inZoneAuto) {
                    scoringMotors.preLaunch();
                    inZoneAuto = true;
                }

                if (((!inRedZone&&!inBlueZone) || !driverIdle) && inZoneAuto) {
                    inZoneAuto = false;
                }
            }
        } else {
            inZoneAuto = false;
            scoringMotors.scoringMotorsTele(gamepad2);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("LaunchSpeed", scoringMotors.getLaunchVel());
        telemetry.addData("LaunchError", scoringMotors.getLaunchVel()- scoringMotors.getTargetLaunchVel());
        telemetryM.debug("Status", "Run Time: " + runtime.toString());
        telemetryM.debug("LaunchSpeed", scoringMotors.getLaunchVel());
        telemetryM.debug("LaunchError", scoringMotors.getLaunchVel()- scoringMotors.getTargetLaunchVel());
        telemetryM.update(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    // this is just a simple test to understand branches
    @Override
    public void stop() {
        driveBase.stop();
        scoringMotors.stop();
    }
}