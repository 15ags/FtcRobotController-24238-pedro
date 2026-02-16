package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.hardware.ScoringMotors;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="teleop", group="Iterative OpMode")
public class teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveBase driveBase;
    private ScoringMotors scoringMotors;
    private Limelight limelight;
    static TelemetryManager telemetryM;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        driveBase = new DriveBase(hardwareMap);

        scoringMotors = new ScoringMotors(hardwareMap);

        limelight = new Limelight(hardwareMap);


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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
        boolean driverIdle = Math.abs(gamepad2.left_stick_y) < 0.1;

        driveBase.baseTele(gamepad1);
        if (!driverIdle) {
            scoringMotors.scoringMotorsTele(gamepad2);
        }

        limelight.update(driverIdle);

        if (limelight.isValid()) {
            telemetryM.addData("Tx", limelight.getTx());
            telemetryM.addData("Ty", limelight.getTy());
            telemetryM.addData("Ta", limelight.getTa());

            if (limelight.isAutoActive()) {
                scoringMotors.preLaunch();
            }
        } else {
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