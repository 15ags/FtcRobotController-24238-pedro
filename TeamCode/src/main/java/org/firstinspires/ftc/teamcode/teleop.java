package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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
    public static Follower follower;
    static TelemetryManager telemetryM;
    private final Pose startPose = new Pose(123.8, 122.6, Math.toRadians(37));
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


        driveBase = new DriveBase(hardwareMap);

        scoringMotors = new ScoringMotors(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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
        driveBase.baseTele(gamepad1);
        scoringMotors.scoringMotorsTele(gamepad2);

        follower.update();
        draw();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("LaunchSpeed", scoringMotors.getLaunchVel());
        telemetry.addData("LaunchError", scoringMotors.getLaunchVel()- scoringMotors.getTargetLaunchVel());

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
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