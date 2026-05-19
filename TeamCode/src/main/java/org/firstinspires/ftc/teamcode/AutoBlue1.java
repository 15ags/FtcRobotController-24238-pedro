package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.hardware.ScoringMotors;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "autoBlue1", group = "Examples")
public class AutoBlue1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Paths paths;
    private final Pose startPose = new Pose(20.2, 122.6, Math.toRadians(144));
    private ScoringMotors scoringMotors;






    public static class Paths {
        public PathChain prePickUp1;
        public PathChain pickUp1;
        public PathChain launch2;
        public PathChain prePickUp2;
        public PathChain pickUp2;
        public PathChain launch3;
        public PathChain prePickUp3;
        public PathChain pickUp3;

        public Paths(Follower follower) {
            prePickUp1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.200, 122.600),

                                    new Pose(54.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                    .build();

            pickUp1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 84.000),

                                    new Pose(19.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            launch2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 84.000),

                                    new Pose(20.200, 122.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                    .build();

            prePickUp2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.200, 122.600),

                                    new Pose(54.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                    .build();

            pickUp2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 60.000),

                                    new Pose(11.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            launch3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.000, 60.000),
                                    new Pose(44.000, 60.000),
                                    new Pose(20.200, 122.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                    .build();

            prePickUp3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.200, 122.600),

                                    new Pose(54.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                    .build();

            pickUp3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 36.000),

                                    new Pose(11.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start initial launch sequence
                scoringMotors.startLaunchSequence();
                setPathState(1);
                break;
            case 1:
                // Wait for launch sequence to complete
                if (scoringMotors.updateLaunchSequence()) {
                    follower.followPath(paths.prePickUp1);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    scoringMotors.intakeBalls();
                    follower.followPath(paths.pickUp1, 0.75, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    scoringMotors.stop();
                    scoringMotors.middlePreLaunch();
                    follower.followPath(paths.launch2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    /* Start launch sequence */
                    scoringMotors.stop();
                    scoringMotors.startLaunchSequence();
                    setPathState(5);
                }
                break;
            case 5:
                // Wait for launch sequence to complete
                if (scoringMotors.updateLaunchSequence()) {
                    follower.followPath(paths.prePickUp2);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    scoringMotors.intakeBalls();
                    follower.followPath(paths.pickUp2, 0.75, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    scoringMotors.stop();
                    scoringMotors.middlePreLaunch();
                    follower.followPath(paths.launch3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    /* Start final launch sequence */
                    scoringMotors.stop();
                    scoringMotors.startLaunchSequence();
                    setPathState(9);
                }
                break;
            case 9:
                // Wait for launch sequence to complete
                if (scoringMotors.updateLaunchSequence()) {
                    setPathState(10); // Done
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    scoringMotors.stop();
                    follower.followPath(paths.prePickUp3, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    scoringMotors.intakeBalls();
                    follower.followPath(paths.pickUp3, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    scoringMotors.stop();
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();

        scoringMotors = new ScoringMotors(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(startPose);


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}