package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "autoRed1", group = "Examples")
public class autoRed1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private Paths paths;
    private final Pose startPose = new Pose(83.813, 11.456, Math.toRadians(90));

    private DcMotorEx pickUp = null;
    private DcMotorEx inter = null;

    // You are not allowed to judge I am sleep deprived
    private DcMotorEx rightPelvis = null;
    private DcMotorEx leftPelvis = null;


    public static class Paths {
        public PathChain launch1;
        public PathChain prePickUp1;
        public PathChain pickUp1;
        public PathChain launch2;
        public PathChain prePickUp2;
        public PathChain pickUp2;
        public PathChain launch3;
        public PathChain prePickUp3;
        public PathChain pickUp3;
        public PathChain launch4;

        public Paths(Follower follower) {
            launch1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.813, 11.456),

                                    new Pose(100.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(43))

                    .build();

            prePickUp1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 100.000),

                                    new Pose(100.000, 83.800)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(180))

                    .build();

            pickUp1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 83.800),

                                    new Pose(125.000, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            launch2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 83.500),

                                    new Pose(100.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(43))

                    .build();

            prePickUp2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 100.000),

                                    new Pose(100.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(180))

                    .build();

            pickUp2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 60.000),

                                    new Pose(125.000, 59.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            launch3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 59.500),

                                    new Pose(100.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(43))

                    .build();

            prePickUp3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 100.000),

                                    new Pose(100.000, 35.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(180))

                    .build();

            pickUp3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 35.500),

                                    new Pose(124.000, 35.300)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            launch4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.000, 35.300),

                                    new Pose(100.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(43))

                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.launch1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    // Turn on pelvises
                    leftPelvis.setVelocity(1800);
                    rightPelvis.setVelocity(1800);// Adjust velocity as needed
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                // Wait 2 seconds
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    // Turn on inter and pickUp
                    inter.setVelocity(500);  // Adjust velocity as needed
                    pickUp.setVelocity(500);  // Adjust velocity as needed
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                // Wait 2 seconds then turn all off
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    leftPelvis.setVelocity(0);
                    rightPelvis.setVelocity(0);
                    inter.setVelocity(0);
                    pickUp.setVelocity(0);
                    setPathState(4);
                }
                break;
            case 4:
                follower.followPath(paths.prePickUp1, true);
                pickUp.setVelocity(500);
                inter.setVelocity(500);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickUp1, true);
                    pickUp.setVelocity(0);
                    inter.setVelocity(0);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.launch2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    // Turn on pelvises
                    leftPelvis.setVelocity(1800);
                    rightPelvis.setVelocity(1800);
                    actionTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                // Wait 2 seconds
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    // Turn on inter and pickUp
                    inter.setVelocity(500);
                    pickUp.setVelocity(500);
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                // Wait 2 seconds then turn all off
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    rightPelvis.setVelocity(0);
                    leftPelvis.setVelocity(0);
                    inter.setVelocity(0);
                    pickUp.setVelocity(0);
                    setPathState(10);
                }
                break;
            case 10:
                follower.followPath(paths.prePickUp2, true);
                pickUp.setVelocity(500);
                inter.setVelocity(500);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickUp2, true);
                    pickUp.setVelocity(0);
                    inter.setVelocity(0);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.launch3, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    // Turn on pelvises
                    rightPelvis.setVelocity(1800);
                    leftPelvis.setVelocity(1800);
                    actionTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14:
                // Wait 2 seconds
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    // Turn on inter and pickUp
                    inter.setVelocity(500);
                    pickUp.setVelocity(500);
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15:
                // Wait 2 seconds then turn all off
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    leftPelvis.setVelocity(0);
                    rightPelvis.setVelocity(0);
                    inter.setVelocity(0);
                    pickUp.setVelocity(0);
                    setPathState(16);
                }
                break;
            case 16:
                follower.followPath(paths.prePickUp3, true);
                pickUp.setVelocity(500);
                inter.setVelocity(500);
                setPathState(17);
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickUp3, true);
                    pickUp.setVelocity(0);
                    inter.setVelocity(0);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(paths.launch4, true);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    // Turn on pelvises
                    leftPelvis.setVelocity(1800);
                    rightPelvis.setVelocity(1800);
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;
            case 20:
                // Wait 2 seconds
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    // Turn on inter and pickUp
                    inter.setVelocity(500);
                    pickUp.setVelocity(500);
                    actionTimer.resetTimer();
                    setPathState(21);
                }
                break;
            case 21:
                // Wait 2 seconds then turn all off
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    leftPelvis.setVelocity(0);
                    rightPelvis.setVelocity(0);
                    inter.setVelocity(0);
                    pickUp.setVelocity(0);
                    setPathState(-1);  // End autonomous
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
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

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(startPose);

        pickUp = hardwareMap.get(DcMotorEx.class,"pickUp");
        rightPelvis = hardwareMap.get(DcMotorEx.class, "rightPelvis");
        leftPelvis = hardwareMap.get(DcMotorEx.class, "leftPelvis");
        inter = hardwareMap.get(DcMotorEx.class, "launch");
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
