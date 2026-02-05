package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ScoringMotors {
    private DcMotorEx intake = null;
    private DcMotorEx launch = null;
    private DcMotorEx middle = null;
    private double targetLaunchVel = 0;
    private int launchState = 0;
    private ElapsedTime launchTimer = new ElapsedTime();
    public static final double middleMax = 2500;
    public static final double intakeV = 2500;
    public static final double maxLaunchVelocity = 1300;
    public static final double launchDelay = 1;
    public ScoringMotors(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        launch = hardwareMap.get(DcMotorEx.class, "Launch");
        middle = hardwareMap.get(DcMotorEx.class, "middle");


        initMotors();
    }

    private void initMotors() {
        middle.setDirection(DcMotorEx.Direction.FORWARD);
        launch.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        middle.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launch.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        middle.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        middle.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(500, 100.0, 0.0, 0.1);
        launch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void scoringMotorsTele(Gamepad gamepad2){
        double launchInput = gamepad2.left_stick_y;
        targetLaunchVel = launchInput*maxLaunchVelocity;
        double middleV = gamepad2.right_stick_y*middleMax;
        double inInput = gamepad2.right_trigger-gamepad2.left_trigger;
        double inTarget = inInput*intakeV;


        middle.setVelocity(middleV);
        launch.setVelocity(targetLaunchVel);
        intake.setVelocity(inTarget);
    }

    public void startLaunchSequence() {
        launchState = 0;
        launchTimer.reset();
    }
    public boolean updateLaunchSequence() {
        switch (launchState) {
            case 0:
                launch.setVelocity(maxLaunchVelocity);
                launchTimer.reset();
                launchState = 1;
                return false;
            case 1:
                if (launchTimer.seconds() > launchDelay) {
                    middle.setVelocity(middleMax);
                    launchTimer.reset();
                    launchState=2;
                    return false;
                }
                break;
            case 2:
                if (launchTimer.seconds() > launchDelay) {
                intake.setVelocity(intakeV);
                launchTimer.reset();
                launchState=3;
                return false;
                }
                break;
            case 3:
                if (launchTimer.seconds() > launchDelay) {
                    stop();
                    launchTimer.reset();
                    launchState=4;
                    return true;
                }
                break;
        }
        return false;
    }
    public double getLaunchVel() {
        return launch.getVelocity();
    }
    public double getTargetLaunchVel() {
        return targetLaunchVel;
    }
    public void stop() {
        launch.setVelocity(0);
        middle.setVelocity(0);
        intake.setVelocity(0);
        launchTimer.reset();
    }
}
