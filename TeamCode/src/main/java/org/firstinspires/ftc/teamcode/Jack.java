package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagDetector;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

@Configurable
@TeleOp(name = "Jack", group = "Main")
public class Jack extends OpMode {

    private DriveTrain drive;
    private double maxLaunchVelocity;
    private DcMotorEx pickUp = null;
    private DcMotorEx inter = null;

    // You are not allowed to judge I am sleep deprived
    private DcMotorEx rightPelvis = null;
    private DcMotorEx leftPelvis = null;

    private final ElapsedTime runtime = new ElapsedTime();
    private double lastLoopTimeSec = 0.0;
    AprilTagDetector tagDetector = new AprilTagDetector();


    @Override
    public void init() {

        tagDetector.init(hardwareMap);

        drive = new DriveTrain();
        drive.init(hardwareMap);



        telemetry.addLine("Drive initialized");
        telemetry.update();
        lastLoopTimeSec = runtime.seconds();

        pickUp = hardwareMap.get(DcMotorEx.class,"pickUp");
        rightPelvis = hardwareMap.get(DcMotorEx.class, "rightPelvis");
        leftPelvis = hardwareMap.get(DcMotorEx.class, "leftPelvis");
        inter = hardwareMap.get(DcMotorEx.class, "launch");

        leftPelvis.setDirection(DcMotorEx.Direction.FORWARD);
        rightPelvis.setDirection(DcMotorEx.Direction.REVERSE);

        pickUp.setDirection(DcMotorEx.Direction.FORWARD);
        pickUp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pickUp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pickUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftPelvis.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightPelvis.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        inter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        inter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        inter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        inter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        AprilTagDetector.TagResult tag = tagDetector.getResult();

        // calculate dt (seconds) and now ms
        double nowSec = runtime.seconds();
        double dt = nowSec - lastLoopTimeSec;
        if (dt <= 0) dt = 0.001;
        lastLoopTimeSec = nowSec;

        double nowMs = runtime.milliseconds();


        if (gamepad2.xWasPressed()){
            maxLaunchVelocity = 2200;
        }

        if (gamepad2.aWasPressed()) {
            maxLaunchVelocity = 1650;
        }

        double inVel = 2000;
        double interVel = 2000;
        double pelvisInput = gamepad2.left_stick_y;
        double intakePower = (gamepad2.right_trigger-gamepad2.right_stick_y)*inVel;
        double interTarget = (gamepad2.left_trigger-gamepad2.right_stick_y)*interVel;
        // Scale to your desired maximum velocity
        // This is now your actual max speed




        double targetVelocity = pelvisInput * maxLaunchVelocity;

        // read gamepad
        double forward = -gamepad1.left_stick_y;   // forward positive
        double strafe  =  gamepad1.right_trigger - gamepad1.left_trigger;   // right positive
        double turn    =  gamepad1.right_stick_x * 0.75;  // clockwise positive

        boolean lockBtn = gamepad1.x; // press X to toggle heading lock


        leftPelvis.setVelocity(targetVelocity);
        rightPelvis.setVelocity(targetVelocity);
        inter.setVelocity(interTarget);
        pickUp.setVelocity(intakePower);


        // feed into drivetrain (non-blocking)
        drive.drive(forward, strafe, turn, lockBtn, nowMs, dt);

        // telemetry (small, useful)
        telemetry.addData("Heading", String.format("%.1f", drive.getRawHeading()));
        telemetry.addData("Status", drive.getStatus());
        telemetry.addData("Distance X (cm)", "%.1f", tag.x);
        telemetry.addData("Distance Y (cm)", "%.1f", tag.y);
        telemetry.addData("Distance Z (cm)", "%.1f", tag.z);
        telemetry.addData("Id:", "%s", tag.id);
        telemetry.update();
    }
}