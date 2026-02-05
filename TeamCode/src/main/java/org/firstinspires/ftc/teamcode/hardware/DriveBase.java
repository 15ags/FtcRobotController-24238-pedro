package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class DriveBase {
    private DcMotorEx LBMotor = null;
    private DcMotorEx RBMotor = null;
    private DcMotorEx LFMotor = null;
    private DcMotorEx RFMotor = null;
    public static final double basePower = 3000;
    public DriveBase(HardwareMap hardwareMap){
        LBMotor  = hardwareMap.get(DcMotorEx.class, "LBMotor");
        RBMotor  = hardwareMap.get(DcMotorEx.class, "RBMotor");
        LFMotor  = hardwareMap.get(DcMotorEx.class, "LFMotor");
        RFMotor  = hardwareMap.get(DcMotorEx.class, "RFMotor");

        initMotors();
    }

    private void initMotors() {
        LBMotor.setDirection(DcMotorEx.Direction.FORWARD);
        RBMotor.setDirection(DcMotorEx.Direction.REVERSE);
        LFMotor.setDirection(DcMotorEx.Direction.REVERSE);
        RFMotor.setDirection(DcMotorEx.Direction.REVERSE);

        LBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        LBMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void baseTele(Gamepad gamepad1){
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.right_trigger - gamepad1.left_trigger;
        //double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;

        double LFPower = Range.clip(drive + strafe + twist, -1.0, 1.0);
        double RFPower = Range.clip(drive - strafe - twist, -1.0, 1.0);
        double LBPower = Range.clip(drive - strafe + twist, -1.0, 1.0);
        double RBPower = Range.clip(drive + strafe - twist, -1.0, 1.0);

        double LFTarget = LFPower*basePower;
        double RFTarget = RFPower*basePower;
        double LBTarget = LBPower*basePower;
        double RBTarget = RBPower*basePower;

        LFMotor.setVelocity(LFTarget);
        RFMotor.setVelocity(RFTarget);
        LBMotor.setVelocity(LBTarget);
        RBMotor.setVelocity(RBTarget);
    }

    public void stop() {
        LFMotor.setVelocity(0);
        RFMotor.setVelocity(0);
        LBMotor.setVelocity(0);
        RBMotor.setVelocity(0);
    }
}
