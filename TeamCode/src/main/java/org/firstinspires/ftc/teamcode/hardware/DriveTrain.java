package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {

    // Motors (exposed for telemetry if needed)
    private DcMotorEx lf, rf, lb, rb;
    private IMU imu;

    // === Drive tuning ===
    private final double deadband = 0.05;     // joystick deadzone
    private final double strafeScale = 1.05;  // slightly boost strafe if needed
    private final double rampRate = 0.2;     // max power change per update (simple slew)

    // === Heading hold variables ===
    private boolean headingLock = false;
    private double targetHeading = 0.0;
    private final PIDController headingPID = new PIDController(0.015, 0.0001, 0.005); // tuned gains
    private final double headingDeadbandDeg = 0.75; // small deadband for heading

    // smoothing unit-vector for heading to avoid wrap issues
    private double smoothVx = 1.0;
    private double smoothVy = 0.0;

    // for ramping
    private double prevLf = 0, prevRf = 0, prevLb = 0, prevRb = 0;

    // toggle debounce (OpMode calls drive each loop and passes button)
    private double lastToggleTimeMs = -1000;
    private final double toggleDebounceMs = 250.0;

    public DriveTrain() {
        // empty ctor; init with init(hardwareMap)
    }

    /** Initialize hardware; call from OpMode.init() */
    public void init(HardwareMap hw) {
        imu = hw.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(params);

        lf = hw.get(DcMotorEx.class, "LFMotor");
        rf = hw.get(DcMotorEx.class, "RFMotor");
        lb = hw.get(DcMotorEx.class, "LBMotor");
        rb = hw.get(DcMotorEx.class, "RBMotor");

        // motor directions - adjust if your wiring is different
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD); // Changed to REVERSE (typical for mecanum)

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        headingPID.setIntegralLimit(50.0); // protect integral
    }

    /**
     * Main drive call — call from your OpMode.loop().
     *
     * @param forward  forward power from -1..1 (positive = forward)
     * @param strafe   strafe power from -1..1 (positive = right)
     * @param turn     rotation power from -1..1 (positive = clockwise)
     * @param lockBtn  boolean button state used to toggle heading lock
     * @param nowMs    current time in milliseconds (pass runtime.milliseconds())
     * @param dt       delta seconds since last update (pass from ElapsedTime)
     */
    public void drive(double forward, double strafe, double turn, boolean lockBtn, double nowMs, double dt) {

        // Apply deadband
        forward = applyDeadband(forward);
        strafe = applyDeadband(strafe) * strafeScale;
        turn = applyDeadband(turn);

        // Handle toggle
        if (lockBtn && (nowMs - lastToggleTimeMs) > toggleDebounceMs) {
            headingLock = !headingLock;
            lastToggleTimeMs = nowMs;
            if (headingLock) {
                targetHeading = getRawHeading();
                headingPID.reset();
            } else {
                headingPID.reset(); // Clean reset on disable
            }
        }

        // Smooth heading with time-based smoothing
        double measured = getRawHeading();
        double rad = Math.toRadians(measured);
        double alpha = dt / (0.1 + dt); // time-based smoothing (tau = 0.1s)
        smoothVx = alpha * Math.cos(rad) + (1 - alpha) * smoothVx;
        smoothVy = alpha * Math.sin(rad) + (1 - alpha) * smoothVy;
        double smoothHeading = Math.toDegrees(Math.atan2(smoothVy, smoothVx));

        // Apply heading hold with turn override
        double turnCommand = turn;
        if (headingLock) {
            if (Math.abs(turn) > 0.15) {
                // Driver override - update target to current heading
                targetHeading = smoothHeading;
                headingPID.reset();
                // Reset smoothing vector to avoid jumps
                smoothVx = Math.cos(Math.toRadians(smoothHeading));
                smoothVy = Math.sin(Math.toRadians(smoothHeading));
            } else {
                double err = angleDiff(targetHeading, smoothHeading);
                if (Math.abs(err) > headingDeadbandDeg) {
                    double correction = headingPID.update(err, dt);
                    turnCommand += Range.clip(correction, -0.4, 0.4);
                }
            }
        }

        // Mecanum kinematics - FIXED: using turnCommand instead of turn
        double lfPower = forward + strafe + turnCommand;
        double rfPower = forward - strafe - turnCommand;
        double lbPower = forward - strafe + turnCommand;
        double rbPower = forward + strafe - turnCommand;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.abs(lfPower),
                Math.max(Math.abs(rfPower), Math.max(Math.abs(lbPower), Math.abs(rbPower)))));
        lfPower /= max;
        rfPower /= max;
        lbPower /= max;
        rbPower /= max;

        // Ramping (simple slew rate)
        lfPower = ramp(prevLf, lfPower, rampRate);
        rfPower = ramp(prevRf, rfPower, rampRate);
        lbPower = ramp(prevLb, lbPower, rampRate);
        rbPower = ramp(prevRb, rbPower, rampRate);

        // Store for next loop
        prevLf = lfPower;
        prevRf = rfPower;
        prevLb = lbPower;
        prevRb = rbPower;

        // Apply to motors
        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }

    // Helper: return raw imu yaw in degrees [-180,180]
    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Helper: normalized shortest difference target - current in [-180,180]
    private double angleDiff(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    }

    private double applyDeadband(double v) {
        return Math.abs(v) < deadband ? 0.0 : v;
    }

    private double ramp(double prev, double wanted, double step) {
        double diff = wanted - prev;
        if (Math.abs(diff) <= step) return wanted;
        return prev + Math.signum(diff) * step;
    }

    // Telemetry-friendly status
    public String getStatus() {
        return String.format("LF: %.2f LFenc: %d\nRF: %.2f RFenc: %d\nLB: %.2f LBenc: %d\nRB: %.2f RBenc: %d\nHLock:%b Ht:%.1f",
                lf.getPower(), lf.getCurrentPosition(),
                rf.getPower(), rf.getCurrentPosition(),
                lb.getPower(), lb.getCurrentPosition(),
                rb.getPower(), rb.getCurrentPosition(),
                headingLock, targetHeading
        );
    }

    // ==== Inner PID controller (simple, robust) ====
    private static class PIDController {
        private final double kP, kI, kD;
        private double integral = 0.0;
        private double lastError = 0.0;
        private double integralLimit = Double.POSITIVE_INFINITY;

        public PIDController(double p, double i, double d) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
        }

        public void setIntegralLimit(double limit) {
            this.integralLimit = Math.abs(limit);
        }

        public double update(double error, double dt) {
            // Clamp dt to prevent division issues
            dt = Math.max(dt, 0.001); // minimum 1ms

            integral += error * dt;
            if (integral > integralLimit) integral = integralLimit;
            if (integral < -integralLimit) integral = -integralLimit;

            double derivative = (error - lastError) / dt;
            lastError = error;

            return kP * error + kI * integral + kD * derivative;
        }

        public void reset() {
            integral = 0;
            lastError = 0;
        }
    }
}