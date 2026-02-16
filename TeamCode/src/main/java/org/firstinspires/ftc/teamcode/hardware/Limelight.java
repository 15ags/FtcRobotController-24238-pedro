package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    private Limelight3A limelight;
    public boolean inZoneAuto = false;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        init();
    }

    public void init() {
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public boolean isValid() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public double getTx() {
        LLResult result = limelight.getLatestResult();
        return result != null ? result.getTx() : 0;
    }

    public double getTy() {
        LLResult result = limelight.getLatestResult();
        return result != null ? result.getTy() : 0;
    }

    public double getTa() {
        LLResult result = limelight.getLatestResult();
        return result != null ? result.getTa() : 0;
    }

    public void update(boolean driverIdle) {
        inZoneAuto = isValid() && inZoneAuto && driverIdle;
    }

    public void checkAndSetAuto(boolean driverIdle) {
        if (isValid() && driverIdle && !inZoneAuto) {
            inZoneAuto = true;
        }
    }

    public void stop() {
        limelight.stop();
    }
}