package frc.robot;

import com.revrobotics.Rev2mDistanceSensor;

public class ClawSensor {
    private Rev2mDistanceSensor distMXP;

    public ClawSensor(){
        distMXP = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP);
        distMXP.setAutomaticMode(true);
        distMXP.setEnabled(true);
        distMXP.setMeasurementPeriod(0.01);
        distMXP.setDistanceUnits(Rev2mDistanceSensor.Unit.kMillimeters);
        distMXP.setRangeProfile(Rev2mDistanceSensor.RangeProfile.kDefault);
    }

    public boolean IsIn(){
       return (distMXP.getRange()>=70 && distMXP.getRange()<=90) ;
    }

    public void periodic() {
        SmartDashboard.putNumber("DistanceSensorRange", distMXP.getRange());
        SmartDashboard.putBoolean("IsItInRange", IsIn();
    }
}
