package frc.robot.utilities;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.Preferences;

public class ChassisSwitcher {
  // DO NOT CHANGE THIS VALUE! IT IS SET IN EACH ROBORIO!
  public static final String PREF_KEY_CHASSISTYPE = "RobotName";

  public static ChassisType getChassisType() {
    // This sets the robot name
    // Preferences.initString(PREF_KEY_CHASSISTYPE, ChassisType.Competition2026.name());
    try {
      String chassisType = Preferences.getString(PREF_KEY_CHASSISTYPE, ChassisType.Unknown.name());
      logf("returning chassisType '%s'", chassisType);
      return ChassisType.valueOf(chassisType);
    } catch (Exception e) {
      logf("returning unknown for unexpected ChassisType");
      return ChassisType.Unknown;
    }
  }
}
