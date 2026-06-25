package frc.robot.utilities;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.HashMap;

public class ConnectionAlertHelper {
  private static HashMap<String, Alert> ALERTS = new HashMap<>();

  public static void reportConnected(String device, boolean connected) {
    ALERTS
        .computeIfAbsent(device, (_key) -> new Alert(device + " is disconnected", AlertType.kError))
        .set(!connected);
  }

  // Static helper class
  private ConnectionAlertHelper() {}
}
