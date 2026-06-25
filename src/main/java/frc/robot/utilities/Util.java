package frc.robot.utilities;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

public class Util {

  public static void logf(String pattern, Object... arguments) {
    try {
      DateFormat dateFormat = new SimpleDateFormat("HH:mm:ss-SSS ");
      dateFormat.setTimeZone(TimeZone.getTimeZone("America/New_York"));
      System.out.printf((dateFormat.format(new Date()) + pattern + "\n"), arguments);
    } catch (Exception e) {
      System.err.println("\nAn error occurred while logging! Pattern: " + pattern);
      e.printStackTrace();
    }
  }

  // Take an angle and convert it to -180 to 180
  public static double normalizeAngle(double angle) {
    double a = (angle + 180) % 360;
    if (a < 0) a += 360;
    return a - 180;
  }

  // Take an angle and convert it to 0 to 360
  public static double unNormalilzeAngle(double angle) {
    double a = angle % 360;
    if (a < 0) a += 360;
    return a;
  }
}
