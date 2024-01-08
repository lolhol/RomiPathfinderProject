package frc.robot.util;

public class MathUtil {

  public static double normaliseDeg(double a) {
    return (a % 360 + 360) % 360;
  }

  public static double diffDeg(double a, double b) {
    double sign = 1;
    if (b < a) {
      double tmp = b;
      b = a;
      a = tmp;
      sign = -1;
    }

    return sign * (b - a < 180 ? b - a : (b - a) - 360);
  }

  public static double wrap360(double angle) {
    return angle < 0 ? angle : 360 - angle;
  }

  public static int[] getPointDistFromWithAngle(
      int[] curPoint,
      double curAngle,
      int dist) {
    double angleRadians = Math.toRadians(curAngle);
    double x = curPoint[0] + dist * Math.cos(angleRadians);
    double y = curPoint[1] + dist * Math.sin(angleRadians);
    return new int[] { (int) x, (int) y };
  }

  public static double calculateAngle(
      int[] pointA,
      int[] pointB,
      int[] pointC) {
    double[] vectorAB = { pointB[0] - pointA[0], pointB[1] - pointA[1] };
    double[] vectorBC = { pointC[0] - pointB[0], pointC[1] - pointB[1] };

    double dotProduct = vectorAB[0] * vectorBC[0] + vectorAB[1] * vectorBC[1];
    double magnitudeAB = Math.sqrt(
        vectorAB[0] * vectorAB[0] + vectorAB[1] * vectorAB[1]);
    double magnitudeBC = Math.sqrt(
        vectorBC[0] * vectorBC[0] + vectorBC[1] * vectorBC[1]);

    double cosTheta = dotProduct / (magnitudeAB * magnitudeBC);
    double angleRad = Math.acos(cosTheta);
    double crossProduct = vectorAB[0] * vectorBC[1] - vectorAB[1] * vectorBC[0];

    return Math.signum(crossProduct) * angleRad;
  }

  // public static double getDistanceBetween(double[] pos1, double[] pos2) {
  // }
}
