package frc.robot.subsystems.path.map;

import edu.wlu.cs.levy.breezyslam.algorithms.DeterministicSLAM;
import edu.wlu.cs.levy.breezyslam.algorithms.RMHCSLAM;
import edu.wlu.cs.levy.breezyslam.algorithms.SinglePositionSLAM;
import edu.wlu.cs.levy.breezyslam.components.Laser;
import edu.wlu.cs.levy.breezyslam.components.Position;
import frc.robot.util.MathUtil;

public class SLAM {

  private static final int MAP_SIZE_PIXELS = 200;
  private static final double MAP_SIZE_METERS = 10; // 32;
  private static final double HOLE_WIDTH_MM = 200; // 32;
  private static final int MAP_QUALITY = 50; // 0-255; default 50
  private static SinglePositionSLAM slam;

  public SLAM(int scanSize, int detectionAnglDeg) {
    RPLidarLaser laser = new RPLidarLaser(scanSize, detectionAnglDeg);
    slam = new RMHCSLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, 9999);
    slam.hole_width_mm = HOLE_WIDTH_MM;
    slam.map_quality = MAP_QUALITY;
  }

  public double[] getPosition() {
    Position pos = slam.getpos();
    return new double[] {
      pos.x_mm,
      pos.y_mm,
      MathUtil.normaliseDeg(180 - pos.theta_degrees),
    };
  }

  public void updateSlam(int[] scan) {
    slam.update(scan);
  }

  public byte[] getMap() {
    byte[] mapBytes = new byte[MAP_SIZE_PIXELS * MAP_SIZE_PIXELS];
    slam.getmap(mapBytes);
    return mapBytes;
  }

  public int getSize() {
    return SLAM.MAP_SIZE_PIXELS;
  }

  public double getSizeM() {
    return SLAM.MAP_SIZE_METERS;
  }

  private class RPLidarLaser extends Laser {

    public RPLidarLaser(int scanSize, int detectionAngleDegrees) {
      // int scan_size,
      // double scan_rate_hz,
      // double detection_angle_degrees,
      // double distance_no_detection_mm,
      // int detection_margin,
      // double offset_mm)
      // super(682, 10, 240, 4000, 70, 145);

      //////
      // super(426, 10, 360, 12000, 5, 145)
      /////
      super(scanSize, 10, detectionAngleDegrees, 12000, 0, 0);
    }
  }
}
