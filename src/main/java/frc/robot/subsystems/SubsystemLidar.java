package frc.robot.subsystems;

import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ev3dev.sensors.slamtec.RPLidarA1;
import ev3dev.sensors.slamtec.RPLidarProviderListener;
import ev3dev.sensors.slamtec.model.Scan;
import ev3dev.sensors.slamtec.model.ScanDistance;
import frc.robot.subsystems.path.map.SLAM;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class SubsystemLidar extends SubsystemBase {

  private final RPLidarA1 LIDAR;
  private final int SCAN_SIZE = 426;
  private final int DETECTION_ANGLE_DEG = 360;

  private boolean isScanning = false;
  private long startTime = 0;

  public final SLAM slam;

  public SubsystemLidar() {
    startTime = System.currentTimeMillis();
    final String USBPort = "/dev/tty.usbserial-0001";
    this.LIDAR = new RPLidarA1(USBPort);
    System.out.println("DONE INIT!");
    slam = new SLAM(SCAN_SIZE, DETECTION_ANGLE_DEG);

    try {
      LIDAR.init();

      LIDAR.addListener(
        new RPLidarProviderListener() {
          @Override
          public void scanFinished(Scan scan) {
            if (scan.getDistances().size() < 360) {
              return;
            }

            List<ScanDistance> distances = new ArrayList<>(scan.getDistances());
            distances.sort(Comparator.comparing(ScanDistance::getAngle));

            int[] scanInt = new int[SCAN_SIZE];

            for (int i = 0; i < scanInt.length; i++) {
              final double angle = (double) i /
              scanInt.length *
              DETECTION_ANGLE_DEG;
              final ScanDistance closest = findClosestScanDistance(
                distances,
                angle
              );
              scanInt[i] = (int) Math.ceil(closest.getDistance() * 10);
            }

            slam.updateSlam(scanInt);
          }
        }
      );

      LIDAR.scan();
      isScanning = true;
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public int getSize() {
    return slam.getSize();
  }

  public double getSizeM() {
    return slam.getSizeM();
  }

  public byte[] getMap() {
    return slam.getMap();
  }

  public double[] getPositionData() {
    return slam.getPosition();
  }

  /*public void tick(CvSource cv) {
    if (System.currentTimeMillis() - startTime >= 1000) {
      byte[] map = slam.getMap();
      Mat frame = new Mat(slam.getSize(), slam.getSize(), CvType.CV_8UC1);
      frame.put(0, 0, map);
      cv.putFrame(frame);
      startTime = System.currentTimeMillis();
    }
  }*/

  private ScanDistance findClosestScanDistance(
    List<ScanDistance> distances,
    double angle
  ) {
    int left = 0;
    int right = distances.size() - 1;

    while (left <= right) {
      int mid = left + (right - left) / 2;
      ScanDistance midDistance = distances.get(mid);

      if (midDistance.getAngle() == angle) {
        return midDistance;
      } else if (midDistance.getAngle() < angle) {
        left = mid + 1;
      } else {
        right = mid - 1;
      }
    }

    if (right < 0) {
      return distances.get(0);
    } else if (left >= distances.size()) {
      return distances.get(distances.size() - 1);
    } else {
      ScanDistance before = distances.get(right);
      ScanDistance after = distances.get(left);

      if (
        Math.abs(before.getAngle() - angle) < Math.abs(after.getAngle() - angle)
      ) {
        return before;
      } else {
        return after;
      }
    }
  }
}
