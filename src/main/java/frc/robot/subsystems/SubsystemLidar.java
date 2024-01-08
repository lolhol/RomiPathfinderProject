package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ev3dev.sensors.slamtec.RPLidarA1;
import ev3dev.sensors.slamtec.RPLidarA1ServiceException;
import ev3dev.sensors.slamtec.RPLidarProviderListener;
import ev3dev.sensors.slamtec.model.Scan;
import ev3dev.sensors.slamtec.model.ScanDistance;
import frc.robot.subsystems.map.util.DataOutPutFinish;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class SubsystemLidar extends SubsystemBase {
  private final RPLidarA1 LIDAR;
  private boolean isScanning = false;

  public SubsystemLidar(DataOutPutFinish callback, boolean isStartScanning, final String USBPort) {
    this.LIDAR = new RPLidarA1(USBPort);

    try {
      LIDAR.init();

      LIDAR.addListener(
          new RPLidarProviderListener() {
            @Override
            public void scanFinished(Scan scan) {
              if (!isScanning || scan.getDistances().size() < 360) {
                return;
              }

              List<ScanDistance> distances = new ArrayList<>(scan.getDistances());
              distances.sort(Comparator.comparing(ScanDistance::getAngle));

              // call the callback of scan finish
              callback.lidarScanFinished(convertToCartesian(distances));
            }
          });

      if (isStartScanning) {
        LIDAR.scan();
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void startScanning() {
    try {
      LIDAR.scan();
    } catch (RPLidarA1ServiceException e) {
      e.printStackTrace();
    }
  }

  public void setScanning(boolean state) {
    this.isScanning = state;
  }

  public boolean isScanning() {
    return isScanning;
  }

  public void closeLidar() {
    try {
      this.LIDAR.close();
    } catch (RPLidarA1ServiceException e) {
    }
  }

  private float[][] convertToCartesian(List<ScanDistance> distances) {
    float[] xCoordinates = new float[distances.size()];
    float[] yCoordinates = new float[distances.size()];

    for (int i = 0; i < distances.size(); i++) {
      double angleRadians = Math.toRadians(distances.get(i).getAngle());
      double dist = distances.get(i).getDistance() / 100;
      if (dist < 16) {
        xCoordinates[i] = (float) (dist * Math.cos(angleRadians));
        yCoordinates[i] = (float) (dist * Math.sin(angleRadians));
      } else {
        xCoordinates[i] = (float) (17 * Math.cos(angleRadians));
        yCoordinates[i] = (float) (17 * Math.sin(angleRadians));
      }
    }

    float[] intensities = new float[distances.size()];
    Arrays.fill(intensities, 255);

    float[][] retList = new float[3][];
    retList[0] = xCoordinates;
    retList[1] = yCoordinates;
    retList[2] = intensities;

    return retList;
  }
}
