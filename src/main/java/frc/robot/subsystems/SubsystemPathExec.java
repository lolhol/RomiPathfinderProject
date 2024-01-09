package frc.robot.subsystems;

import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.map.util.CartographerOut;
import frc.robot.util.FinderThread;
import frc.robot.util.MathUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import star.finder.util.Node;

public class SubsystemPathExec extends SubsystemBase {

  public boolean turnedOn = false;

  private int mapRenderTick = 0;

  private int ticksUntilSend = 0;

  private int finderRunCount = 50;
  private final int finalMapSize = 250;
  private FinderThread finderRunThread = null;
  private final RomiDrivetrain rominator;

  private float curRotation = 0;

  public int[] endPos = new int[] { 148, 98 };

  private int initCount = 0;

  private List<Node> path = new ArrayList<>();
  private Node curSQGoing = null;

  private GenericEntry widgetRobot;
  private GenericEntry widgetTarget;

  private boolean renderedNewMap = false;

  private boolean[] sentQueue = new boolean[] { false, false, false, false };

  byte[] cutMap;

  boolean isAdded = false;

  // 0 = right, 1 = left, 2 = forward, 3 = backward

  public SubsystemPathExec(
      boolean isOn,
      RomiDrivetrain rominator) {
    this.rominator = rominator;
    this.turnedOn = isOn;

    ShuffleboardTab tab = Shuffleboard.getTab("SLAM");
    widgetRobot = tab.add("Robot",
        "").withWidget(BuiltInWidgets.kTextView).getEntry();
    widgetTarget = tab.add("Target",
        "").withWidget(BuiltInWidgets.kTextView).getEntry();
  }

  public void tick(CartographerOut output, CvSource cv) {
    if (output == null || output.map.length == 0 || initCount <= 400) {
      initCount++;
      return;
    }

    if (!turnedOn)
      return;

    mapRenderTick(cv, output);
    float[] curPosData = output.functions.GetGlobalData();

    if (finderRunCount >= 50 && renderedNewMap && cutMap != null) {
      finderRunCount = 0;

      if (finderRunThread != null) {
        List<Node> res = finderRunThread.getRes();

        if (res != null) {
          if (res.isEmpty()) {
            System.out.println("NO PATH FOUND!");
          }

          path = res;
          curSQGoing = null;
          finderRunThread = null;
        }
      } else {
        int[] curPos = output.convertPosition(output.FromPosToMap(output.functions.GetGlobalData()),
            (int) output.mapSizeX,
            (int) output.mapSizeY, finalMapSize, finalMapSize);

        if (!isAdded) {
          isAdded = true;
          endPos = new int[] { curPos[0] - 20, curPos[1] };
        }

        // System.out.println(endPos[0] + " | " + endPos[1]);

        finderRunThread = new FinderThread(
            endPos,
            curPos,
            cutMap, finalMapSize, finalMapSize);
        finderRunThread.start();

        System.out.println("STARTING FINDER!");
      }

      renderedNewMap = false;
    } else {
      finderRunCount++;
    }

    if (!path.isEmpty()) {
      if (curSQGoing == null) {
        curSQGoing = path.get(path.size() - 1);
      }

      if (output.distanceFromGlobalToMap(curPosData, new int[] { curSQGoing.x, curSQGoing.y }) < 0.1) {
        curSQGoing = null;
        rominator.forwardBackward(0);
        // Reached node
      } else {
        double angleDiff = getAngle(output.MapXYtoGlobal(new int[] { curSQGoing.x, curSQGoing.y }), curPosData);

        widgetRobot.setString(
            "a=" +
                Math.round(
                    MathUtil.wrap360(curPosData[2])));
        widgetTarget.setString(
            "a=" +
                Math.round(angleDiff));

        // System.out.println(angleDiff + "!!!!");

        if (Math.abs(angleDiff) > 5) {
          if (sentQueue[2]) {
            rominator.forwardBackward(0);
            sentQueue[2] = false;
            return;
          }

          if (angleDiff > 0) {
            if (!sentQueue[1] || ticksUntilSend >= 50) {
              rominator.turn(false, 33);
              sentQueue[1] = true;

              if (sentQueue[0]) {
                sentQueue[0] = false;
              }

              ticksUntilSend = 0;
            }
          } else {
            if (!sentQueue[0] || ticksUntilSend >= 50) {
              rominator.turn(true, 33);
              sentQueue[0] = true;

              if (sentQueue[1]) {
                sentQueue[1] = false;
              }

              ticksUntilSend = 0;
            }
          }
        } else {
          if (sentQueue[0] || sentQueue[1]) {
            rominator.turn(false, 0);
            sentQueue[0] = false;
            sentQueue[1] = false;
          } else {
            if (!sentQueue[2] || ticksUntilSend >= 50) {
              rominator.forwardBackward(40);
              sentQueue[2] = true;
              ticksUntilSend = 0;
            }
          }
        }

        ticksUntilSend++;
      }
    }
  }

  private boolean isPathfinderPathValid() {
    if (path.isEmpty())
      return false;

    for (Node i : path) {
      Node[] nodesAround = i.getNodesAround(3, 3);
      int nonAirCount = 0;
      for (Node j : nodesAround) {
        if (j.x < 0 || j.y < 0 || j.x >= finalMapSize || j.y >= finalMapSize)
          continue;
        if (cutMap[j.y * finalMapSize * j.x] == 255) {
          nonAirCount++;
        }
      }

      if (nonAirCount > 3) {
        return false;
      }
    }

    return true;
  }

  private double getAngle(double[] mCurSQGoing, float[] curPos) {

    double dy = curPos[1] - mCurSQGoing[1]; // 7228-7450
    double dx = mCurSQGoing[0] - curPos[0]; // 5000-5313
    double angle = MathUtil.normaliseDeg(
        Math.atan2(dy, dx) / Math.PI * 180);

    // System.out.println(curPos[2]);
    double angleDeg = MathUtil.wrap360((curPos[2] / Math.PI * 180));
    // System.out.println(angleDeg + " !!!!!!!!");

    return MathUtil.diffDeg(angleDeg, angle);
  }

  public static double calculateTurn(float[] currentPosition, double[] targetNode, double currentAngle) {
    double targetAngle = Math
        .toDegrees(Math.atan2(targetNode[1] - currentPosition[1], targetNode[0] - currentPosition[0]));
    targetAngle = (targetAngle + 360) % 360;
    double angleDifference = (targetAngle - currentAngle + 360) % 360;
    double optimalAngle = (angleDifference <= 180) ? angleDifference : angleDifference - 360;

    return optimalAngle;
  }

  private List<Node> removeEverythingUntil(List<Node> original) {
    if (!original.isEmpty()) {
      List<Node> retList = new ArrayList<>();
      for (int i = 0; i < original.size(); i += 5) {
        retList.add(retList.get(i));
      }

      if (!retList
          .get(retList.size() - 1)
          .equals(original.get(original.size() - 1))) {
        retList.add(original.get(original.size() - 1));
      }

      return retList;
    }

    return new ArrayList<>();
  }

  private void mapRenderTick(CvSource cv, CartographerOut output) {
    if (output.map.length > 0) {
      if (mapRenderTick >= 25) {
        mapRenderTick = 0;

        Mat frame = new Mat(finalMapSize, finalMapSize, CvType.CV_8UC1);

        byte[] newMap = resizeMap(output.map, (int) output.mapSizeX, (int) output.mapSizeY, finalMapSize, finalMapSize);
        cutMap = newMap.clone();

        newMap = new byte[250 * 250];
        Arrays.fill(newMap, (byte) 255);

        for (Node i : path) {
          newMap[(int) (i.y * finalMapSize + i.x)] = 0;
        }

        int[] curPos = output.convertPosition(output.FromPosToMap(output.functions.GetGlobalData()),
            (int) output.mapSizeX, (int) output.mapSizeY, finalMapSize, finalMapSize);

        int[] pointInDirection = getPointInDirection(curPos, output.functions.GetGlobalData()[2], 5.0);
        newMap[curPos[1] * finalMapSize + curPos[0]] = 0;
        newMap[curPos[1] * finalMapSize + curPos[0] + 1] = 0;
        newMap[curPos[1] * finalMapSize + curPos[0] + 2] = 0;
        newMap[(curPos[1] + 1) * finalMapSize + curPos[0]] = 0;
        newMap[(curPos[1] + 1) * finalMapSize + curPos[0] + 1] = 0;

        newMap[endPos[1] * finalMapSize + endPos[0]] = 0;
        newMap[endPos[1] * finalMapSize + endPos[0] + 1] = 0;
        newMap[(endPos[1] + 1) * finalMapSize + endPos[0]] = 0;
        newMap[(endPos[1] + 1) * finalMapSize + endPos[0] + 1] = 0;

        newMap[(pointInDirection[1]) * finalMapSize + pointInDirection[0]] = 0;

        // System.out.println("MAP UPDATE!");

        frame.put(0, 0, newMap);
        cv.putFrame(frame);

        renderedNewMap = true;
      } else {
        mapRenderTick++;
      }
    }
  }

  private int[] getPointInDirection(int[] coordinates, double angle, double dist) {
    double deg = MathUtil.wrap360Rad(angle);
    double newAngle = Math.toRadians(deg);
    int x = coordinates[0];
    int y = coordinates[1];
    int newX = x + (int) (dist * Math.cos(newAngle));
    int newY = y + (int) (dist * Math.sin(newAngle));
    return new int[] { newX, newY };
  }

  public byte[] resizeMap(byte[] oldMap, int oldSizeX, int oldSizeY, int newSizeX, int newSizeY) {
    byte[] newMap = new byte[newSizeX * newSizeY];
    for (int y = 0; y < newSizeY; y++) {
      for (int x = 0; x < newSizeX; x++) {
        int oldX = (int) ((long) x * oldSizeX / newSizeX);
        int oldY = (int) ((long) y * oldSizeY / newSizeY);
        double val = (oldMap[oldY * oldSizeX + oldX] & 0xFF) / 100.;

        int color = (val == 2.55 || val < 0.5) ? 255 : 0;
        /*
         * if (val == 2.55) {
         * color = 127;
         * } else {
         * if (val < 0.5) {
         * color = 255;
         * } else {
         * color = 0;
         * }
         * }
         */

        newMap[y * newSizeX + x] = (byte) color;
      }
    }

    return newMap;
  }
}
