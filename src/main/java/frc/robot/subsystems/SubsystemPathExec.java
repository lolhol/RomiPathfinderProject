package frc.robot.subsystems;

import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.path.map.GridBoard;
import frc.robot.util.FinderThread;
import frc.robot.util.MathUtil;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import star.finder.util.Node;

public class SubsystemPathExec extends SubsystemBase {

  public boolean turnedOn = false;

  private int mapRenderTick = 0;

  private int ticksUntilSend = 0;

  private GridBoard curGrid;
  private int finderRunCount = 50;
  private FinderThread finderRunThread = null;
  private final RomiDrivetrain rominator;

  public int[] endPos = new int[] { 100, 150 };

  private int initCount = 0;

  private List<Node> path = new ArrayList<>();
  private Node curSQGoing = null;

  private GenericEntry widgetRobot;
  private GenericEntry widgetTarget;

  private boolean[] sentQueue = new boolean[] { false, false, false, false };

  // 0 = right, 1 = left, 2 = forward, 3 = backward

  public SubsystemPathExec(
    int mapSizeX,
    int mapSizeY,
    double mapSizeMX,
    double mapSizeMY,
    boolean isOn,
    RomiDrivetrain rominator
  ) {
    this.rominator = rominator;
    curGrid =
      new GridBoard(
        new byte[mapSizeX * mapSizeY],
        mapSizeX,
        mapSizeY,
        mapSizeMX * 1000,
        mapSizeMY * 1000,
        new double[] { mapSizeMX / 2, mapSizeMY / 2 }
      );
    this.turnedOn = isOn;

    ShuffleboardTab tab = Shuffleboard.getTab("SLAM");
    widgetRobot =
      tab.add("Robot", "").withWidget(BuiltInWidgets.kTextView).getEntry();
    widgetTarget =
      tab.add("Target", "").withWidget(BuiltInWidgets.kTextView).getEntry();
  }

  public void tick(
    double[] curPosDataMMDeg,
    byte[] curMap,
    CvSource cv,
    int pxSize
  ) {
    if (initCount <= 400) {
      initCount++;
      return;
    }

    if (!turnedOn) return;

    if (mapRenderTick >= 25) {
      mapRenderTick = 0;
      Mat frame = new Mat(pxSize, pxSize, CvType.CV_8UC1);

      byte[] newMap = curMap.clone();
      for (Node i : path) {
        newMap[(int) (i.y * pxSize + i.x)] = 0;
      }

      newMap[endPos[1] * pxSize + endPos[0]] = 0;

      int[] posMap = curGrid.getPosOnMapList(curPosDataMMDeg);
      newMap[posMap[1] * pxSize + posMap[0]] = 0;

      frame.put(0, 0, newMap);
      cv.putFrame(frame);
    } else {
      mapRenderTick++;
    }

    curGrid.overWriteMap(curMap);
    curGrid.updatePosition(curPosDataMMDeg);

    if (finderRunCount >= 50) {
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
        finderRunThread =
          new FinderThread(
            endPos,
            curGrid.getPosOnMapList(curPosDataMMDeg),
            curGrid.getMap(),
            curGrid.getMapSizeX()
          );
        finderRunThread.start();
      }
    } else {
      finderRunCount++;
    }

    if (!path.isEmpty()) {
      if (curSQGoing == null) {
        curSQGoing = path.get(path.size() - 1);
      }

      if (
        curGrid.getDistanceBetweenPosAndLoc(
          new int[] { curSQGoing.x, curSQGoing.y }
        ) <
        10
      ) {
        curSQGoing = null;

        rominator.forwardBackward(0);
      } else {
        double[] posMMNode = curGrid.convertToMM(
          new int[] { curSQGoing.x, curSQGoing.y }
        );

        double dy = curPosDataMMDeg[1] - posMMNode[1]; // 7228-7450
        double dx = posMMNode[0] - curPosDataMMDeg[0]; // 5000-5313
        double angle = MathUtil.normaliseDeg(
          Math.atan2(dy, dx) / Math.PI * 180
        );

        double angleDiff = MathUtil.diffDeg(curPosDataMMDeg[2], angle);

        widgetRobot.setString(
          "a=" +
          Math.round(curPosDataMMDeg[2]) +
          " x=" +
          Math.round(curPosDataMMDeg[0]) +
          " y=" +
          Math.round(curPosDataMMDeg[1])
        );
        widgetTarget.setString(
          "a=" +
          Math.round(angle) +
          " x=" +
          Math.round(posMMNode[0]) +
          " y=" +
          Math.round(posMMNode[1]) +
          " angleDiff=" +
          Math.round(angleDiff)
        );

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
            rominator.turn(true, 0);
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

  private List<Node> removeEverythingUntil(List<Node> original) {
    if (!original.isEmpty()) {
      List<Node> retList = new ArrayList<>();
      for (int i = 0; i < original.size(); i += 5) {
        retList.add(retList.get(i));
      }

      if (
        !retList
          .get(retList.size() - 1)
          .equals(original.get(original.size() - 1))
      ) {
        retList.add(original.get(original.size() - 1));
      }

      return retList;
    }

    return new ArrayList<>();
  }
}
