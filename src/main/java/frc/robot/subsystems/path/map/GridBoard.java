package frc.robot.subsystems.path.map;

import edu.wpi.first.cscore.CvSource;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import star.finder.util.Node;

public class GridBoard {

  private byte[] data;
  private final int mapSizeX;
  private final int mapSizeY;
  private final double mapSizeMMX;
  private final double mapSizeMMY;

  private double[] position;
  private double rotation;

  private final double sqSizeMMX;
  private final double sqSizeMMY;

  private List<Node> markedCoords = new ArrayList<>();

  // List #1 = X
  // List #2 = Y
  public GridBoard(
    byte[] initData,
    int mapSizeX,
    int mapSizeY,
    double mapSizeMMX,
    double mapSizeMMY,
    double[] initPos
  ) {
    this.data = initData;
    this.mapSizeX = mapSizeX;
    this.mapSizeY = mapSizeY;
    this.mapSizeMMX = mapSizeMMX;
    this.mapSizeMMY = mapSizeMMY;
    this.position = initPos;
    this.sqSizeMMX = mapSizeMMX / mapSizeX;
    this.sqSizeMMY = mapSizeMMY / mapSizeY;
  }

  public byte getData(int x, int y) {
    return data[y * mapSizeY + x];
  }

  public int getDataInt(int x, int y) {
    return (int) getData(x, y) & 0xFF;
  }

  public void updatePosition(double[] newPos) {
    this.position[0] = newPos[0];
    this.position[1] = newPos[1];
    this.rotation = newPos[2];
  }

  public int[] getPosOnMapList(double[] curPosMM) {
    return new int[] {
      (int) (curPosMM[0] / sqSizeMMX),
      (int) (curPosMM[1] / sqSizeMMY),
    };
  }

  public byte[] getMap() {
    return this.data;
  }

  public void overWriteMap(byte[] newMap) {
    this.data = newMap;
  }

  public double[] getCurPosData() {
    return new double[] { this.position[0], this.position[1], this.rotation };
  }

  /*public void overWriteMap(byte[] mapBytes, CvSource cv) {
    byte[] newArr = new byte[mapBytes.length];
    Mat frame = new Mat(mapSizeX, mapSizeX, CvType.CV_8UC1);

    for (int y = 0; y < this.mapSizeY; y++) {
      for (int x = 0; x < this.mapSizeX; x++) {
        byte defaultColor = mapBytes[y * this.mapSizeX + x];
        newArr[y * this.mapSizeX + x + 0] = defaultColor;
        //newArr[y * this.mapSizeX + x + 1] = defaultColor;
        //newArr[y * this.mapSizeX + x + 2] = defaultColor;
        //this.data.get(x).set(y, (int) defaultColor);
      }
    }

    int[] curPos = getPosOnMapList(position);

    for (Node i : markedCoords) {
      newArr[(int) (i.y * this.mapSizeX + i.x) + 0] = 0;
      newArr[(int) (i.y * this.mapSizeX + i.x) + 1] = (byte) 255;
      newArr[(int) (i.y * this.mapSizeX + i.x) + 2] = 0;
    }

    newArr[(int) (curPos[1] * this.mapSizeX + curPos[0]) + 0] = 0;
    newArr[(int) (curPos[1] * this.mapSizeX + curPos[0]) + 1] = 0;
    newArr[(int) (curPos[1] * this.mapSizeX + curPos[0]) + 2] = (byte) 255;

    frame.put(mapSizeY, mapSizeX, newArr);
    cv.putFrame(frame);
  }*/

  public void putRGBPathAndLoc(List<Node> path, double[] curPosDataM) {
    markedCoords.clear();
    markedCoords.addAll(path);
  }

  public int getMapSizeX() {
    return mapSizeX;
  }

  public double getDistanceBetweenMM(int[] pos1, int[] pos2) {
    double[] pos1MM = convertToMM(pos1);
    double[] pos2MM = convertToMM(pos2);

    double dX = pos1MM[0] - pos2MM[0];
    double dY = pos1MM[1] - pos2MM[1];
    return Math.sqrt(dX * dX + dY * dY);
  }

  public double getDistanceBetweenMM(int[] bp, double[] mmPos2) {
    double[] mmPosBP = convertToMM(bp);

    double dX = mmPosBP[0] - mmPos2[0];
    double dY = mmPosBP[1] - mmPos2[1];
    return Math.sqrt(dX * dX + dY * dY);
  }

  public double[] convertToMM(int[] pos) {
    return new double[] { pos[0] * sqSizeMMX, pos[1] * sqSizeMMY };
  }

  public double getDistanceBetweenPosAndLoc(int[] bpPos) {
    double[] mPos = convertToMM(bpPos);
    double dX = mPos[0] - position[0];
    double dY = mPos[1] - position[1];
    return Math.sqrt(dX * dX + dY * dY);
  }
}
