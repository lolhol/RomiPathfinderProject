package frc.robot.util;

import java.util.List;
import star.finder.main.AStar;
import star.finder.util.Node;

public class FinderThread extends Thread {

  private final AStar star;
  private List<Node> res = null;
  private final int[] start;
  private final int[] end;
  private final byte[] board;
  private final int xSize, ySize;

  public FinderThread(int[] endPos, int[] startPos, byte[] board, int xSize, int ySize) {
    this.star = new AStar();
    this.end = endPos;
    this.start = startPos;
    this.board = board;
    this.xSize = xSize;
    this.ySize = ySize;
  }

  @Override
  public void run() {
    this.res = star.run(start, end, board, xSize, xSize);
  }

  public List<Node> getRes() {
    return this.res;
  }
}
