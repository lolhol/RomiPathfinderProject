package frc.robot.util;

import java.util.List;
import star.finder.main.AStar;
import star.finder.util.Node;
import star.finder.util.Options;

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
    this.star.setIter(100000);
    this.res = star.run(start, end, board, xSize, ySize, new Options() {

      @Override
      public boolean isAddNode(byte arg0) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isAddNode'");
      }

      @Override
      public boolean isAddNode(int arg0) {
        return arg0 != 255;
      }

    });
  }

  public List<Node> getRes() {
    return this.res;
  }
}
