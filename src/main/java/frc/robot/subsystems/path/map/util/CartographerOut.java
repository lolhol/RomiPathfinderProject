package frc.robot.subsystems.path.map.util;

public class CartographerOut {
    public long mapSizeX, mapSizeY;
    public double originX, originY, resolution;
    public byte[] map;

    public CartographerOut(long mapSizeX, long mapSizeY, double originX, double originY, double resolution,
            byte[] map) {
    }

    public int fromXToMapX(double x) {
        return (int) ((x - originX) / resolution);
    }

    public int fromYToMapY(double y) {
        return (int) ((y - originY) / resolution);
    }

    public int[] fromPosToMap(double[] curPos) {
        return new int[] { fromXToMapX(curPos[0]), fromXToMapX(curPos[1]) };
    }
}
