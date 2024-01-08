package frc.robot.subsystems.map.util;

import java.util.function.Function;

public class CartographerOut {
    public long mapSizeX, mapSizeY;
    public double originX, originY, resolution;
    public byte[] map;

    public CartoFunctions functions;

    public CartographerOut(long mapSizeX, long mapSizeY, double originX, double originY, double resolution,
            byte[] map, CartoFunctions functions) {
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

    public double mapXToGlobalX(int x) {
        return (x * resolution + originX);
    }

    public double mapYToGlobalY(int y) {
        return (y * resolution + originY);
    }

    public double[] MapXYtoGlobal(int[] mapPos) {
        return new double[] { mapXToGlobalX(mapPos[0]), mapYToGlobalY(mapPos[1]) };
    }

    public double distanceFromGlobalToMap(float[] globalXY, int[] posMap) {
        double mPosX = mapXToGlobalX(posMap[0]);
        double mPosY = mapYToGlobalY(posMap[1]);

        return Math.sqrt((globalXY[0] - mPosX) * (globalXY[0] - mPosX) + (globalXY[1] - mPosY) * (globalXY[1] - mPosY));
    }

    public interface CartoFunctions {
        float[] GetGlobalData();
    }
}
