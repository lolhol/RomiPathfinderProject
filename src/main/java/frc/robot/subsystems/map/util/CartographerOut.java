package frc.robot.subsystems.map.util;

import java.util.function.Function;

public class CartographerOut {
    public long mapSizeX, mapSizeY;
    public double originX, originY, resolution;
    public byte[] map;

    public CartoFunctions functions;

    public CartographerOut(long mapSizeX, long mapSizeY, double originX, double originY, double resolution,
            byte[] map, CartoFunctions functions) {
        this.mapSizeX = mapSizeX;
        this.mapSizeY = mapSizeY;
        this.originX = originX;
        this.originY = originY;
        this.resolution = resolution;
        this.map = map;
        this.functions = functions;
    }

    public int fromXToMapX(double x) {
        return (int) ((x - originX) / resolution);
    }

    public int fromYToMapY(double y) {
        return (int) ((y - originY) / resolution);
    }

    public int[] FromPosToMap(float[] curPos) {
        return new int[] { fromXToMapX(curPos[0]), fromXToMapX(curPos[1]) };
    }

    public int[] FromPosToMapWithNewMap(float[] curPos, int newMapSizeX, int newMapSizeY, int oldMapX, int oldMapY) {
        int[] curUnmodifiedPos = FromPosToMap(curPos);
        double changeX = oldMapX / newMapSizeX;
        double changeY = oldMapY / newMapSizeY;
        return new int[] { (int) (curUnmodifiedPos[0] * changeX), (int) (curUnmodifiedPos[1] * changeY) };
    }

    public int[] convertPosition(int[] position, int oldMapSizeX, int oldMapSizeY, int newMapSizeX, int newMapSizeY) {
        int oldX = position[0];
        int oldY = position[1];

        double xRatio = (double) newMapSizeX / oldMapSizeX;
        double yRatio = (double) newMapSizeY / oldMapSizeY;

        int newX = (int) Math.round(oldX * xRatio);
        int newY = (int) Math.round(oldY * yRatio);

        return new int[] { newX, newY };
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
