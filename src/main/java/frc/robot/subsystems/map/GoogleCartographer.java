package frc.robot.subsystems.map;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import brigero.cartographer4java.Cartographer4Java;
import frc.robot.subsystems.map.util.CartographerOut;
import frc.robot.subsystems.map.util.DataOutPutFinish;
import frc.robot.subsystems.map.util.CartographerOut.CartoFunctions;

public class GoogleCartographer {
    private boolean isInitiated = false;
    private Cartographer4Java cartographerPort;

    public GoogleCartographer() {
        cartographerPort = new Cartographer4Java();

    }

    public void initiate(String pathToFolder, String mainFile, boolean useImu, boolean useOdom, double lidarHZ) {
        isInitiated = true;
        cartographerPort.init(pathToFolder, mainFile, useImu, useOdom, lidarHZ);
    }

    public CartographerOut getCartographerMapData() {
        byte[] rawMap = cartographerPort.paintMap();

        ByteBuffer byteBuffer = ByteBuffer.wrap(rawMap).order(ByteOrder.nativeOrder());
        final long mapSizeX = byteBuffer.getLong();
        final long mapSizeY = byteBuffer.getLong();
        final double originX = byteBuffer.getDouble();
        final double originY = byteBuffer.getDouble();
        final double resolution = byteBuffer.getDouble();
        final byte[] mapBytes = new byte[byteBuffer.remaining()];
        byteBuffer.get(mapBytes);

        return new CartographerOut(mapSizeX, mapSizeY, originX, originY, resolution, mapBytes, new CartoFunctions() {
            @Override
            public float[] GetGlobalData() {
                return GetRobotPositionData();
            }
        });
    }

    public float getGlobalX() {
        return cartographerPort.posX();
    }

    public float getGlobalY() {
        return cartographerPort.posY();
    }

    public void deleteCartoAndOptimize() {
        cartographerPort.stopAndOptimize();
    }

    public float getCartoAngleRadians() {
        return cartographerPort.angle();
    }

    public void updateLidarData(long timeStampMS /* System.currentTimeMillis() */, float[] cartesianX,
            float[] cartesianY, float[] intensities /* how reflective a surface is */) { // if lidar does not return
                                                                                         // intensities jst make new arr
                                                                                         // filled with 255

        System.out.println("CALLABACK!");
        if (!isInitiated) {
            return;
        }

        cartographerPort.updateLidarData(timeStampMS, cartesianX, cartesianY, intensities);
    }

    public float[] GetGlobalPosXY() {
        return new float[] { getGlobalX(), getGlobalY() };
    }

    public float[] GetRobotPositionData() {
        return new float[] { this.getGlobalX(), this.getGlobalY(), this.getCartoAngleRadians() };
    }

    public DataOutPutFinish getCallback() {
        return new DataOutPutFinish() {
            @Override
            public void lidarScanFinished(float[][] scanCartesianData) {
                updateLidarData(System.currentTimeMillis(), scanCartesianData[0], scanCartesianData[1],
                        scanCartesianData[2]);
            }
        };
    }
}
