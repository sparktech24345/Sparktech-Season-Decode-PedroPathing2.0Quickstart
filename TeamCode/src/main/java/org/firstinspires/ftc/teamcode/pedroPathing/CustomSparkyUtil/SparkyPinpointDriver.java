package org.firstinspires.ftc.teamcode.pedroPathing.CustomSparkyUtil;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;



@I2cDeviceType
@DeviceProperties(
        name = "goBILDA® Pinpoint Odometry Computer",
        xmlTag = "goBILDAPinpoint",
        description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
)
public class SparkyPinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

    private int deviceStatus = 0;
    private int loopTime = 0;
    private int xEncoderValue = 0;
    private int yEncoderValue = 0;
    private float xPosition = 0;
    private float yPosition = 0;
    private float hOrientation = 0;
    private float xVelocity = 0;
    private float yVelocity = 0;
    private float hVelocity = 0;

    private static final float goBILDA_SWINGARM_POD = 13.26291192f;
    private static final float goBILDA_4_BAR_POD = 19.89436789f;

    private static final byte DEFAULT_ADDRESS = 0x31;

    public SparkyPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }


    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.GoBilda;
    }

    @Override
    protected synchronized boolean doInitialize() {
        ((LynxI2cDeviceSynch) (deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        return true;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA® Pinpoint Odometry Computer";
    }
    private enum Register {
        DEVICE_ID(1),
        DEVICE_VERSION(2),
        DEVICE_STATUS(3),
        DEVICE_CONTROL(4),
        LOOP_TIME(5),
        X_ENCODER_VALUE(6),
        Y_ENCODER_VALUE(7),
        X_POSITION(8),
        Y_POSITION(9),
        H_ORIENTATION(10),
        X_VELOCITY(11),
        Y_VELOCITY(12),
        H_VELOCITY(13),
        TICKS_PER_MM(14),
        X_POD_OFFSET(15),
        Y_POD_OFFSET(16),
        YAW_SCALAR(17),
        BULK_READ(18);

        private final int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }
    public enum DeviceStatus {
        NOT_READY(0),
        READY(1),
        CALIBRATING(1 << 1),
        FAULT_X_POD_NOT_DETECTED(1 << 2),
        FAULT_Y_POD_NOT_DETECTED(1 << 3),
        FAULT_NO_PODS_DETECTED(1 << 2 | 1 << 3),
        FAULT_IMU_RUNAWAY(1 << 4),
        FAULT_BAD_READ(1 << 5);

        private final int status;

        DeviceStatus(int status) {
            this.status = status;
        }
    }
    public enum EncoderDirection {
        FORWARD,
        REVERSED
    }

    public enum GoBildaOdometryPods {
        goBILDA_SWINGARM_POD,
        goBILDA_4_BAR_POD
    }

    public enum ReadData {
        ONLY_UPDATE_HEADING,
    }
    private enum DeviceControl {
        RECALIBRATE_IMU(1 << 0),
        RESET_POS_AND_IMU(1 << 1),
        SET_X_ENCODER_REVERSED(1 << 4),
        SET_X_ENCODER_FORWARD(1 << 5),
        SET_Y_ENCODER_REVERSED(1 << 2),
        SET_Y_ENCODER_FORWARD(1 << 3);

        public final int value;

        DeviceControl(int value) {
            this.value = value;
        }
    }

    private void writeInt(final SparkyPinpointDriver.Register reg, int i) {
        deviceClient.write(reg.bVal, TypeConversion.intToByteArray(i, ByteOrder.LITTLE_ENDIAN));
    }

    private int readInt(SparkyPinpointDriver.Register reg) {
        return byteArrayToInt(deviceClient.read(reg.bVal, 4), ByteOrder.LITTLE_ENDIAN);
    }

    private float byteArrayToFloat(byte[] byteArray, ByteOrder byteOrder) {
        return ByteBuffer.wrap(byteArray).order(byteOrder).getFloat();
    }

    private float readFloat(SparkyPinpointDriver.Register reg) {
        return byteArrayToFloat(deviceClient.read(reg.bVal, 4), ByteOrder.LITTLE_ENDIAN);
    }

    private byte[] floatToByteArray(float value, ByteOrder byteOrder) {
        return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array();
    }

    private void writeByteArray(SparkyPinpointDriver.Register reg, byte[] bytes) {
        deviceClient.write(reg.bVal, bytes);
    }

    private void writeFloat(SparkyPinpointDriver.Register reg, float f) {
        byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(f).array();
        deviceClient.write(reg.bVal, bytes);
    }

    private SparkyPinpointDriver.DeviceStatus lookupStatus(int s) {
        if ((s & SparkyPinpointDriver.DeviceStatus.CALIBRATING.status) != 0) {
            return SparkyPinpointDriver.DeviceStatus.CALIBRATING;
        }
        boolean xPodDetected = (s & SparkyPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) == 0;
        boolean yPodDetected = (s & SparkyPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) == 0;

        if (!xPodDetected && !yPodDetected) {
            return SparkyPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED;
        }
        if (!xPodDetected) {
            return SparkyPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED;
        }
        if (!yPodDetected) {
            return SparkyPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
        }
        if ((s & SparkyPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0) {
            return SparkyPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY;
        }
        if ((s & SparkyPinpointDriver.DeviceStatus.READY.status) != 0) {
            return SparkyPinpointDriver.DeviceStatus.READY;
        }
        if ((s & SparkyPinpointDriver.DeviceStatus.FAULT_BAD_READ.status) != 0) {
            return SparkyPinpointDriver.DeviceStatus.FAULT_BAD_READ;
        } else {
            return SparkyPinpointDriver.DeviceStatus.NOT_READY;
        }
    }

    private Float isPositionCorrupt(float oldValue, float newValue, int threshold, boolean bulkUpdate) {
        boolean noData = bulkUpdate && (loopTime < 1);

        boolean isCorrupt = noData || Float.isNaN(newValue) || Math.abs(newValue - oldValue) > threshold;

        if (!isCorrupt) {
            return newValue;
        }

        deviceStatus = SparkyPinpointDriver.DeviceStatus.FAULT_BAD_READ.status;
        return oldValue;
    }

    private Float isVelocityCorrupt(float oldValue, float newValue, int threshold) {
        boolean isCorrupt = Float.isNaN(newValue) || Math.abs(newValue) > threshold;
        boolean noData = (loopTime <= 1);

        if (!isCorrupt) {
            return newValue;
        }

        deviceStatus = SparkyPinpointDriver.DeviceStatus.FAULT_BAD_READ.status;
        return oldValue;
    }

    public void update() {
        final int positionThreshold = 5000;
        final int headingThreshold = 120;
        final int velocityThreshold = 10000;
        final int headingVelocityThreshold = 120;

        float oldPosX = xPosition;
        float oldPosY = yPosition;
        float oldPosH = hOrientation;
        float oldVelX = xVelocity;
        float oldVelY = yVelocity;
        float oldVelH = hVelocity;

        byte[] bArr = deviceClient.read(SparkyPinpointDriver.Register.BULK_READ.bVal, 40);
        deviceStatus = byteArrayToInt(Arrays.copyOfRange(bArr, 0, 4), ByteOrder.LITTLE_ENDIAN);
        loopTime = byteArrayToInt(Arrays.copyOfRange(bArr, 4, 8), ByteOrder.LITTLE_ENDIAN);
        xEncoderValue = byteArrayToInt(Arrays.copyOfRange(bArr, 8, 12), ByteOrder.LITTLE_ENDIAN);
        yEncoderValue = byteArrayToInt(Arrays.copyOfRange(bArr, 12, 16), ByteOrder.LITTLE_ENDIAN);
        xPosition = byteArrayToFloat(Arrays.copyOfRange(bArr, 16, 20), ByteOrder.LITTLE_ENDIAN);
        yPosition = byteArrayToFloat(Arrays.copyOfRange(bArr, 20, 24), ByteOrder.LITTLE_ENDIAN);
        hOrientation = byteArrayToFloat(Arrays.copyOfRange(bArr, 24, 28), ByteOrder.LITTLE_ENDIAN);
        xVelocity = byteArrayToFloat(Arrays.copyOfRange(bArr, 28, 32), ByteOrder.LITTLE_ENDIAN);
        yVelocity = byteArrayToFloat(Arrays.copyOfRange(bArr, 32, 36), ByteOrder.LITTLE_ENDIAN);
        hVelocity = byteArrayToFloat(Arrays.copyOfRange(bArr, 36, 40), ByteOrder.LITTLE_ENDIAN);

        xPosition = isPositionCorrupt(oldPosX, xPosition, positionThreshold, true);
        yPosition = isPositionCorrupt(oldPosY, yPosition, positionThreshold, true);
        hOrientation = isPositionCorrupt(oldPosH, hOrientation, headingThreshold, true);
        xVelocity = isVelocityCorrupt(oldVelX, xVelocity, velocityThreshold);
        yVelocity = isVelocityCorrupt(oldVelY, yVelocity, velocityThreshold);
        hVelocity = isVelocityCorrupt(oldVelH, hVelocity, headingVelocityThreshold);

    }

    public void update(SparkyPinpointDriver.ReadData data) {
        if (data == SparkyPinpointDriver.ReadData.ONLY_UPDATE_HEADING) {
            final int headingThreshold = 120;

            float oldPosH = hOrientation;

            hOrientation = byteArrayToFloat(deviceClient.read(SparkyPinpointDriver.Register.H_ORIENTATION.bVal, 4), ByteOrder.LITTLE_ENDIAN);

            hOrientation = isPositionCorrupt(oldPosH, hOrientation, headingThreshold, false);

            if (deviceStatus == SparkyPinpointDriver.DeviceStatus.FAULT_BAD_READ.status) {
                deviceStatus = SparkyPinpointDriver.DeviceStatus.READY.status;
            }
        }
    }

    public void setOffsets(double xOffset, double yOffset, DistanceUnit distanceUnit) {
        writeFloat(SparkyPinpointDriver.Register.X_POD_OFFSET, (float) distanceUnit.toMm(xOffset));
        writeFloat(SparkyPinpointDriver.Register.Y_POD_OFFSET, (float) distanceUnit.toMm(yOffset));
    }

    public void recalibrateIMU() {
        writeInt(SparkyPinpointDriver.Register.DEVICE_CONTROL, SparkyPinpointDriver.DeviceControl.RECALIBRATE_IMU.value);
    }

    public void resetPosAndIMU() {
        writeInt(SparkyPinpointDriver.Register.DEVICE_CONTROL, SparkyPinpointDriver.DeviceControl.RESET_POS_AND_IMU.value);
    }

    public void setEncoderDirections(SparkyPinpointDriver.EncoderDirection xEncoder, SparkyPinpointDriver.EncoderDirection yEncoder) {
        if (xEncoder == SparkyPinpointDriver.EncoderDirection.FORWARD) {
            writeInt(SparkyPinpointDriver.Register.DEVICE_CONTROL, SparkyPinpointDriver.DeviceControl.SET_X_ENCODER_FORWARD.value);
        }
        if (xEncoder == SparkyPinpointDriver.EncoderDirection.REVERSED) {
            writeInt(SparkyPinpointDriver.Register.DEVICE_CONTROL, SparkyPinpointDriver.DeviceControl.SET_X_ENCODER_REVERSED.value);
        }

        if (yEncoder == SparkyPinpointDriver.EncoderDirection.FORWARD) {
            writeInt(SparkyPinpointDriver.Register.DEVICE_CONTROL, SparkyPinpointDriver.DeviceControl.SET_Y_ENCODER_FORWARD.value);
        }
        if (yEncoder == SparkyPinpointDriver.EncoderDirection.REVERSED) {
            writeInt(SparkyPinpointDriver.Register.DEVICE_CONTROL, SparkyPinpointDriver.DeviceControl.SET_Y_ENCODER_REVERSED.value);
        }
    }

    public void setEncoderResolution(SparkyPinpointDriver.GoBildaOdometryPods pods) {
        if (pods == SparkyPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
            setEncoderResolution(goBILDA_SWINGARM_POD, DistanceUnit.MM);
        }
        if (pods == SparkyPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD) {
            setEncoderResolution(goBILDA_4_BAR_POD, DistanceUnit.MM);
        }
    }

    public void setEncoderResolution(double ticksPerUnit, DistanceUnit distanceUnit) {
        double resolution = distanceUnit.toMm(ticksPerUnit);
        writeByteArray(SparkyPinpointDriver.Register.TICKS_PER_MM, (floatToByteArray((float) resolution, ByteOrder.LITTLE_ENDIAN)));
    }

    public void setYawScalar(double yawScalar) {
        writeByteArray(SparkyPinpointDriver.Register.YAW_SCALAR, (floatToByteArray((float) yawScalar, ByteOrder.LITTLE_ENDIAN)));
    }

    public void setPosition(Pose2D pos) {
        setPosX(- pos.getX(DistanceUnit.MM), DistanceUnit.MM); /// TODO HERE
        setPosY(pos.getY(DistanceUnit.MM), DistanceUnit.MM);
        setHeading(pos.getHeading(AngleUnit.RADIANS), AngleUnit.RADIANS);
    }

    public void setPosX(double posX, DistanceUnit distanceUnit) {
        writeByteArray(SparkyPinpointDriver.Register.X_POSITION, (floatToByteArray((float) distanceUnit.toMm(posX), ByteOrder.LITTLE_ENDIAN)));
    }

    public void setPosY(double posY, DistanceUnit distanceUnit) {
        writeByteArray(SparkyPinpointDriver.Register.Y_POSITION, (floatToByteArray((float) distanceUnit.toMm(posY), ByteOrder.LITTLE_ENDIAN)));
    }

    public void setHeading(double heading, AngleUnit angleUnit) {
        writeByteArray(SparkyPinpointDriver.Register.H_ORIENTATION, (floatToByteArray((float) angleUnit.toRadians(heading), ByteOrder.LITTLE_ENDIAN)));
    }

    public int getDeviceID() {
        return readInt(SparkyPinpointDriver.Register.DEVICE_ID);
    }

    public int getDeviceVersion() {
        return readInt(SparkyPinpointDriver.Register.DEVICE_VERSION);
    }

    public float getYawScalar() {
        return readFloat(SparkyPinpointDriver.Register.YAW_SCALAR);
    }

    public SparkyPinpointDriver.DeviceStatus getDeviceStatus() {
        return lookupStatus(deviceStatus);
    }

    public int getLoopTime() {
        return loopTime;
    }

    public double getFrequency() {
        if (loopTime != 0) {
            return 1000000.0 / loopTime;
        } else {
            return 0;
        }
    }

    public int getEncoderX() {
        return - xEncoderValue; /// TODO HERE
    }
    public int getEncoderY() {
        return yEncoderValue;
    }

    public double getPosX(DistanceUnit distanceUnit) {
        return - distanceUnit.fromMm(xPosition); /// TODO HERE
    }

    public double getPosY(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(yPosition);
    }

    public double getHeading(AngleUnit angleUnit) {
        return angleUnit.fromRadians(hOrientation);
    }

    public double getHeading(UnnormalizedAngleUnit unnormalizedAngleUnit) {
        return unnormalizedAngleUnit.fromRadians(hOrientation);
    }

    public double getVelX(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(-xVelocity); /// TODO HERE
    }

    public double getVelY(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(yVelocity);
    }

    public double getHeadingVelocity(UnnormalizedAngleUnit unnormalizedAngleUnit) {
        return unnormalizedAngleUnit.fromRadians(hVelocity);
    }

    public float getXOffset(DistanceUnit distanceUnit) {
        return (float) - distanceUnit.fromMm(readFloat(SparkyPinpointDriver.Register.X_POD_OFFSET));
    }

    public float getYOffset(DistanceUnit distanceUnit) {
        return (float) distanceUnit.fromMm(readFloat(SparkyPinpointDriver.Register.Y_POD_OFFSET));
    }

    public Pose2D getPosition() {
        return new Pose2D(DistanceUnit.MM,
                -xPosition,
                yPosition,
                AngleUnit.RADIANS,
                AngleUnit.normalizeRadians(hOrientation));
    }
}