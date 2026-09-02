package us.ihmc.scs2.session.log.heightScan;

import java.util.List;

/**
 * Plain decoded snapshot of one {@code perception_msgs/HeightScanMessage}, as written by
 * {@code HeightScanMcapLogger} to {@code heightScan.mcap}. See {@link HeightScanMessageDecoder} for the CDR
 * field-order this is decoded from - that order (and this class' fields) must stay in sync with the logger's
 * {@code HeightScanMcapLogger.SCHEMA} string.
 */
public class HeightScanData
{
   private final long sequenceId;
   private final long controllerTimestamp;
   private final String frameId;

   private final double positionX;
   private final double positionY;
   private final double positionZ;
   private final double orientationX;
   private final double orientationY;
   private final double orientationZ;
   private final double orientationW;

   private final int columnCount;
   private final double cellSizeX;
   private final double cellSizeY;
   private final int rowStride;
   private final int cellStride;

   private final List<PackedElementFieldData> fields;
   private final byte[] data;

   public HeightScanData(long sequenceId,
                         long controllerTimestamp,
                         String frameId,
                         double positionX,
                         double positionY,
                         double positionZ,
                         double orientationX,
                         double orientationY,
                         double orientationZ,
                         double orientationW,
                         int columnCount,
                         double cellSizeX,
                         double cellSizeY,
                         int rowStride,
                         int cellStride,
                         List<PackedElementFieldData> fields,
                         byte[] data)
   {
      this.sequenceId = sequenceId;
      this.controllerTimestamp = controllerTimestamp;
      this.frameId = frameId;
      this.positionX = positionX;
      this.positionY = positionY;
      this.positionZ = positionZ;
      this.orientationX = orientationX;
      this.orientationY = orientationY;
      this.orientationZ = orientationZ;
      this.orientationW = orientationW;
      this.columnCount = columnCount;
      this.cellSizeX = cellSizeX;
      this.cellSizeY = cellSizeY;
      this.rowStride = rowStride;
      this.cellStride = cellStride;
      this.fields = fields;
      this.data = data;
   }

   /**
    * Byte offset, within a cell's {@link #getCellStride()} bytes, of the field with the given name - or
    * {@code null} if this grid does not carry that field (e.g. an older log with a different field set).
    */
   public Integer findFieldOffset(String fieldName)
   {
      for (PackedElementFieldData field : fields)
      {
         if (field.name().equals(fieldName))
            return (int) field.offset();
      }
      return null;
   }

   public int getRowCount()
   {
      return rowStride <= 0 ? 0 : data.length / rowStride;
   }

   public long getSequenceId()
   {
      return sequenceId;
   }

   public long getControllerTimestamp()
   {
      return controllerTimestamp;
   }

   public String getFrameId()
   {
      return frameId;
   }

   public double getPositionX()
   {
      return positionX;
   }

   public double getPositionY()
   {
      return positionY;
   }

   public double getPositionZ()
   {
      return positionZ;
   }

   public double getOrientationX()
   {
      return orientationX;
   }

   public double getOrientationY()
   {
      return orientationY;
   }

   public double getOrientationZ()
   {
      return orientationZ;
   }

   public double getOrientationW()
   {
      return orientationW;
   }

   public int getColumnCount()
   {
      return columnCount;
   }

   public double getCellSizeX()
   {
      return cellSizeX;
   }

   public double getCellSizeY()
   {
      return cellSizeY;
   }

   public int getRowStride()
   {
      return rowStride;
   }

   public int getCellStride()
   {
      return cellStride;
   }

   public List<PackedElementFieldData> getFields()
   {
      return fields;
   }

   public byte[] getData()
   {
      return data;
   }

   /** Mirrors {@code perception_msgs/PackedElementField}: describes one packed value within a grid cell. */
   public record PackedElementFieldData(String name, long offset, int type)
   {
   }
}
