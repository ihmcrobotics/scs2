package us.ihmc.scs2.session.log.heightMap;

import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;
import us.ihmc.scs2.session.mcap.specs.records.Message;

import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * Hand-written CDR decoder for {@code perception_msgs/HeightScanMessage}, matching the field order of
 * {@code PerceptionMcapLogger.HEIGHT_SCAN_SCHEMA} (and the underlying {@code .msg} sources it was copied from) exactly:
 * sequence_id, controllerTimestamp, frame_id, pose (position, orientation), column_count, cell_size, row_stride,
 * cell_stride, fields[], data[]. A future full/global height map source with the same packed-grid schema shape
 * could reuse this decoder as-is; a genuinely different wire schema (e.g. a voxel map) would need its own decoder.
 * <p>
 * A generic schema-driven decode (as used elsewhere for MCAP topics, see {@code MCAPFrameTransformManager}) isn't
 * used here: the message type is fixed and known at compile time by both the writer and this reader, so decoding
 * directly against that known field order is simpler and avoids the generic decoder's "one YoVariable per scalar
 * leaf field" behavior, which is not applicable to a bulk {@code uint8[] data} grid.
 */
public class HeightMapMessageDecoder
{
   public static HeightMapData decode(Message message)
   {
      CDRDeserializer cdr = new CDRDeserializer();
      cdr.initialize(message.messageBuffer(), 0, message.dataLength());

      try
      {
         long sequenceId = cdr.read_uint64();
         long controllerTimestamp = cdr.read_int64();
         String frameId = cdr.read_string();

         double positionX = cdr.read_float64();
         double positionY = cdr.read_float64();
         double positionZ = cdr.read_float64();
         double orientationX = cdr.read_float64();
         double orientationY = cdr.read_float64();
         double orientationZ = cdr.read_float64();
         double orientationW = cdr.read_float64();

         int columnCount = (int) cdr.read_uint32();
         double cellSizeX = cdr.read_float64();
         double cellSizeY = cdr.read_float64();
         int rowStride = (int) cdr.read_uint32();
         int cellStride = (int) cdr.read_uint32();

         List<HeightMapData.PackedElementFieldData> fields = new ArrayList<>();
         cdr.read_sequence((elementIndex, elementCdr) ->
         {
            String name = elementCdr.read_string();
            long offset = elementCdr.read_uint32();
            int type = elementCdr.read_uint8();
            fields.add(new HeightMapData.PackedElementFieldData(name, offset, type));
         });

         ByteArrayOutputStream dataOut = new ByteArrayOutputStream();
         cdr.read_sequence((elementIndex, elementCdr) -> dataOut.write(elementCdr.read_uint8()));

         return new HeightMapData(sequenceId,
                                   controllerTimestamp,
                                   frameId,
                                   positionX,
                                   positionY,
                                   positionZ,
                                   orientationX,
                                   orientationY,
                                   orientationZ,
                                   orientationW,
                                   columnCount,
                                   cellSizeX,
                                   cellSizeY,
                                   rowStride,
                                   cellStride,
                                   fields,
                                   dataOut.toByteArray());
      }
      finally
      {
         cdr.finalize(true);
      }
   }
}
