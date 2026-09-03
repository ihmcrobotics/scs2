package us.ihmc.scs2.session.log.heightMap;

import perception_msgs.HeightScanMessage;
import perception_msgs.PackedElementField;
import us.ihmc.fastddsjava.cdr.idl.IDLByteSequence;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;
import us.ihmc.scs2.session.mcap.specs.records.Message;

import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * Decodes {@code perception_msgs/HeightScanMessage} into {@link HeightMapData}, from either of two sources:
 * <ul>
 * <li>{@link #decode(Message)} - raw CDR bytes read back from a logged {@code perception.mcap} chunk, matching the
 * field order of {@code PerceptionMcapLogger.HEIGHT_SCAN_SCHEMA} exactly: sequence_id, controllerTimestamp,
 * frame_id, pose (position, orientation), column_count, cell_size, row_stride, cell_stride, fields[], data[].
 * <li>{@link #decode(HeightScanMessage)} - an already-deserialized message received live over a ROS2 subscription
 * (see {@code HeightMapRos2LiveFeed}). No CDR parsing needed here: jros2 hands back a real object, so this is a
 * plain getter-by-getter mapping - simpler than the mcap path, and kept in sync with it only by both ultimately
 * describing the same wire message.
 * </ul>
 * A future full/global height map source with the same packed-grid schema shape could reuse either decoder as-is;
 * a genuinely different wire schema (e.g. a voxel map) would need its own decoder(s).
 * <p>
 * {@link #decode(Message)} doesn't use a generic schema-driven decode (as used elsewhere for MCAP topics, see
 * {@code MCAPFrameTransformManager}): the message type is fixed and known at compile time by both the writer and
 * this reader, so decoding directly against that known field order is simpler and avoids the generic decoder's
 * "one YoVariable per scalar leaf field" behavior, which is not applicable to a bulk {@code uint8[] data} grid.
 */
public class HeightMapMessageDecoder
{
   public static HeightMapData decode(HeightScanMessage message)
   {
      List<HeightMapData.PackedElementFieldData> fields = new ArrayList<>();
      IDLObjectSequence<PackedElementField> fieldSequence = message.getFields();
      for (int i = 0; i < fieldSequence.size(); i++)
      {
         PackedElementField field = fieldSequence.get(i);
         fields.add(new HeightMapData.PackedElementFieldData(field.getNameAsString(), field.getOffset(), field.getType()));
      }

      IDLByteSequence dataSequence = message.getData();
      byte[] data = new byte[dataSequence.size()];
      for (int i = 0; i < data.length; i++)
         data[i] = dataSequence.get(i);

      return new HeightMapData(message.getSequenceId(),
                                message.getControllerTimestamp(),
                                message.getFrameIdAsString(),
                                message.getPose().getPosition().getX(),
                                message.getPose().getPosition().getY(),
                                message.getPose().getPosition().getZ(),
                                message.getPose().getOrientation().getX(),
                                message.getPose().getOrientation().getY(),
                                message.getPose().getOrientation().getZ(),
                                message.getPose().getOrientation().getW(),
                                message.getColumnCount(),
                                message.getCellSize().getX(),
                                message.getCellSize().getY(),
                                message.getRowStride(),
                                message.getCellStride(),
                                fields,
                                data);
   }

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
