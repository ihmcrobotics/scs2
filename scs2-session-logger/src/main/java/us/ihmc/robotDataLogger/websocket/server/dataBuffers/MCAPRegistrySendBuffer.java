package us.ihmc.robotDataLogger.websocket.server.dataBuffers;

import us.ihmc.scs2.session.mcap.output.MCAPByteBufferDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.MutableChunk;
import us.ihmc.scs2.session.mcap.specs.records.MutableMessage;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.nio.ByteBuffer;

public class MCAPRegistrySendBuffer
{
   private int registryID;
   private long timestamp;
   private long transmitTime;
   private int numberOfVariables;
   private long sequenceID = 0;
   private final MCAPBuilder mcapBuilder;
   private final MCAPByteBufferDataOutput dataOutput;
   private final YoVariable[] variables;

   private final MutableChunk chunk = new MutableChunk();
   private final MutableRecord chunkRecord = new MutableRecord(chunk);

   protected MCAPRegistrySendBuffer(MCAPBuilder mcapBuilder, int registryID, YoRegistry registry)
   {
      this.mcapBuilder = mcapBuilder;

      variables = registry.collectSubtreeVariables().toArray(new YoVariable[0]);
      int numberOfVariables = variables.length; // TODO Figure out joint states
      dataOutput = new MCAPByteBufferDataOutput(numberOfVariables * 8, 2, false);

      this.registryID = registryID;
   }

   public void update(long timestamp, long sequenceID)
   {
      this.sequenceID = sequenceID;
      this.timestamp = timestamp;
      transmitTime = System.nanoTime();
      this.numberOfVariables = variables.length;
      addSchemas();
      addChannels();
      addMessages(timestamp);
      dataOutput.getBuffer().clear();
      chunkRecord.write(dataOutput);
      dataOutput.getBuffer().flip();
   }

   private void addSchemas()
   {
      chunk.records().add(mcapBuilder.getVariableSchemaRecord(YoBoolean.class));
      chunk.records().add(mcapBuilder.getVariableSchemaRecord(YoDouble.class));
      chunk.records().add(mcapBuilder.getVariableSchemaRecord(YoInteger.class));
      chunk.records().add(mcapBuilder.getVariableSchemaRecord(YoLong.class));
      chunk.records().add(mcapBuilder.getVariableSchemaRecord(YoEnum.class));
   }

   public void addChannels()
   {
      for (YoVariable variable : variables)
      {
         chunk.records().add(mcapBuilder.getOrCreateVariableChannelRecord(variable));
      }
   }

   private void addMessages(long timestamp)
   {
      for (YoVariable variable : variables)
      {
         MutableMessage message = new MutableMessage();
         mcapBuilder.packVariableMessage(variable, message);
         message.setPublishTime(timestamp);
         message.setLogTime(timestamp);
         message.setSequence(sequenceID);

         chunk.records().add(new MutableRecord(message));
      }
   }

   public ByteBuffer getBuffer()
   {
      return dataOutput.getBuffer();
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public long getSequenceID()
   {
      return sequenceID;
   }
}
