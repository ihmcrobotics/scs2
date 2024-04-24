package us.ihmc.robotDataLogger.websocket.dataBuffers;

import us.ihmc.scs2.session.mcap.output.MCAPByteBufferDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.Compression;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.MutableChunk;
import us.ihmc.scs2.session.mcap.specs.records.MutableMessage;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.nio.ByteBuffer;
import java.util.ArrayList;

public class MCAPRegistrySendBuffer
{
   private final int registryID;
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
      chunk.setRecords(new ArrayList<>());
      addMessages(timestamp);
      dataOutput.getBuffer().clear();
      chunk.setCompression(Compression.NONE);
      chunkRecord.write(dataOutput);
      dataOutput.getBuffer().flip();
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
