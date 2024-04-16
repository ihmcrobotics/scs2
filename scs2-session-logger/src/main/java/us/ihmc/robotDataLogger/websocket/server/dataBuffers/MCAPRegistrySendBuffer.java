package us.ihmc.robotDataLogger.websocket.server.dataBuffers;

import us.ihmc.robotDataLogger.dataBuffers.RegistryBuffer;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.scs2.session.mcap.output.MCAPByteBufferDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.nio.ByteBuffer;

public class MCAPRegistrySendBuffer extends RegistryBuffer
{
   private final MCAPBuilder mcapBuilder;
   private final MCAPByteBufferDataOutput dataOutput;
   private final YoVariable[] variables;

   protected MCAPRegistrySendBuffer(MCAPBuilder mcapBuilder, int registryID, YoRegistry registry)
   {
      this.mcapBuilder = mcapBuilder;

      variables = registry.collectSubtreeVariables().toArray(new YoVariable[0]);
      int numberOfVariables = variables.length; // TODO Figure out joint states
      dataOutput = new MCAPByteBufferDataOutput(numberOfVariables * 8, 2, false);

      this.registryID = registryID;
   }

   public void initializeSchemas()
   {
      ByteBuffer dataOutputBuffer = dataOutput.getBuffer();
      dataOutputBuffer.clear();
      for (YoVariable variable : variables)
      {
         Record variableSchemaRecord = mcapBuilder.getVariableSchemaRecord(variable.getClass());
         variableSchemaRecord.write(dataOutput);
      }
      dataOutputBuffer.flip();
      dataOutputBuffer.clear();
      dataOutputBuffer.limit(dataOutputBuffer.limit() * 8);
   }

   public void initializeChannels()
   {
      ByteBuffer dataOutputBuffer = dataOutput.getBuffer();
      dataOutputBuffer.clear();
      for (YoVariable variable : variables)
      {
         Record variableChannelRecord = mcapBuilder.getOrCreateVariableChannelRecord(variable);
         variableChannelRecord.write(dataOutput);
      }
      dataOutputBuffer.flip();
      dataOutputBuffer.clear();
      dataOutputBuffer.limit(dataOutputBuffer.limit() * 8);
   }

   /**
    * Pack the internal buffer with data from the variables
    *
    * @param timestamp
    * @param uid
    */
   public void updateBufferFromVariables(long timestamp, long uid)
   {
      this.uid = uid;
      this.timestamp = timestamp;
      transmitTime = System.nanoTime();
      this.numberOfVariables = variables.length;
      data.clear();
      for (int i = 0; i < numberOfVariables; i++)
      {
         data.put(variables[i].getValueAsLongBits());
      }
      data.flip();
      dataOutpu.clear();
      dataOutpu.limit(data.limit() * 8);
      int jointOffset = 0;
      for (JointHolder jointHolder : jointHolders)
      {
         jointHolder.get(jointStates, jointOffset);
         jointOffset += jointHolder.getNumberOfStateVariables();
      }
   }

   public double[] getJointStates()
   {
      return jointStates;
   }

   public ByteBuffer getBuffer()
   {
      return dataOutput.getBuffer();
   }
}
