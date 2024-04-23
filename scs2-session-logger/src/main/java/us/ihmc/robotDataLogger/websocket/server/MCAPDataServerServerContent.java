package us.ihmc.robotDataLogger.websocket.server;

import io.netty.buffer.ByteBuf;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.client.discovery.WebsocketMCAPStarter;
import us.ihmc.robotDataLogger.websocket.client.discovery.WebsocketResourcesAttachment;
import us.ihmc.robotDataLogger.websocket.dataBuffers.MCAPRegistrySendBufferBuilder;
import us.ihmc.scs2.session.mcap.output.MCAPNettyByteBufDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.List;

public class MCAPDataServerServerContent
{
   public static final String MCAP_STARTER = "/mcap_starter.mcap";
   public static final String ROBOT_MODEL_RESOURCES = "/robot_model_resources.zip";

   private final ByteBuf mcapStarterBuffer;
   private final ByteBuf robotModelResourcesBuffer;

   public MCAPDataServerServerContent(String name,
                                      MCAPBuilder mcapBuilder,
                                      LogModelProvider logModelProvider,
                                      DataServerSettings dataServerSettings,
                                      List<MCAPRegistrySendBufferBuilder> registeredBuffers)
   {
      WebsocketMCAPStarter mcapStarter = new WebsocketMCAPStarter(name, getHostName(), logModelProvider, dataServerSettings, mcapBuilder);
      registeredBuffers.forEach(buffer -> mcapStarter.channelStarterChunk().addYoVariables(buffer.getYoRegistry().collectSubtreeVariables()));

      MCAPNettyByteBufDataOutput output = new MCAPNettyByteBufDataOutput(false);
      mcapStarter.write(output);
      mcapStarterBuffer = output.getBuffer();

      // Separate resources as they can be quite large
      WebsocketResourcesAttachment resourcesAttachment = WebsocketResourcesAttachment.create(logModelProvider);

      if (resourcesAttachment != null)
      {
         MCAPNettyByteBufDataOutput resourcesOutput = new MCAPNettyByteBufDataOutput(false);
         resourcesAttachment.write(resourcesOutput);
         robotModelResourcesBuffer = resourcesOutput.getBuffer();
      }
      else
      {
         robotModelResourcesBuffer = null;
      }
   }

   private static String getHostName()
   {
      try
      {
         return InetAddress.getLocalHost().getHostName();
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }

   public ByteBuf getMCAPStarterBuffer()
   {
      return mcapStarterBuffer.retainedDuplicate();
   }

   public ByteBuf getRobotModelResourcesBuffer()
   {
      return robotModelResourcesBuffer == null ? null : robotModelResourcesBuffer.retainedDuplicate();
   }
}
