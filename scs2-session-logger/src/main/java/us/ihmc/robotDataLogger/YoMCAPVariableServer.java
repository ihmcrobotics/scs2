package us.ihmc.robotDataLogger;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.handshake.SummaryProvider;
import us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder;
import us.ihmc.robotDataLogger.interfaces.BufferListenerInterface;
import us.ihmc.robotDataLogger.interfaces.DataProducer;
import us.ihmc.robotDataLogger.interfaces.RegistryPublisher;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.server.DataServerServerContent;
import us.ihmc.robotDataLogger.websocket.server.WebsocketDataProducer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class YoMCAPVariableServer implements RobotVisualizer, VariableChangedListener
{
   private static final int CHANGED_BUFFER_CAPACITY = 128;

   private final double dt;

   private final String name;
   private final LogModelProvider logModelProvider;
   private final DataServerSettings dataServerSettings;

   private String rootRegistryName = "main";

   private YoRegistry mainRegistry = null;
   private final ArrayList<RegistrySendBufferBuilder> registeredBuffers = new ArrayList<>();

   private final ArrayList<RegistryHolder> registryHolders = new ArrayList<>();

   // State
   private boolean started = false;
   private boolean stopped = false;

   // Servers
   private final DataProducer dataProducer;
   private YoVariableHandShakeBuilder handshakeBuilder;

   private volatile long latestTimestamp;

   private final SummaryProvider summaryProvider = new SummaryProvider();

   private final LogWatcher logWatcher = new LogWatcher();

   private BufferListenerInterface bufferListener = null;

   public YoMCAPVariableServer(String mainClazz, LogModelProvider logModelProvider, DataServerSettings dataServerSettings, double dt)
   {
      this.dt = dt;
      this.name = mainClazz;
      this.logModelProvider = logModelProvider;
      this.dataServerSettings = dataServerSettings;

      dataProducer = new WebsocketDataProducer(this, logWatcher, dataServerSettings);
   }

   public void setRootRegistryName(String name)
   {
      rootRegistryName = name;
   }

   /**
    * Add a listener for new buffer data
    *
    * This could be used to implement, for example, a in-memory logger
    *
    * @param bufferListener
    */
   public synchronized void addBufferListener(BufferListenerInterface bufferListener)
   {
      if (started)
      {
         throw new RuntimeException("Server already started");
      }

      this.bufferListener = bufferListener;
   }

   public synchronized void start()
   {
      if (started)
      {
         throw new RuntimeException("Server already started");
      }

      if (stopped)
      {
         throw new RuntimeException("Cannot restart a YoVariable server");
      }

      handshakeBuilder = new YoVariableHandShakeBuilder(rootRegistryName, dt);
      handshakeBuilder.setFrames(ReferenceFrame.getWorldFrame());
      handshakeBuilder.setSummaryProvider(summaryProvider);

      for (int i = 0; i < registeredBuffers.size(); i++)
      {
         RegistrySendBufferBuilder builder = registeredBuffers.get(i);
         handshakeBuilder.addRegistryBuffer(builder);
      }

      try
      {
         if (bufferListener != null)
         {
            bufferListener.allocateBuffers(registeredBuffers.size());
         }
         for (int i = 0; i < registeredBuffers.size(); i++)
         {
            RegistrySendBufferBuilder builder = registeredBuffers.get(i);
            YoRegistry registry = builder.getYoRegistry();

            try
            {
               ConcurrentRingBuffer<VariableChangedMessage> variableChangeData = new ConcurrentRingBuffer<>(new VariableChangedMessage.Builder(),
                                                                                                            CHANGED_BUFFER_CAPACITY);
               RegistryPublisher publisher = dataProducer.createRegistryPublisher(builder, bufferListener);

               registryHolders.add(new RegistryHolder(registry, publisher, variableChangeData));

               publisher.start();
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
         }

         DataServerServerContent content = new DataServerServerContent(name, handshakeBuilder.getHandShake(), logModelProvider, dataServerSettings);

         if (bufferListener != null)
         {
            bufferListener.setContent(content);
            bufferListener.start();
         }

         dataProducer.setDataServerContent(content);
         dataProducer.announce();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      started = true;
   }

   private RegistryHolder getRegistryHolder(YoRegistry registry)
   {
      for (int i = 0; i < registryHolders.size(); i++)
      {
         RegistryHolder registryHolder = registryHolders.get(i);

         if (registryHolder.registry == registry)
         {
            return registryHolder;
         }
      }

      throw new RuntimeException("Registry " + registry.getName() + " not registered with addRegistry() or setMainRegistry()");
   }

   @Override
   public synchronized void close()
   {
      if (started && !stopped)
      {
         started = false;
         stopped = true;
         for (int i = 0; i < registryHolders.size(); i++)
         {
            registryHolders.get(i).publisher.stop();
         }
         dataProducer.remove();

         if (bufferListener != null)
         {
            bufferListener.close();
         }
      }
   }

   /**
    * Update main buffer data. Note: If the timestamp is not increasing between updates(), no data
    * might be sent to clients.
    *
    * @param timestamp timestamp to send to logger
    */
   @Override
   public void update(long timestamp)
   {
      update(timestamp, mainRegistry);
   }

   /**
    * Update registry data Note: If the timestamp is not increasing between updates(), no data might be
    * sent to clients.
    *
    * @param timestamp timestamp to send to the logger
    * @param registry  Top level registry to update
    */
   @Override
   public void update(long timestamp, YoRegistry registry)
   {
      if (!started || stopped)
      {
         return;
      }
      if (registry == mainRegistry)
      {
         dataProducer.publishTimestamp(timestamp);
         latestTimestamp = timestamp;
      }

      RegistryHolder registryHolder = getRegistryHolder(registry);

      registryHolder.publisher.update(timestamp);
      updateChangedVariables(registryHolder);

      logWatcher.update(timestamp);
   }

   private void updateChangedVariables(RegistryHolder rootRegistry)
   {
      ConcurrentRingBuffer<VariableChangedMessage> buffer = rootRegistry.variableChangeData;
      buffer.poll();
      VariableChangedMessage msg;

      while ((msg = buffer.read()) != null)
      {
         msg.getVariable().setValueFromDouble(msg.getVal());
      }

      buffer.flush();
   }

   @Override
   public void addRegistry(YoRegistry registry, YoGraphicsListRegistry scs1YoGraphics, YoGraphicGroupDefinition scs2YoGraphics)
   {
      if (mainRegistry == null)
      {
         throw new RuntimeException("Main registry is not set. Set main registry first");
      }

      registeredBuffers.add(new RegistrySendBufferBuilder(registry, scs1YoGraphics, scs2YoGraphics));
   }

   @Override
   public void setMainRegistry(YoRegistry registry,
                               List<? extends JointBasics> jointsToPublish,
                               YoGraphicsListRegistry scs1YoGraphics,
                               YoGraphicGroupDefinition scs2YoGraphics)
   {
      if (mainRegistry != null)
      {
         throw new RuntimeException("Main registry is already set");
      }
      registeredBuffers.add(new RegistrySendBufferBuilder(registry, jointsToPublish, scs1YoGraphics, scs2YoGraphics));
      mainRegistry = registry;
   }

   private YoVariable findVariableInRegistries(String variableName)
   {

      for (RegistrySendBufferBuilder buffer : registeredBuffers)
      {
         YoRegistry registry = buffer.getYoRegistry();
         YoVariable ret = registry.findVariable(variableName);
         if (ret != null)
         {
            return ret;
         }
      }
      return null;
   }

   public void createSummary(YoVariable isWalkingVariable)
   {
      createSummary(isWalkingVariable.getFullNameString());
   }

   public void createSummary(String summaryTriggerVariable)
   {
      if (findVariableInRegistries(summaryTriggerVariable) == null)
      {
         throw new RuntimeException("Variable " + summaryTriggerVariable + " is not registered with the logger");
      }
      summaryProvider.setSummarize(true);
      summaryProvider.setSummaryTriggerVariable(summaryTriggerVariable);
   }

   public void addSummarizedVariable(String variable)
   {
      if (findVariableInRegistries(variable) == null)
      {
         throw new RuntimeException("Variable " + variable + " is not registered with the logger");
      }
      summaryProvider.addSummarizedVariable(variable);
   }

   public void addSummarizedVariable(YoVariable variable)
   {
      summaryProvider.addSummarizedVariable(variable);
   }

   @Override
   public long getLatestTimestamp()
   {
      return latestTimestamp;
   }

   public boolean isLogging()
   {
      return logWatcher.isLogging();
   }

   @Override
   public void changeVariable(int id, double newValue)
   {
      VariableChangedMessage message;
      ImmutablePair<YoVariable, YoRegistry> variableAndRootRegistry = handshakeBuilder.getVariablesAndRootRegistries().get(id);

      RegistryHolder holder = getRegistryHolder(variableAndRootRegistry.getRight());
      ConcurrentRingBuffer<VariableChangedMessage> buffer = holder.variableChangeData;
      while ((message = buffer.next()) == null)
      {
         ThreadTools.sleep(1);
      }

      if (message != null)
      {
         message.setVariable(variableAndRootRegistry.getLeft());
         message.setVal(newValue);
         buffer.commit();
      }
   }

   private class RegistryHolder
   {
      private final YoRegistry registry;
      private final RegistryPublisher publisher;
      private final ConcurrentRingBuffer<VariableChangedMessage> variableChangeData;

      public RegistryHolder(YoRegistry registry, RegistryPublisher publisher, ConcurrentRingBuffer<VariableChangedMessage> variableChangeData)
      {
         this.registry = registry;
         this.publisher = publisher;
         this.variableChangeData = variableChangeData;
      }
   }
}
