package us.ihmc.robotDataLogger;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.interfaces.RegistryPublisher;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.dataBuffers.MCAPRegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.websocket.server.MCAPDataServerServerContent;
import us.ihmc.robotDataLogger.websocket.server.MCAPWebsocketDataProducer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class YoMCAPVariableServer implements RobotVisualizer, VariableChangedListener
{
   private static final int CHANGED_BUFFER_CAPACITY = 128;

   private final MCAPBuilder mcapBuilder = new MCAPBuilder();
   private final double dt;

   private final String name;
   private final LogModelProvider logModelProvider;
   private final DataServerSettings dataServerSettings;

   private YoRegistry mainRegistry = null;
   private final List<MCAPRegistrySendBufferBuilder> registeredBuffers = new ArrayList<>();

   private final List<RegistryHolder> registryHolders = new ArrayList<>();

   // State
   private boolean started = false;
   private boolean stopped = false;

   private MCAPWebsocketDataProducer dataProducer;

   private volatile long latestTimestamp;

   private final LogWatcher logWatcher = new LogWatcher();

   public YoMCAPVariableServer(String mainClazz, LogModelProvider logModelProvider, DataServerSettings dataServerSettings, double dt)
   {
      this.dt = dt;
      this.name = mainClazz;
      this.logModelProvider = logModelProvider;
      this.dataServerSettings = dataServerSettings;
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

      try
      {
         dataProducer = new MCAPWebsocketDataProducer(this, logWatcher, dataServerSettings);

         for (int i = 0; i < registeredBuffers.size(); i++)
         {
            MCAPRegistrySendBufferBuilder builder = registeredBuffers.get(i);
            builder.build(i);
            YoRegistry registry = builder.getYoRegistry();

            try
            {
               ConcurrentRingBuffer<VariableChangedMessage> variableChangeData = new ConcurrentRingBuffer<>(new VariableChangedMessage.Builder(),
                                                                                                            CHANGED_BUFFER_CAPACITY);
               RegistryPublisher publisher = dataProducer.createRegistryPublisher(builder);

               registryHolders.add(new RegistryHolder(registry, publisher, variableChangeData));

               publisher.start();
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
         }

         MCAPDataServerServerContent serverContent = new MCAPDataServerServerContent(name,
                                                                                     mcapBuilder,
                                                                                     logModelProvider,
                                                                                     dataServerSettings,
                                                                                     registeredBuffers);
         dataProducer.setDataServerContent(serverContent);
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

      registeredBuffers.add(new MCAPRegistrySendBufferBuilder(mcapBuilder, registry, Collections.emptyList(), scs1YoGraphics, scs2YoGraphics));
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
      registeredBuffers.add(new MCAPRegistrySendBufferBuilder(mcapBuilder, registry, jointsToPublish, scs1YoGraphics, scs2YoGraphics));
      mainRegistry = registry;
   }

   private YoVariable findVariableInRegistries(String variableName)
   {

      for (MCAPRegistrySendBufferBuilder buffer : registeredBuffers)
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
      // FIXME
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
