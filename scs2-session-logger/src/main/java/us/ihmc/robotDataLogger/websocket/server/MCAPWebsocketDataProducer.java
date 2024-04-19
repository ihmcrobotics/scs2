package us.ihmc.robotDataLogger.websocket.server;

import io.netty.bootstrap.ServerBootstrap;
import io.netty.channel.Channel;
import io.netty.channel.ChannelFuture;
import io.netty.channel.EventLoopGroup;
import io.netty.handler.logging.LogLevel;
import io.netty.handler.logging.LoggingHandler;
import io.netty.util.ResourceLeakDetector;
import io.netty.util.ResourceLeakDetector.Level;
import us.ihmc.robotDataLogger.interfaces.RegistryPublisher;
import us.ihmc.robotDataLogger.listeners.VariableChangedListener;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.logger.LogAliveListener;
import us.ihmc.robotDataLogger.util.NettyUtils;
import us.ihmc.robotDataLogger.websocket.dataBuffers.MCAPRegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.websocket.server.discovery.DataServerLocationBroadcastSender;

import java.io.IOException;

public class MCAPWebsocketDataProducer
{
   private final MCAPWebsocketDataBroadcaster broadcaster = new MCAPWebsocketDataBroadcaster();
   private final VariableChangedListener variableChangedListener;
   private final LogAliveListener logAliveListener;

   private final int port;

   private final Object lock = new Object();
   private Channel channel = null;

   private final EventLoopGroup bossGroup = NettyUtils.createEventGroundLoop(1);

   /**
    * Create a single worker. If "writeAndFlush" is called in the eventloop of the outbound channel, no
    * extra objects will be created. The registryPublisher is scheduled on the main eventloop to avoid
    * having extra threads and delay.
    */
   private final EventLoopGroup workerGroup = NettyUtils.createEventGroundLoop(1);

   private DataServerLocationBroadcastSender broadcastSender;

   private MCAPDataServerServerContent dataServerContent;

   // TODO Figure out how to either determine the max buffer size or remove it.
   private final int maximumBufferSize = 200000;

   private final boolean autoDiscoverable;

   private int nextBufferID = 0;

   public MCAPWebsocketDataProducer(VariableChangedListener variableChangedListener, LogAliveListener logAliveListener, DataServerSettings dataServerSettings)
   {
      this.variableChangedListener = variableChangedListener;
      this.logAliveListener = logAliveListener;
      port = dataServerSettings.getPort();
      autoDiscoverable = dataServerSettings.isAutoDiscoverable();
   }

   public void remove()
   {
      synchronized (lock)
      {
         try
         {
            if (broadcastSender != null)
               broadcastSender.stop();
            if (broadcaster != null)
               broadcaster.stop();

            if (channel != null)
            {
               ChannelFuture closeFuture = channel.close();
               closeFuture.sync();
            }

            if (bossGroup != null)
               bossGroup.shutdownGracefully();

            if (workerGroup != null)
               workerGroup.shutdownGracefully();
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }

   public void setDataServerContent(MCAPDataServerServerContent dataServerServerContent)
   {
      this.dataServerContent = dataServerServerContent;
   }

   public void announce() throws IOException
   {
      if (dataServerContent == null)
      {
         throw new RuntimeException("No content provided");
      }

      synchronized (lock)
      {
         ResourceLeakDetector.setLevel(Level.DISABLED);
         try
         {
            int numberOfRegistryBuffers = nextBufferID; // Next buffer ID is incremented the last time a registry was added
            ServerBootstrap serverBootstrap = new ServerBootstrap();
            serverBootstrap.group(bossGroup, workerGroup)
                           .channel(NettyUtils.getServerSocketChannelClass())
                           .handler(new LoggingHandler(LogLevel.INFO))
                           .childHandler(new MCAPWebsocketDataServerInitializer(dataServerContent,
                                                                                broadcaster,
                                                                                variableChangedListener,
                                                                                logAliveListener,
                                                                                maximumBufferSize,
                                                                                numberOfRegistryBuffers));

            channel = serverBootstrap.bind(port).sync().channel();

            if (autoDiscoverable)
            {
               broadcastSender = new DataServerLocationBroadcastSender(port);
               broadcastSender.start();
            }
            else
            {
               broadcastSender = null;
            }
         }
         catch (InterruptedException e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   public void publishTimestamp(long timestamp)
   {
      broadcaster.publishTimestamp(timestamp);
   }

   public RegistryPublisher createRegistryPublisher(MCAPRegistrySendBufferBuilder builder) throws IOException
   {
      MCAPWebsocketRegistryPublisher websocketRegistryPublisher = new MCAPWebsocketRegistryPublisher(workerGroup, builder, broadcaster, nextBufferID);
      nextBufferID++;
      return websocketRegistryPublisher;
   }
}
