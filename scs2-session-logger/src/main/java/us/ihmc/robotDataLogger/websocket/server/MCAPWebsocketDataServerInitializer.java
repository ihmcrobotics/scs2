package us.ihmc.robotDataLogger.websocket.server;

import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.socket.SocketChannel;
import io.netty.handler.codec.http.HttpObjectAggregator;
import io.netty.handler.codec.http.HttpServerCodec;
import io.netty.handler.codec.http.websocketx.WebSocketServerProtocolHandler;
import us.ihmc.robotDataLogger.logger.LogAliveListener;
import us.ihmc.robotDataLogger.websocket.interfaces.MCAPMessageListener;

/**
 * Initializes a connection
 *
 * @author Jesper Smith
 */
class MCAPWebsocketDataServerInitializer extends ChannelInitializer<SocketChannel>
{
   private static final String WEBSOCKET_PATH = "/websocket";

   private final MCAPDataServerServerContent logServerContent;
   private final MCAPWebsocketDataBroadcaster broadcaster;
   private final MCAPMessageListener variableChangedMessageListener;
   private final LogAliveListener logAliveListener;
   private final int dataSize;
   private final int numberOfRegistryBuffers;

   public MCAPWebsocketDataServerInitializer(MCAPDataServerServerContent logServerContent,
                                             MCAPWebsocketDataBroadcaster broadcaster,
                                             MCAPMessageListener variableChangedMessageListener,
                                             LogAliveListener logAliveListener,
                                             int dataSize,
                                             int numberOfRegistryBuffers)
   {
      this.logServerContent = logServerContent;
      this.broadcaster = broadcaster;
      this.dataSize = dataSize;
      this.variableChangedMessageListener = variableChangedMessageListener;
      this.logAliveListener = logAliveListener;
      this.numberOfRegistryBuffers = numberOfRegistryBuffers;
   }

   @Override
   protected void initChannel(SocketChannel ch) throws Exception
   {
      ChannelPipeline pipeline = ch.pipeline();

      pipeline.addLast(new DataServerIOExceptionHandler());
      pipeline.addLast(new HttpServerCodec());
      pipeline.addLast(new HttpObjectAggregator(65536));
      pipeline.addLast(new WebSocketServerProtocolHandler(WEBSOCKET_PATH, null, true));
      pipeline.addLast(new HTTPMCAPDataServerDescriptionServer(logServerContent));
      pipeline.addLast(new MCAPWebsocketDataServerFrameHandler(broadcaster,
                                                               dataSize,
                                                               numberOfRegistryBuffers,
                                                               variableChangedMessageListener,
                                                               logAliveListener));
   }
}
