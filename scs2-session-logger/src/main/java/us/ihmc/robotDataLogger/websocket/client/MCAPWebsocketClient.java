package us.ihmc.robotDataLogger.websocket.client;

import io.netty.bootstrap.Bootstrap;
import io.netty.buffer.ByteBuf;
import io.netty.channel.Channel;
import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelOption;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.ChannelPromise;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.channel.socket.SocketChannel;
import io.netty.handler.codec.http.DefaultHttpHeaders;
import io.netty.handler.codec.http.FullHttpResponse;
import io.netty.handler.codec.http.HttpClientCodec;
import io.netty.handler.codec.http.HttpObjectAggregator;
import io.netty.handler.codec.http.websocketx.BinaryWebSocketFrame;
import io.netty.handler.codec.http.websocketx.CloseWebSocketFrame;
import io.netty.handler.codec.http.websocketx.PingWebSocketFrame;
import io.netty.handler.codec.http.websocketx.PongWebSocketFrame;
import io.netty.handler.codec.http.websocketx.TextWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketClientHandshaker;
import io.netty.handler.codec.http.websocketx.WebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketVersion;
import io.netty.handler.codec.http.websocketx.extensions.compression.WebSocketClientCompressionHandler;
import io.netty.handler.timeout.IdleState;
import io.netty.handler.timeout.IdleStateEvent;
import io.netty.handler.timeout.IdleStateHandler;
import io.netty.util.CharsetUtil;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.listeners.TimestampListener;
import us.ihmc.robotDataLogger.util.NettyUtils;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPMCAPServerConnection;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPMCAPServerConnection.DisconnectPromise;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.robotDataLogger.websocket.dataBuffers.MCAPDataReceiveScheduler;
import us.ihmc.robotDataLogger.websocket.interfaces.ConnectionStateListener;
import us.ihmc.robotDataLogger.websocket.interfaces.DataServerCommandConsumer;
import us.ihmc.robotDataLogger.websocket.interfaces.MCAPRecordListener;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPNettyByteBufDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.Record;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.ByteBuffer;
import java.util.concurrent.TimeUnit;

import static io.netty.handler.codec.http.websocketx.WebSocketClientHandshakerFactory.newHandshaker;

public class MCAPWebsocketClient
{
   private final EventLoopGroup group = NettyUtils.createEventGroundLoop();
   private final MCAPDataReceiveScheduler dataReceiveScheduler;

   private final Channel channel;

   private final DisconnectPromise disconnectPromise;
   private final UDPTimestampClient udpTimestampClient;

   private final Handler handler;

   public MCAPWebsocketClient(HTTPMCAPServerConnection connection,
                              TimestampListener timestampListener,
                              MCAPRecordListener mcapRecordListener,
                              DataServerCommandConsumer dataServerCommandConsumer,
                              ConnectionStateListener connectionStateListener,
                              int timeoutInMs) throws IOException
   {
      disconnectPromise = connection.take();
      HTTPDataServerDescription target = connection.getTarget();

      URI uri;
      try
      {
         uri = new URI("ws://" + target.getHost() + ":" + target.getPort() + "/websocket");
      }
      catch (URISyntaxException e)
      {
         throw new IOException(e);
      }

      dataReceiveScheduler = new MCAPDataReceiveScheduler(mcapRecordListener, connectionStateListener);
      udpTimestampClient = new UDPTimestampClient(timestampListener);
      udpTimestampClient.start();

      handler = new Handler(newHandshaker(uri, WebSocketVersion.V13, null, true, new DefaultHttpHeaders()),
                            udpTimestampClient.getPort(),
                            dataReceiveScheduler,
                            connectionStateListener,
                            dataServerCommandConsumer);

      Bootstrap b = new Bootstrap();
      b.group(group).channel(NettyUtils.getSocketChannelClass()).handler(new ChannelInitializer<SocketChannel>()
      {
         @Override
         protected void initChannel(SocketChannel ch)
         {
            ChannelPipeline p = ch.pipeline();
            p.addLast(new HttpClientCodec(),
                      new HttpObjectAggregator(65536),
                      WebSocketClientCompressionHandler.INSTANCE,
                      new IdleStateHandler(timeoutInMs, 0, 0, TimeUnit.MILLISECONDS),
                      handler);
         }
      });

      b.option(ChannelOption.CONNECT_TIMEOUT_MILLIS, 1000);

      try
      {
         channel = b.connect(uri.getHost(), uri.getPort()).syncUninterruptibly().channel();
         channel.closeFuture().addListener((e) -> disconnected());
      }
      catch (Exception e)
      {
         disconnected();
         throw new IOException(e);
      }
   }

   private void disconnected()
   {
      udpTimestampClient.stop();
      udpTimestampClient.join();
      dataReceiveScheduler.stopImmediately();
      try
      {
         dataReceiveScheduler.join();
      }
      catch (InterruptedException e)
      {
      }
      disconnectPromise.complete();
      group.shutdownGracefully();
   }

   public boolean isActive()
   {
      return channel.isActive();
   }

   public void close()
   {
      channel.close();
   }

   public void sendRecord(Record record)
   {
      try
      {
         ByteBuf data = channel.alloc().buffer((int) record.getElementLength());
         record.write(new MCAPNettyByteBufDataOutput(data));
         BinaryWebSocketFrame frame = new BinaryWebSocketFrame(data);
         channel.writeAndFlush(frame);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void sendCommand(DataServerCommand command, int argument)
   {
      try
      {
         ByteBuf cmdBuf = channel.alloc().buffer(DataServerCommand.MaxCommandSize());
         command.getBytes(cmdBuf, argument);
         TextWebSocketFrame frame = new TextWebSocketFrame(cmdBuf);

         channel.writeAndFlush(frame);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private static class Handler extends SimpleChannelInboundHandler<Object>
   {
      private final WebSocketClientHandshaker handshaker;
      private final int timestampPort;
      private final MCAPDataReceiveScheduler scheduler;
      private final DataServerCommandConsumer dataServerCommandConsumer;

      private final ConnectionStateListener connectionStateListener;

      private ChannelPromise handshakeFuture;

      private boolean sendConfiguration = false;

      private volatile boolean waitingForPong = false;

      public Handler(WebSocketClientHandshaker handshaker,
                     int timestampPort,
                     MCAPDataReceiveScheduler scheduler,
                     ConnectionStateListener connectionStateListener,
                     DataServerCommandConsumer dataServerCommandConsumer)
      {
         this.handshaker = handshaker;
         this.scheduler = scheduler;
         this.timestampPort = timestampPort;
         this.connectionStateListener = connectionStateListener;
         this.dataServerCommandConsumer = dataServerCommandConsumer;
      }

      public ChannelFuture handshakeFuture()
      {
         return handshakeFuture;
      }

      @Override
      public void handlerAdded(ChannelHandlerContext context)
      {
         handshakeFuture = context.newPromise();
      }

      @Override
      public void channelActive(ChannelHandlerContext context)
      {
         handshaker.handshake(context.channel());
      }

      @Override
      public void channelRead0(ChannelHandlerContext context, Object message) throws Exception
      {
         Channel channel = context.channel();
         if (!handshaker.isHandshakeComplete())
         {
            handshaker.finishHandshake(channel, (FullHttpResponse) message);
            connectionStateListener.connected();
            handshakeFuture.setSuccess();

            return;
         }

         if (message instanceof FullHttpResponse response)
         {
            throw new IllegalStateException(
                  "Unexpected FullHttpResponse (getStatus=" + response.status() + ", content=" + response.content().toString(CharsetUtil.UTF_8) + ')');
         }

         WebSocketFrame frame = (WebSocketFrame) message;
         if (frame instanceof TextWebSocketFrame)
         {
            DataServerCommand command = DataServerCommand.getCommand(frame.content());
            if (dataServerCommandConsumer != null && command != null)
            {
               int argument = command.getArgument(frame.content());
               if (argument != -1)
               {
                  dataServerCommandConsumer.receivedCommand(command, argument);
               }
            }
         }
         else if (frame instanceof BinaryWebSocketFrame)
         {
            ByteBuffer byteBuffer = ByteBuffer.allocate(frame.content().readableBytes());
            frame.content().readBytes(byteBuffer);
            byteBuffer.flip();
            scheduler.onNewData(MCAPDataInput.wrap(byteBuffer));

            if (!sendConfiguration)
            {
               ByteBuf sendTimestampCmd = context.alloc().buffer(DataServerCommand.MaxCommandSize());
               DataServerCommand.SEND_TIMESTAMPS.getBytes(sendTimestampCmd, timestampPort);
               TextWebSocketFrame sendTimestampFrame = new TextWebSocketFrame(sendTimestampCmd);
               channel.writeAndFlush(sendTimestampFrame);
               sendConfiguration = true;
            }
         }
         else if (frame instanceof PongWebSocketFrame)
         {
            waitingForPong = false;
         }
         else if (frame instanceof CloseWebSocketFrame)
         {
            LogTools.info("Connection closed by server");
            channel.close();
         }
      }

      @Override
      public void userEventTriggered(ChannelHandlerContext context, Object event)
      {
         if (event instanceof IdleStateEvent)
         {
            IdleState idleState = ((IdleStateEvent) event).state();
            if (idleState == IdleState.READER_IDLE)
            {
               if (waitingForPong)
               {
                  LogTools.warn("Timeout receiving pong. Closing connection.");
                  context.close();
               }
               else
               {
                  waitingForPong = true;
                  context.channel().writeAndFlush(new PingWebSocketFrame());
               }
            }
         }
      }

      @Override
      public void exceptionCaught(ChannelHandlerContext context, Throwable cause)
      {
         LogTools.warn("Connection closed: " + cause.getMessage());
         if (!handshakeFuture.isDone())
         {
            handshakeFuture.setFailure(cause);
         }
         context.close();
      }
   }
}
