package us.ihmc.robotDataLogger.websocket.client;

import io.netty.buffer.ByteBuf;
import io.netty.channel.Channel;
import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelPromise;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.handler.codec.http.FullHttpResponse;
import io.netty.handler.codec.http.websocketx.BinaryWebSocketFrame;
import io.netty.handler.codec.http.websocketx.CloseWebSocketFrame;
import io.netty.handler.codec.http.websocketx.PingWebSocketFrame;
import io.netty.handler.codec.http.websocketx.PongWebSocketFrame;
import io.netty.handler.codec.http.websocketx.TextWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketClientHandshaker;
import io.netty.handler.codec.http.websocketx.WebSocketFrame;
import io.netty.handler.timeout.IdleState;
import io.netty.handler.timeout.IdleStateEvent;
import io.netty.util.CharsetUtil;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.robotDataLogger.websocket.dataBuffers.ConnectionStateListener;
import us.ihmc.robotDataLogger.websocket.dataBuffers.MCAPDataScheduler;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;

import java.nio.ByteBuffer;

public class MCAPWebSocketDataServerClientHandler extends SimpleChannelInboundHandler<Object>
{
   private final WebSocketClientHandshaker handshaker;
   private final MCAPDataScheduler consumer;

   private final int timestampPort;
   private final ConnectionStateListener connectionStateListener;

   private ChannelPromise handshakeFuture;

   private boolean sendConfiguration = false;

   private volatile boolean waitingForPong = false;

   private DataServerCommandConsumer dataServerCommandConsumer;

   public MCAPWebSocketDataServerClientHandler(WebSocketClientHandshaker handshaker,
                                               int timestampPort,
                                               MCAPDataScheduler consumer,
                                               ConnectionStateListener connectionStateListener)
   {
      this.handshaker = handshaker;
      this.consumer = consumer;
      this.timestampPort = timestampPort;
      this.connectionStateListener = connectionStateListener;
   }

   public void setDataServerCommandConsumer(DataServerCommandConsumer dataServerCommandConsumer)
   {
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
         consumer.onNewData(MCAPDataInput.wrap(byteBuffer));

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

   public interface DataServerCommandConsumer
   {
      void receivedCommand(DataServerCommand command, int argument);
   }
}