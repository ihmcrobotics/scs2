package us.ihmc.robotDataLogger.websocket.client.discovery;

import io.netty.bootstrap.Bootstrap;
import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.channel.Channel;
import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelOption;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.channel.socket.SocketChannel;
import io.netty.handler.codec.http.DefaultFullHttpRequest;
import io.netty.handler.codec.http.HttpClientCodec;
import io.netty.handler.codec.http.HttpContent;
import io.netty.handler.codec.http.HttpContentDecompressor;
import io.netty.handler.codec.http.HttpHeaderNames;
import io.netty.handler.codec.http.HttpHeaderValues;
import io.netty.handler.codec.http.HttpMethod;
import io.netty.handler.codec.http.HttpObject;
import io.netty.handler.codec.http.HttpRequest;
import io.netty.handler.codec.http.HttpResponse;
import io.netty.handler.codec.http.HttpResponseStatus;
import io.netty.handler.codec.http.HttpVersion;
import io.netty.handler.codec.http.LastHttpContent;
import us.ihmc.robotDataLogger.util.NettyUtils;
import us.ihmc.robotDataLogger.websocket.mcap.WebsocketMCAPStarter;
import us.ihmc.robotDataLogger.websocket.server.MCAPDataServerServerContent;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;

import java.io.IOException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Future;
import java.util.function.Consumer;

public class HTTPMCAPDataServerConnection
{
   private static final int TIMEOUT_MS = 1000;

   private final EventLoopGroup group = NettyUtils.createEventGroundLoop();
   private final HTTPDataServerDescription target;
   private final HTTPDataServerConnectionListener listener;
   private WebsocketMCAPStarter mcapStarter;

   private Channel channel;

   private CompletableFuture<ByteBuf> requestFuture;
   private ByteBuf requestedBuffer;

   private boolean taken = false;

   public interface HTTPDataServerConnectionListener
   {
      /**
       * Channel has successfully connected and received a announcement
       *
       * @param connection
       */
      void connected(HTTPMCAPDataServerConnection connection);

      /**
       * Channel has been disconnected. The channel is still cleaning up. closed() will be called when
       * cleanup is finished
       *
       * @param connection
       */
      default void disconnected(HTTPMCAPDataServerConnection connection)
      {

      }

      /**
       * Connection has been refused
       *
       * @param target
       */
      void connectionRefused(HTTPDataServerDescription target);

      /**
       * The channel is closed and all threads have shut down.
       *
       * @param httpDataServerConnection
       */
      default void closed(HTTPMCAPDataServerConnection httpDataServerConnection)
      {

      }
   }

   /**
    * A promise to disconnect later. Used in the logger to avoid a reconnect.
    *
    * @author Jesper Smith
    */
   public static class DisconnectPromise
   {
      private final HTTPDataServerConnectionListener listener;
      private final HTTPMCAPDataServerConnection connection;

      private DisconnectPromise(HTTPDataServerConnectionListener listener, HTTPMCAPDataServerConnection connection)
      {
         this.listener = listener;
         this.connection = connection;
      }

      public void complete()
      {
         listener.disconnected(connection);
         listener.closed(connection);
      }
   }

   /**
    * Connect to a given host
    *
    * @param host IP or hostname
    * @param port port
    * @return A connection if successful
    * @throws IOException if connection failed
    */
   public static HTTPMCAPDataServerConnection connect(String host, int port) throws IOException
   {
      HTTPDataServerDescription target = new HTTPDataServerDescription(host, port, null, false);

      CompletableFuture<HTTPMCAPDataServerConnection> connectionFuture = new CompletableFuture<>();

      new HTTPMCAPDataServerConnection(target, new HTTPMCAPDataServerConnection.HTTPDataServerConnectionListener()
      {
         @Override
         public void connectionRefused(HTTPDataServerDescription target)
         {
            connectionFuture.completeExceptionally(new IOException("Connection refused"));
         }

         @Override
         public void connected(HTTPMCAPDataServerConnection connection)
         {
            connectionFuture.complete(connection);
         }
      });

      try
      {
         return connectionFuture.get();
      }
      catch (Exception e)
      {
         throw new IOException(e);
      }
   }

   /**
    * Setup a new connection If you want a blocking connect call, use the static {@link connect()}
    * function
    *
    * @param target
    * @param listener
    */
   public HTTPMCAPDataServerConnection(HTTPDataServerDescription target, HTTPDataServerConnectionListener listener)
   {
      this.target = target;
      this.listener = listener;

      Bootstrap b = new Bootstrap();
      b.group(group).channel(NettyUtils.getSocketChannelClass()).handler(new HttpSnoopClientInitializer());
      b.option(ChannelOption.CONNECT_TIMEOUT_MILLIS, TIMEOUT_MS);

      ChannelFuture connectFuture = b.connect(target.getHost(), target.getPort());
      connectFuture.addListener((f) ->
                                {
                                   if (f.isSuccess())
                                      connected(((ChannelFuture) f.sync()).channel());
                                   else
                                      group.shutdownGracefully().addListener(e -> listener.connectionRefused(target));
                                });
   }

   private void connected(Channel channel)
   {
      this.channel = channel;
      requestResource(MCAPDataServerServerContent.MCAP_STARTER, buffer ->
      {
         try
         {
            mcapStarter = new WebsocketMCAPStarter(MCAPDataInput.wrap(buffer));
         }
         catch (Exception e)
         {
            e.printStackTrace();
            throw e;
         }
         listener.connected(this);
      });
   }

   public boolean isConnected()
   {
      return channel != null && channel.isActive();
   }

   public Future<ByteBuf> requestResource(String path)
   {
      return requestResource(path, null);
   }

   public Future<ByteBuf> requestResource(String path, Consumer<ByteBuf> action)
   {
      if (requestFuture != null && !requestFuture.isDone())
      {
         throw new RuntimeException("Previous request still pending");
      }

      requestFuture = new CompletableFuture<>();

      if (action != null)
      {
         requestFuture.thenAccept(action);
      }

      if (channel != null)
      {
         // Prepare the HTTP request.
         HttpRequest request = new DefaultFullHttpRequest(HttpVersion.HTTP_1_1, HttpMethod.GET, path);
         request.headers().set(HttpHeaderNames.HOST, target);
         request.headers().set(HttpHeaderNames.CONNECTION, HttpHeaderValues.KEEP_ALIVE);
         request.headers().set(HttpHeaderNames.ACCEPT_ENCODING, HttpHeaderValues.GZIP);
         // Send the HTTP request.
         try
         {
            channel.writeAndFlush(request).syncUninterruptibly();
         }
         catch (Exception e)
         {
            requestFuture.completeExceptionally(e);
            channel.close();
         }
      }
      else
      {
         requestFuture.completeExceptionally(new IOException("Channel not open"));
      }

      return requestFuture;
   }

   /**
    * Close the channel. The DataDiscoveryClient will re-try to connect.
    */
   public void close()
   {
      if (channel != null)
      {
         channel.close();
      }
   }

   /**
    * Take over the connection to start a session. This closes the channel and stops the discovery
    * client from re-trying.
    */
   public DisconnectPromise take()
   {
      taken = true;
      channel.close();

      return new DisconnectPromise(listener, this);
   }

   public HTTPDataServerDescription getTarget()
   {
      return target;
   }

   public WebsocketMCAPStarter getMCAPStarter()
   {
      return mcapStarter;
   }

   private class HttpSnoopClientInitializer extends ChannelInitializer<SocketChannel>
   {

      public HttpSnoopClientInitializer()
      {
      }

      @Override
      public void initChannel(SocketChannel ch)
      {
         ChannelPipeline p = ch.pipeline();

         p.addLast(new HttpClientCodec());
         p.addLast(new HttpContentDecompressor());
         p.addLast(new Handler());
      }
   }

   private class Handler extends SimpleChannelInboundHandler<HttpObject>
   {

      @Override
      protected void channelRead0(ChannelHandlerContext ctx, HttpObject msg) throws Exception
      {
         if (!(msg instanceof HttpResponse) && !(msg instanceof HttpContent))
            return;

         if (requestFuture == null || requestFuture.isDone())
         {
            throw new IOException("HTTP response received without matching request: " + msg.getClass().getSimpleName() + " " + msg);
         }

         if (msg instanceof HttpResponse response)
         {
            if (response.status() == HttpResponseStatus.OK)
            {
               int contentLength = response.headers().getInt("content-length", 0);
               if (contentLength <= 0)
               {
                  requestFuture.completeExceptionally(new IOException("No content-length set."));
                  ctx.close();
                  return;
               }

               requestedBuffer = Unpooled.buffer(contentLength);
            }
            else if (response.status() == HttpResponseStatus.NO_CONTENT)
            {
               requestedBuffer = null;
            }
            else
            {
               requestFuture.completeExceptionally(new IOException("Invalid response received " + response.status()));
               ctx.close();
               return;
            }
         }

         if (msg instanceof HttpContent content)
         {
            if (requestedBuffer == null)
            {
               if (content.content().readableBytes() != 0)
               {
                  requestFuture.completeExceptionally(new IOException("Expected empty content"));
                  ctx.close();
                  return;
               }
            }
            else if (requestedBuffer.isWritable(content.content().readableBytes()))
            {
               requestedBuffer.writeBytes(content.content());
            }
            else
            {
               requestFuture.completeExceptionally(new IOException("Content-length exceeds allocated space"));
               ctx.close();
               return;
            }

            if (content instanceof LastHttpContent)
            {
               requestFuture.complete(requestedBuffer);
               requestedBuffer = null;
            }
         }
      }

      @Override
      public void channelInactive(ChannelHandlerContext ctx) throws Exception
      {
         if (!taken)
         {
            listener.disconnected(HTTPMCAPDataServerConnection.this);
            group.shutdownGracefully().addListener((e) -> listener.closed(HTTPMCAPDataServerConnection.this));
         }
         else
         {
            group.shutdownGracefully();
         }
      }

      @Override
      public void exceptionCaught(ChannelHandlerContext ctx, Throwable cause) throws Exception
      {
         cause.printStackTrace();
         ctx.close();
      }
   }
}
