package us.ihmc.robotDataLogger.websocket.server;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelFutureListener;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.handler.codec.http.DefaultFullHttpResponse;
import io.netty.handler.codec.http.FullHttpRequest;
import io.netty.handler.codec.http.FullHttpResponse;
import io.netty.handler.codec.http.HttpHeaderNames;
import io.netty.handler.codec.http.HttpUtil;
import io.netty.util.CharsetUtil;

import static io.netty.handler.codec.http.HttpMethod.GET;
import static io.netty.handler.codec.http.HttpResponseStatus.*;
import static io.netty.handler.codec.http.HttpVersion.HTTP_1_1;

public class HTTPMCAPDataServerDescriptionServer extends SimpleChannelInboundHandler<FullHttpRequest>
{

   private final MCAPDataServerServerContent serverContent;

   public HTTPMCAPDataServerDescriptionServer(MCAPDataServerServerContent serverContent)
   {

      this.serverContent = serverContent;
   }

   @Override
   protected void channelRead0(ChannelHandlerContext ctx, FullHttpRequest req) throws Exception
   {
      // Handle a bad request.
      if (!req.decoderResult().isSuccess())
      {
         sendHttpResponse(ctx, req, new DefaultFullHttpResponse(HTTP_1_1, BAD_REQUEST));
         return;
      }

      // Allow only GET methods.
      if (req.method() != GET)
      {
         sendHttpResponse(ctx, req, new DefaultFullHttpResponse(HTTP_1_1, FORBIDDEN));
         return;
      }

      if (MCAPDataServerServerContent.MCAP_STARTER.equals(req.uri()))
         sendContent(ctx, req, serverContent.getMCAPStarterBuffer(), "application/mcap");
      else if (MCAPDataServerServerContent.ROBOT_MODEL_RESOURCES.equals(req.uri()))
         sendContent(ctx, req, serverContent.getRobotModelResourcesBuffer(), "application/zip");
      else
         sendHttpResponse(ctx, req, new DefaultFullHttpResponse(HTTP_1_1, NOT_FOUND));
   }

   private static void sendContent(ChannelHandlerContext ctx, FullHttpRequest req, ByteBuf content, String contentType)
   {
      FullHttpResponse res;
      if (content != null)
      {
         res = new DefaultFullHttpResponse(HTTP_1_1, OK, content);
         HttpUtil.setContentLength(res, content.readableBytes());
      }
      else
      {
         res = new DefaultFullHttpResponse(HTTP_1_1, NO_CONTENT);
         HttpUtil.setContentLength(res, 0);
      }

      res.headers().set(HttpHeaderNames.CONTENT_TYPE, contentType);
      sendHttpResponse(ctx, req, res);
   }

   private static void sendHttpResponse(ChannelHandlerContext ctx, FullHttpRequest req, FullHttpResponse res)
   {
      // Generate an error page if response getStatus code is not OK (200) nor NO_CONTENT (204).
      boolean isNotOKNorNoContent = res.status().code() != OK.code() && res.status().code() != NO_CONTENT.code();
      if (isNotOKNorNoContent)
      {
         ByteBuf buf = Unpooled.copiedBuffer(res.status().toString(), CharsetUtil.UTF_8);
         res.content().writeBytes(buf);
         buf.release();
         HttpUtil.setContentLength(res, res.content().readableBytes());
      }

      // Send the response and close the connection if necessary.
      ChannelFuture f = ctx.channel().writeAndFlush(res);
      if (!HttpUtil.isKeepAlive(req) || isNotOKNorNoContent)
      {
         f.addListener(ChannelFutureListener.CLOSE);
      }
   }
}