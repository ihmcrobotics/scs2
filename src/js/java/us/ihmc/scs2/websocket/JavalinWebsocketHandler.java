package us.ihmc.scs2.websocket;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

import org.eclipse.jetty.websocket.api.Session;

import io.javalin.websocket.WsBinaryMessageContext;
import io.javalin.websocket.WsCloseContext;
import io.javalin.websocket.WsConnectContext;
import io.javalin.websocket.WsErrorContext;
import io.javalin.websocket.WsHandler;
import io.javalin.websocket.WsMessageContext;

public class JavalinWebsocketHandler
{
   private final ConcurrentLinkedQueue<Session> activeSessions = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<Consumer<WsConnectContext>> onConnectHandlers = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<Consumer<WsCloseContext>> onCloseHandlers = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<Consumer<WsMessageContext>> onMessageHandlers = new ConcurrentLinkedQueue<>();

   public JavalinWebsocketHandler()
   {
   }

   public Consumer<WsHandler> setupAsWsHandlerConsumer()
   {
      return ws ->
      {
         ws.onConnect(this::handleConnect);
         ws.onMessage(this::handleMessage);
         ws.onBinaryMessage(this::handleBinaryMessage);
         ws.onClose(this::handleClose);
         ws.onError(this::handleError);
      };
   }

   public void addOnConnect(Consumer<WsConnectContext> handler)
   {
      onConnectHandlers.add(handler);
   }

   public void removeOnConnect(Consumer<WsConnectContext> handler)
   {
      onConnectHandlers.remove(handler);
   }

   public void addOnClose(Consumer<WsCloseContext> handler)
   {
      onCloseHandlers.add(handler);
   }

   public void removeOnClose(Consumer<WsCloseContext> handler)
   {
      onCloseHandlers.remove(handler);
   }

   public void addOnMessage(Consumer<WsMessageContext> handler)
   {
      onMessageHandlers.add(handler);
   }

   public void removeOnMessage(Consumer<WsMessageContext> handler)
   {
      onMessageHandlers.remove(handler);
   }

   public ConcurrentLinkedQueue<Session> getActiveSessions()
   {
      return activeSessions;
   }

   private void handleConnect(WsConnectContext context)
   {
      activeSessions.add(context.session);
      onConnectHandlers.forEach(handler -> handler.accept(context));
   }

   private void handleMessage(WsMessageContext context)
   {
      onMessageHandlers.forEach(handler -> handler.accept(context));
   }

   private void handleBinaryMessage(WsBinaryMessageContext context)
   {

   }

   private void handleClose(WsCloseContext context)
   {
      activeSessions.remove(context.session);
      onCloseHandlers.forEach(handler -> handler.accept(context));
   }

   private void handleError(WsErrorContext context)
   {
   }
}
