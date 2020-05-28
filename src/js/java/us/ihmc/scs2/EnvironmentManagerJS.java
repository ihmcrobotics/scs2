package us.ihmc.scs2;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import javafx.scene.SubScene;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.protobuf.ThreeProto;
import us.ihmc.scs2.protobuf.ThreeProto.MeshGroup;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.managers.BackgroundExecutorManager;
import us.ihmc.scs2.websocket.JavalinManager;
import us.ihmc.scs2.websocket.JavalinWebsocketHandler;

public class EnvironmentManagerJS implements Manager
{
   private final List<ThreeProto.MeshGroup> environmentMeshGroups = new ArrayList<>();
   private final ConcurrentLinkedQueue<org.eclipse.jetty.websocket.api.Session> activeWebSocketSessions = new ConcurrentLinkedQueue<>();

   public EnvironmentManagerJS(JavalinManager javalinManager, BackgroundExecutorManager backgroundExecutorManager)
   {
      JavalinWebsocketHandler handler = javalinManager.webSocket("/viewport");
      handler.getActiveSessions().forEach(this::onSessionOpen);
      handler.addOnConnect(ctx -> onSessionOpen(ctx.session));
      handler.addOnClose(ctx -> onSessionClose(ctx.session));
   }

   private void onSessionOpen(org.eclipse.jetty.websocket.api.Session session)
   {
      for (ThreeProto.MeshGroup meshGroup : environmentMeshGroups)
      {
         session.getRemote().sendBytesByFuture(meshGroup.toByteString().asReadOnlyByteBuffer());
      }
      activeWebSocketSessions.add(session);
   }

   private void onSessionClose(org.eclipse.jetty.websocket.api.Session session)
   {
      activeWebSocketSessions.remove(session);
   }

   public void addWorldCoordinateSystem(double size)
   {
      // TODO Implement me
   }

   public void addSkybox(SubScene subScene)
   {
      // TODO Implement me
   }

   private void broadcastMesh(MeshGroup meshGroup)
   {
      ByteBuffer byteBuffer = meshGroup.toByteString().asReadOnlyByteBuffer();

      for (org.eclipse.jetty.websocket.api.Session session : activeWebSocketSessions)
      {
         session.getRemote().sendBytesByFuture(byteBuffer);
      }
   }

   @Override
   public void startSession(Session session)
   {
      List<TerrainObjectDefinition> terrainObjectDefinitions = session.getTerrainObjectDefinitions();
      for (TerrainObjectDefinition definition : terrainObjectDefinitions)
      {
         MeshGroup meshGroup = ThreeProtoTools.toProtoMeshGroup(definition.getName(), definition.getVisualDefinitions());
         broadcastMesh(meshGroup);
         environmentMeshGroups.add(meshGroup);
      }
   }

   @Override
   public void stopSession()
   {
      // TODO Implement me
   }

   @Override
   public boolean isSessionLoaded()
   {
      return true;
   }
}
