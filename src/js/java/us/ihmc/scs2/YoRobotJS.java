package us.ihmc.scs2;

import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.eclipse.jetty.websocket.api.Session;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.sessionVisualizer.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.managers.YoManager;
import us.ihmc.scs2.sharedMemory.LinkedYoVariableRegistry;
import us.ihmc.scs2.sharedMemory.tools.YoMirroredRegistryTools;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.websocket.JavalinManager;
import us.ihmc.scs2.websocket.JavalinWebsocketHandler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoRobotJS
{
   private final RobotDefinition robotDefinition;
   private final RigidBodyJS rootBody;
   private final YoVariableRegistry robotRegistry;
   private final LinkedYoVariableRegistry robotLinkedYoVariableRegistry;

   private final ConcurrentLinkedQueue<Session> activeWebSocketSessions = new ConcurrentLinkedQueue<>();

   public YoRobotJS(YoManager yoManager, JavalinManager javalinManager, ReferenceFrameManager referenceFrameManager, RobotDefinition robotDefinition)
   {
      this.robotDefinition = robotDefinition;

      robotRegistry = YoMirroredRegistryTools.newRegistryFromNameSpace(SimulationSession.ROOT_REGISTRY_NAME, robotDefinition.getName());

      ReferenceFrame worldFrame = referenceFrameManager.getWorldFrame();

      rootBody = MultiBodySystemFactoriesJS.toYoMultiBodySystemJS(robotDefinition.newIntance(ReferenceFrameTools.constructARootFrame("dummy")),
                                                                  worldFrame,
                                                                  robotDefinition,
                                                                  robotRegistry);

      robotLinkedYoVariableRegistry = yoManager.newLinkedYoVariableRegistry(robotRegistry);
      yoManager.linkNewYoVariables();

      JavalinWebsocketHandler handler = javalinManager.webSocket("/viewport");
      handler.addOnConnect(ctx -> onSessionOpen(ctx.session));
      handler.addOnClose(ctx -> onSessionClose(ctx.session));
   }

   private void onSessionOpen(Session session)
   {
      for (RigidBodyJS body : rootBody.subtreeIterable())
      {
         ByteBuffer asReadOnlyByteBuffer = body.getGraphics().getFullMeshGroup().toByteString().asReadOnlyByteBuffer();
         session.getRemote().sendBytesByFuture(asReadOnlyByteBuffer);
      }

      activeWebSocketSessions.add(session);
   }

   private void onSessionClose(Session session)
   {
      activeWebSocketSessions.remove(session);
   }

   public void update()
   {
      if (rootBody != null && robotLinkedYoVariableRegistry != null)
      {
         if (robotLinkedYoVariableRegistry.pull())
         {
            rootBody.updateFramesRecursively();
            rootBody.updateSubtreeGraphics();

            for (RigidBodyJS body : rootBody.subtreeIterable())
            {
               ByteBuffer asReadOnlyByteBuffer = body.getGraphics().getLightMeshGroup().toByteString().asReadOnlyByteBuffer();
               activeWebSocketSessions.forEach(session -> session.getRemote().sendBytesByFuture(asReadOnlyByteBuffer));
            }
         }
      }
   }

   public void detachFromScene()
   {
      // TODO
   }

   public RobotDefinition getRobotDefinition()
   {
      return robotDefinition;
   }
}
