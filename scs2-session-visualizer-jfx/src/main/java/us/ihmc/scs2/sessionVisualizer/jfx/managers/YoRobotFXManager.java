package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.stream.Collectors;

import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.CameraObjectTrackingRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem.FrameNode;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.YoRobotFX;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

public class YoRobotFXManager extends ObservedAnimationTimer implements Manager
{
   private final Group rootNode = new Group();
   private final List<YoRobotFX> robots = new ArrayList<>();
   private final YoManager yoManager;
   private final ReferenceFrameManager referenceFrameManager;
   private final BackgroundExecutorManager backgroundExecutorManager;

   private int numberOfRobotDefinitions = -1;

   public YoRobotFXManager(JavaFXMessager messager,
                           SessionVisualizerTopics topics,
                           YoManager yoManager,
                           ReferenceFrameManager referenceFrameManager,
                           BackgroundExecutorManager backgroundExecutorManager)
   {
      this.yoManager = yoManager;
      this.referenceFrameManager = referenceFrameManager;
      this.backgroundExecutorManager = backgroundExecutorManager;

      messager.registerTopicListener(topics.getCameraTrackObject(), request ->
      {
         if (!isSessionLoaded())
            throw new IllegalOperationException("Session has not been loaded yet.");

         String rigidBodyName = request.getRigidBodyName();
         String robotName = request.getRobotName();

         if (rigidBodyName != null)
         {
            Optional<FrameNode> result;

            if (robotName != null)
            {
               result = robots.stream().filter(r -> r.getRobotDefinition().getName().equalsIgnoreCase(robotName)).findFirst()
                              .map(r -> r.findRigidBodyFrameNode(rigidBodyName));
            }
            else
            {
               result = robots.stream().map(r -> r.findRigidBodyFrameNode(rigidBodyName)).filter(Objects::nonNull).findFirst();
            }

            result.ifPresent(rigidBodyFrameNode ->
            {
               if (rigidBodyFrameNode != null && rigidBodyFrameNode.getNode() != null)
                  messager.submitMessage(topics.getCameraTrackObject(), new CameraObjectTrackingRequest(rigidBodyFrameNode.getNode()));
            });
         }
      });

      messager.registerTopicListener(topics.getRobotVisualRequest(), this::handleRobotVisualRequest);
   }

   private void handleRobotVisualRequest(NewRobotVisualRequest request)
   {
      String robotName = request.getRobotName();
      if (robotName == null)
      {
         LogTools.warn("Received request but robot name is null, ignoring.");
         return;
      }

      if (robotName.equals(NewRobotVisualRequest.ALL_ROBOTS))
      {
         if (request.getRequestedVisible() != null)
            robots.forEach(robot -> robot.getRootNode().setVisible(request.getRequestedVisible()));
         if (request.getRequestedDrawMode() != null)
            robots.forEach(robot -> robot.setDrawMode(request.getRequestedDrawMode()));
      }
      else
      {
         YoRobotFX robot = robots.stream().filter(r -> r.getRobotDefinition().getName().equalsIgnoreCase(robotName)).findFirst().orElse(null);

         if (robot == null)
         {
            LogTools.warn("Could not find robot named: {}, ignoring request.", robotName);
            return;
         }

         if (request.getRequestedVisible() != null)
            robot.getRootNode().setVisible(request.getRequestedVisible());
         if (request.getRequestedDrawMode() != null)
            robot.setDrawMode(request.getRequestedDrawMode());
      }
   }

   public void addRobotDefinition(RobotDefinition robotDefinition)
   {
      YoRobotFX robot = new YoRobotFX(yoManager, referenceFrameManager, robotDefinition);
      robot.loadRobot(command -> backgroundExecutorManager.queueTaskToExecuteInBackground(this, command));
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         robots.add(robot);
         rootNode.getChildren().add(robot.getRootNode());
      });
   }

   public void addRobotDefinitions(Collection<? extends RobotDefinition> robotDefinitions)
   {
      robotDefinitions.forEach(robotDefinition -> addRobotDefinition(robotDefinition));
   }

   public void removeRobotDefinition(RobotDefinition robotDefinition)
   {
      Optional<YoRobotFX> result = robots.stream().filter(robotFX -> robotFX.getRobotDefinition() == robotDefinition).findFirst();

      if (result.isPresent())
      {
         YoRobotFX robotToRemove = result.get();
         JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
         {
            rootNode.getChildren().remove(robotToRemove.getRootNode());
            robots.remove(robotToRemove);
         });
      }
   }

   public void removeRobotDefinitions(Collection<? extends RobotDefinition> robotDefinitions)
   {
      robotDefinitions.forEach(this::removeRobotDefinition);
   }

   public void removeAllRobotDefinitions()
   {
      List<Node> nodesToDetach = robots.stream().map(YoRobotFX::getRootNode).collect(Collectors.toList());
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         rootNode.getChildren().removeAll(nodesToDetach);
         robots.clear();
      });
   }

   @Override
   public void handleImpl(long now)
   {
      robots.forEach(YoRobotFX::render);
   }

   @Override
   public void startSession(Session session)
   {
      start();
      List<RobotDefinition> robotDefinitions = session.getRobotDefinitions();
      numberOfRobotDefinitions = robotDefinitions.size();
      addRobotDefinitions(robotDefinitions);
   }

   @Override
   public void stopSession()
   {
      removeAllRobotDefinitions();
      numberOfRobotDefinitions = -1;
   }

   @Override
   public boolean isSessionLoaded()
   {
      if (robots.size() < numberOfRobotDefinitions)
         return false;
      return robots.stream().allMatch(YoRobotFX::isRobotLoaded);
   }

   public RigidBodyReadOnly getRobotRootBody(String robotName)
   {
      return robots.stream().filter(robot -> robot.getRobotDefinition().getName().equals(robotName)).findFirst().map(YoRobotFX::getRootBody).orElse(null);
   }

   public Group getRootNode()
   {
      return rootNode;
   }
}
