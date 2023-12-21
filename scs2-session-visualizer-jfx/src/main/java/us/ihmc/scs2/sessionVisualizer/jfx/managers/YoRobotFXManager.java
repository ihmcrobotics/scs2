package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionRobotDefinitionListChange;
import us.ihmc.scs2.sessionVisualizer.jfx.Camera3DRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.Camera3DRequest.FocalPointRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem.FrameNode;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.YoRobotFX;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

import java.util.*;
import java.util.stream.Collectors;

public class YoRobotFXManager extends ObservedAnimationTimer implements Manager
{
   private final Group rootNode = new Group();
   private final List<YoRobotFX> robots = new ArrayList<>();
   private final JavaFXMessager messager;
   private final SessionVisualizerTopics topics;
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
      this.messager = messager;
      this.topics = topics;
      this.yoManager = yoManager;
      this.referenceFrameManager = referenceFrameManager;
      this.backgroundExecutorManager = backgroundExecutorManager;

      messager.addTopicListener(topics.getCamera3DRequest(), request ->
      {
         if (!isSessionLoaded())
            throw new IllegalOperationException("Session has not been loaded yet.");

         FocalPointRequest focalPointRequest = request.getFocalPointRequest();
         if (focalPointRequest == null)
            return;

         String rigidBodyName = focalPointRequest.getRigidBodyName();
         String robotName = focalPointRequest.getRobotName();

         if (rigidBodyName != null)
         {
            Optional<FrameNode> result;

            if (robotName != null)
            {
               result = robots.stream()
                              .filter(r -> r.getRobotDefinition().getName().equalsIgnoreCase(robotName))
                              .findFirst()
                              .map(r -> r.findRigidBodyFrameNode(rigidBodyName));
            }
            else
            {
               result = robots.stream().map(r -> r.findRigidBodyFrameNode(rigidBodyName)).filter(Objects::nonNull).findFirst();
            }

            result.ifPresent(rigidBodyFrameNode ->
                             {
                                if (rigidBodyFrameNode.getNode() != null)
                                   messager.submitMessage(topics.getCamera3DRequest(),
                                                          new Camera3DRequest(FocalPointRequest.trackNode(rigidBodyFrameNode.getNode())));
                             });
         }
      });

      messager.addTopicListener(topics.getRobotVisualRequest(), this::handleRobotVisualRequest);
      messager.addFXTopicListener(topics.getSessionRobotDefinitionListChangeState(), this::handleSessionRobotDefinitionListChangeState);
   }

   private void handleSessionRobotDefinitionListChangeState(SessionRobotDefinitionListChange change)
   {
      if (change.getAddedRobotDefinition() == null)
      {
         LogTools.warn("Received request but robot definition is null, ignoring.");
         return;
      }

      switch (change.getChangeType())
      {
         case ADD:
         {
            addRobotDefinition(change.getAddedRobotDefinition());
            break;
         }
         case REMOVE:
         {
            removeRobotDefinition(change.getRemovedRobotDefinition());
            break;
         }
         case REPLACE:
         {
            removeRobotDefinition(change.getRemovedRobotDefinition());
            addRobotDefinition(change.getAddedRobotDefinition());
            break;
         }
      }
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
         if (robots.size() == 1)
            messager.submitMessage(topics.getCamera3DRequest(), new Camera3DRequest(FocalPointRequest.trackNode(robot.getRootNode())));
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
