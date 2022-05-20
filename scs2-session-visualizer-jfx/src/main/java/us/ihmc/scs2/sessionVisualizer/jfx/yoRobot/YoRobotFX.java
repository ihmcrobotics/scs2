package us.ihmc.scs2.sessionVisualizer.jfx.yoRobot;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executor;

import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.collections.FXCollections;
import javafx.collections.MapChangeListener;
import javafx.collections.ObservableMap;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.util.Duration;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem.FrameNode;
import us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem.RigidBodyFrameNodeFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.SimJointAuxiliaryData;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.robot.sensors.SimSensor;
import us.ihmc.scs2.simulation.robot.trackers.KinematicPoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoRobotFX
{
   private final Group rootNode = new Group();
   private final RobotDefinition robotDefinition;
   private final YoManager yoManager;
   private final ReferenceFrameManager referenceFrameManager;

   private SimRigidBodyBasics rootBody;
   private YoRegistry robotRegistry;
   private ObservableMap<String, FrameNode> rigidBodyFrameNodeMap = FXCollections.observableMap(new ConcurrentHashMap<>(64));

   private LinkedYoRegistry robotLinkedYoRegistry;

   private boolean initialize = true;

   public YoRobotFX(YoManager yoManager, ReferenceFrameManager referenceFrameManager, RobotDefinition robotDefinition)
   {
      this.yoManager = yoManager;
      this.referenceFrameManager = referenceFrameManager;
      this.robotDefinition = robotDefinition;
   }

   public void loadRobot(Executor graphicLoader)
   {
      LogTools.info("Loading robot: " + robotDefinition.getName());

      YoRegistry rootRegistry = new YoRegistry(SimulationSession.ROOT_REGISTRY_NAME);
      Robot robot = new Robot(robotDefinition, referenceFrameManager.getWorldFrame(), false);
      robotRegistry = robot.getRegistry();
      rootRegistry.addChild(robotRegistry);
      rootBody = robot.getRootBody();

      rigidBodyFrameNodeMap.addListener((MapChangeListener<String, FrameNode>) change ->
      {
         if (change.wasRemoved())
         {
            FrameNode removedNode = change.getValueRemoved();
            JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> rootNode.getChildren().remove(removedNode.getNode()));
         }
         else if (change.wasAdded())
         {
            FrameNode addedNode = change.getValueAdded();
            Node node = addedNode.getNode();
            node.setScaleX(0.0);
            node.setScaleY(0.0);
            node.setScaleZ(0.0);
            Timeline timeline = new Timeline(new KeyFrame(Duration.seconds(0.25),
                                                          new KeyValue(node.scaleXProperty(), 1.0),
                                                          new KeyValue(node.scaleYProperty(), 1.0),
                                                          new KeyValue(node.scaleZProperty(), 1.0)));
            timeline.setCycleCount(1);

            JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
            {
               addedNode.updatePose();
               timeline.playFromStart();
               rootNode.getChildren().add(addedNode.getNode());
            });
         }
      });
      RigidBodyFrameNodeFactories.createRobotFrameNodeMap(rootBody, robotDefinition, graphicLoader, rigidBodyFrameNodeMap);
      LogTools.info("Loaded robot: " + robotDefinition.getName());

      attachRobotToScene();
   }

   private void attachRobotToScene()
   {
      robotLinkedYoRegistry = yoManager.newLinkedYoRegistry(robotRegistry);
      robotRegistry.getVariables().forEach(var ->
      {
         if (var.getName().startsWith("q_"))
         { // Only link the YoVariables for the joint angles and root joint configuration.
            LinkedYoVariable<YoVariable> linkYoVariable = robotLinkedYoRegistry.linkYoVariable(var);
            linkYoVariable.addUser(this);
         }
      });

      for (SimRigidBodyBasics rigidBody : rootBody.subtreeIterable())
      {
         if (rigidBody.isRootBody())
            continue;

         SimJointAuxiliaryData auxialiryData = rigidBody.getParentJoint().getAuxialiryData();

         auxialiryData.getKinematicPoints().forEach(this::linkKinematicPointFrame);
         auxialiryData.getExternalWrenchPoints().forEach(this::linkKinematicPointFrame);
         auxialiryData.getGroundContactPoints().forEach(this::linkKinematicPointFrame);

         auxialiryData.getIMUSensors().forEach(this::linkSensorFrame);
         auxialiryData.getWrenchSensors().forEach(this::linkSensorFrame);
         auxialiryData.getCameraSensors().forEach(this::linkSensorFrame);
      }
   }

   private void linkKinematicPointFrame(KinematicPoint kp)
   {
      linkYoFrameOffset(kp.getOffset());
   }

   private void linkSensorFrame(SimSensor sensor)
   {
      linkYoFrameOffset(sensor.getOffset());
   }

   private void linkYoFrameOffset(YoFramePoseUsingYawPitchRoll offset)
   {
      YoDouble[] offsetVariables = {offset.getYoX(), offset.getYoY(), offset.getYoZ(), offset.getYoYaw(), offset.getYoPitch(), offset.getYoRoll()};

      for (YoDouble variable : offsetVariables)
      {
         LinkedYoVariable<YoDouble> linkYoVariable = robotLinkedYoRegistry.linkYoVariable(variable);
         linkYoVariable.addUser(this);
      }
   }

   public void render()
   {
      if (rootBody != null && robotLinkedYoRegistry != null)
      {
         if (robotLinkedYoRegistry.pull() || initialize)
         {
            rootBody.updateFramesRecursively();
            rigidBodyFrameNodeMap.values().forEach(node -> node.updatePose());
            initialize = false;
         }
      }
   }

   public RigidBodyReadOnly getRootBody()
   {
      return rootBody;
   }

   public RigidBodyBasics findRigidBody(String rigidBodyName)
   {
      return MultiBodySystemTools.findRigidBody(rootBody, rigidBodyName);
   }

   public FrameNode findRigidBodyFrameNode(String rigidBodyName)
   {
      return rigidBodyFrameNodeMap.get(rigidBodyName);
   }

   public RobotDefinition getRobotDefinition()
   {
      return robotDefinition;
   }

   public Node getRootNode()
   {
      return rootNode;
   }

   public boolean isRobotLoaded()
   {
      return !initialize;
   }
}
