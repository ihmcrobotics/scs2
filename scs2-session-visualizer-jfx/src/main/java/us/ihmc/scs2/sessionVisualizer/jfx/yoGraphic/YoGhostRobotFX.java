package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.binding.Bindings;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableBooleanValue;
import javafx.collections.FXCollections;
import javafx.collections.MapChangeListener;
import javafx.collections.ObservableMap;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Shape3D;
import javafx.util.Duration;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoYawPitchRollDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicRobotDefinition.YoOneDoFJointStateDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicRobotDefinition.YoRobotStateDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem.FrameNode;
import us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem.RigidBodyFrameNodeFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;
import us.ihmc.scs2.sharedMemory.LinkedYoDouble;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleConsumer;

public class YoGhostRobotFX extends YoGraphicFX3D
{ // FIXME Need to handle the color property
   private final Group rootNode = new Group();
   private final YoManager yoManager;

   private final Property<RobotDefinition> robotDefinitionProperty = new SimpleObjectProperty<>(this, "robotDefinition", null);
   private final Property<YoRobotStateDefinition> robotStateDefinitionProperty = new SimpleObjectProperty<>(this, "robotStateDefinition", null);
   private boolean robotDefinitionChanged = true;
   private boolean robotStateDefinitionChanged = true;

   private Robot robot;
   private final ObservableMap<String, FrameNode> rigidBodyFrameNodeMap = FXCollections.observableMap(new ConcurrentHashMap<>(64));

   private final List<LinkedYoVariable<?>> linkedYoVariables = new ArrayList<>();

   private final AtomicBoolean forceUpdate = new AtomicBoolean(true);

   private final List<Runnable> clearStateBindingTasks = new ArrayList<>();

   private boolean initializeMaterial = true;
   private PhongMaterial overridingMaterial = null;
   private final List<Runnable> reverseOverridingMaterialTasks = new ArrayList<>();

   private final BooleanProperty rootJointPoseValid = new SimpleBooleanProperty(this, "rootJointPoseValid", true);

   public YoGhostRobotFX(YoManager yoManager)
   {
      this.yoManager = yoManager;
      setColor((BaseColorFX) null); // Remove the default color.

      drawModeProperty.addListener((o, oldValue, newValue) -> JavaFXMissingTools.setDrawModeRecursive(rootNode, newValue));

      rootJointPoseValid.addListener((o, oldValue, newValue) ->
                                     {
                                        if (newValue)
                                           rootNode.setScaleX(1.0);
                                        else
                                           rootNode.setScaleX(0.0);
                                     });
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
            if (overridingMaterial != null)
               overrideMaterialRecursive(node, overridingMaterial);
            JavaFXMissingTools.setDrawModeRecursive(node, getDrawMode());
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
   }

   public void setRobotDefinition(RobotDefinition robotDefinition)
   {
      if (!robotDefinition.equals(robotDefinitionProperty.getValue()))
         setRobotStateDefinition(null);

      robotDefinitionProperty.setValue(robotDefinition);
      robotDefinitionChanged = true;
   }

   public void setRobotStateDefinition(YoRobotStateDefinition robotStateDefinition)
   {
      robotStateDefinitionProperty.setValue(robotStateDefinition);
      robotStateDefinitionChanged = true;
   }

   private void updateRobotDefinition()
   {
      if (!robotDefinitionChanged)
         return;

      robotDefinitionChanged = false;
      rigidBodyFrameNodeMap.clear();
      reverseOverridingMaterialTasks.clear(); // Dropping the previous tasks as they are no longer valid.

      RobotDefinition newRobotDefinition = robotDefinitionProperty.getValue();

      if (newRobotDefinition == null)
      {
         robot = null;
         return;
      }
      robot = new Robot(newRobotDefinition, ReferenceFrameTools.constructARootFrame("dummy"), false);
      RigidBodyFrameNodeFactories.createRobotFrameNodeMap(robot.getRootBody(), newRobotDefinition, null, rigidBodyFrameNodeMap);
      // Need to update the robot state definition as the robot has changed.
      robotStateDefinitionChanged = true;
   }

   private void updateRobotStateDefinition()
   {
      if (!robotStateDefinitionChanged)
         return;

      // Clear previous bindings
      clearRobotStateBindings();

      if (robot == null)
         return;

      robotStateDefinitionChanged = false;

      YoRobotStateDefinition newRobotStateDefinition = robotStateDefinitionProperty.getValue();
      if (newRobotStateDefinition == null)
         return;

      // We link the robot state to the yoVariables from YoRobotStateDefinition.
      YoTuple3DDefinition rootJointPositionDefinition = newRobotStateDefinition.getRootJointPosition();

      ObservableBooleanValue rootJointPositionValidity = null;

      if (rootJointPositionDefinition != null)
      {
         Pose3DBasics pose = robot.getFloatingRootJoint().getJointPose();

         BooleanProperty xValid = setupBinding(rootJointPositionDefinition.getX(), pose::setX);
         BooleanProperty yValid = setupBinding(rootJointPositionDefinition.getY(), pose::setY);
         BooleanProperty zValid = setupBinding(rootJointPositionDefinition.getZ(), pose::setZ);
         rootJointPositionValidity = xValid.and(yValid).and(zValid);
      }

      YoOrientation3DDefinition rootJointOrientationDefinition = newRobotStateDefinition.getRootJointOrientation();
      ObservableBooleanValue rootJointOrientationValidity = null;

      if (rootJointOrientationDefinition != null)
      {
         QuaternionBasics orientation = robot.getFloatingRootJoint().getJointPose().getOrientation();

         if (rootJointOrientationDefinition instanceof YoYawPitchRollDefinition yawPitchRollDefinition)
         {
            BooleanProperty yawValid = setupBinding(yawPitchRollDefinition.getYaw(),
                                                    newValue -> orientation.setYawPitchRoll(newValue, orientation.getPitch(), orientation.getRoll()));
            BooleanProperty pitchValid = setupBinding(yawPitchRollDefinition.getPitch(),
                                                      newValue -> orientation.setYawPitchRoll(orientation.getYaw(), newValue, orientation.getRoll()));
            BooleanProperty rollValid = setupBinding(yawPitchRollDefinition.getRoll(),
                                                     newValue -> orientation.setYawPitchRoll(orientation.getYaw(), orientation.getPitch(), newValue));
            rootJointOrientationValidity = yawValid.and(pitchValid).and(rollValid);
         }
         else if (rootJointOrientationDefinition instanceof YoQuaternionDefinition quaternionDefinition)
         {
            BooleanProperty xValid = setupBinding(quaternionDefinition.getX(),
                                                  newValue -> orientation.setUnsafe(newValue, orientation.getY(), orientation.getZ(), orientation.getS()));
            BooleanProperty yValid = setupBinding(quaternionDefinition.getY(),
                                                  newValue -> orientation.setUnsafe(orientation.getX(), newValue, orientation.getZ(), orientation.getS()));
            BooleanProperty zValid = setupBinding(quaternionDefinition.getZ(),
                                                  newValue -> orientation.setUnsafe(orientation.getX(), orientation.getY(), newValue, orientation.getS()));
            BooleanProperty sValid = setupBinding(quaternionDefinition.getS(),
                                                  newValue -> orientation.setUnsafe(orientation.getX(), orientation.getY(), orientation.getZ(), newValue));
            rootJointOrientationValidity = xValid.and(yValid).and(zValid).and(sValid);
         }
         else
         {
            LogTools.error("Unsupported root joint orientation: " + rootJointOrientationDefinition);
         }
      }

      if (rootJointPositionValidity != null && rootJointOrientationValidity != null)
         rootJointPoseValid.bind(Bindings.and(rootJointPositionValidity, rootJointOrientationValidity));
      else if (rootJointPositionValidity != null)
         rootJointPoseValid.bind(rootJointPositionValidity);
      else if (rootJointOrientationValidity != null)
         rootJointPoseValid.bind(rootJointOrientationValidity);

      if (newRobotStateDefinition.getJointPositions() != null)
      {
         for (YoOneDoFJointStateDefinition jointPositionDefinition : newRobotStateDefinition.getJointPositions())
         {
            String jointName = jointPositionDefinition.getJointName();
            SimOneDoFJointBasics joint = robot.getOneDoFJoint(jointName);
            if (joint == null)
               LogTools.error("Could not find joint: " + jointName);
            else
               setupBinding(jointPositionDefinition.getJointPosition(), joint::setQ);
         }
      }

      forceUpdate.set(true);
   }

   private BooleanProperty setupBinding(String variableName, DoubleConsumer setter)
   {
      DoubleProperty doubleProperty = toDoubleProperty(variableName);
      BooleanProperty validityProperty = new SimpleBooleanProperty(this, variableName + " - valid", true);
      ChangeListener<Number> changeListener = (o, oldValue, newValue) ->
      {
         boolean isValid = newValue != null && Double.isFinite(newValue.doubleValue());
         validityProperty.set(isValid);
         if (isValid)
            setter.accept(newValue.doubleValue());
         forceUpdate.set(true);
      };
      doubleProperty.addListener(changeListener);
      // Trigger the change listener once to set the initial value.
      changeListener.changed(doubleProperty, null, doubleProperty.getValue());
      clearStateBindingTasks.add(() -> doubleProperty.removeListener(changeListener));
      return validityProperty;
   }

   private DoubleProperty toDoubleProperty(String variableName)
   {
      if (variableName == null)
         return new SimpleDoubleProperty(this, "dummy", 0.0);
      DoubleProperty doubleProperty = CompositePropertyTools.toDoubleProperty(yoManager.getRootRegistryDatabase(),
                                                                              yoManager::newLinkedYoVariable,
                                                                              variableName);
      if (doubleProperty instanceof YoDoubleProperty yoDoubleProperty)
      {
         LinkedYoDouble linkedYoDouble = yoDoubleProperty.getLinkedBuffer();
         linkedYoVariables.add(linkedYoDouble);
         clearStateBindingTasks.add(() ->
                                    {
                                       yoDoubleProperty.dispose();
                                       linkedYoDouble.dispose();
                                    });
      }
      return doubleProperty;
   }

   @Override
   public void computeBackground()
   {
      updateRobotDefinition();
      updateRobotStateDefinition();
   }

   @Override
   public void render()
   {
      if (robot == null)
         return;

      boolean updateRobot = false;
      for (int i = 0; i < linkedYoVariables.size(); i++)
      {
         updateRobot |= linkedYoVariables.get(i).pull();
      }

      if (!updateRobot)
      {
         updateRobot = forceUpdate.getAndSet(false);
      }

      if (updateRobot)
      {
         robot.getRootBody().updateFramesRecursively();
         rigidBodyFrameNodeMap.values().forEach(FrameNode::updatePose);
      }

      if (initializeMaterial || (getColor() == null) != (overridingMaterial == null))
      {
         initializeMaterial = false;

         if (getColor() == null)
         {
            reverseOverridingMaterialTasks.forEach(Runnable::run);
            reverseOverridingMaterialTasks.clear();
            overridingMaterial = null;
         }
         else
         {
            overridingMaterial = new PhongMaterial();
            overrideMaterialRecursive(rootNode, overridingMaterial);
         }
      }

      if (getColor() != null)
      {
         overridingMaterial.setDiffuseColor(getColor().get());
      }
   }

   private void overrideMaterialRecursive(Node start, PhongMaterial material)
   {
      if (start instanceof Group group)
      {
         group.getChildren().forEach(child -> overrideMaterialRecursive(child, material));
      }
      else if (start instanceof Shape3D shape)
      {
         Material originalMaterial = shape.getMaterial();
         reverseOverridingMaterialTasks.add(() -> shape.setMaterial(originalMaterial));
         shape.setMaterial(material);
      }
   }

   public RobotDefinition getRobotDefinition()
   {
      return robotDefinitionProperty.getValue();
   }

   public YoRobotStateDefinition getRobotStateDefinition()
   {
      return robotStateDefinitionProperty.getValue();
   }

   @Override
   public void clear()
   {
      robot = null;
      rigidBodyFrameNodeMap.clear();
      rootNode.getChildren().clear();
      clearRobotStateBindings();
   }

   private void clearRobotStateBindings()
   {
      clearStateBindingTasks.forEach(Runnable::run);
      clearStateBindingTasks.clear();
      linkedYoVariables.clear();
   }

   @Override
   public YoGraphicFX clone()
   {
      YoGhostRobotFX clone = new YoGhostRobotFX(yoManager);
      clone.setName(getName());
      clone.setColor(getColor());
      clone.setVisible(isVisible());
      clone.setRobotDefinition(robotDefinitionProperty.getValue());
      clone.setRobotStateDefinition(robotStateDefinitionProperty.getValue());
      clone.setDrawMode(getDrawMode());
      return clone;
   }

   @Override
   public Node getNode()
   {
      return rootNode;
   }
}
