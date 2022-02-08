package us.ihmc.scs2.simulation.robot.sensors;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SimCameraSensor extends SimSensor
{
   private final YoBoolean enable;

   private final YoDouble fieldOfView;
   private final YoDouble clipNear;
   private final YoDouble clipFar;

   private final YoInteger imageWidth;
   private final YoInteger imageHeight;

   private final List<CameraFrameConsumer> cameraFrameConsumers = new ArrayList<>();

   private final AtomicBoolean notifyDefinitionConsumers = new AtomicBoolean(false);
   private final List<CameraDefinitionConsumer> cameraDefinitionConsumers = new ArrayList<>();

   public SimCameraSensor(CameraSensorDefinition definition, SimJointBasics parentJoint)
   {
      this(definition.getName(), parentJoint, definition.getTransformToJoint());
      setEnable(definition.getEnable());
      setResolution(definition.getImageWidth(), definition.getImageHeight());
      setFieldOfView(definition.getFieldOfView());
      setClip(definition.getClipNear(), definition.getClipFar());
      setSamplingRate(toSamplingRate(definition.getUpdatePeriod()));
   }

   public SimCameraSensor(String name, SimJointBasics parentJoint, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, parentJoint, transformToParent);

      YoRegistry registry = parentJoint.getRegistry();
      enable = new YoBoolean(name + "Enable", registry);
      fieldOfView = new YoDouble(name + "FieldOfView", registry);
      clipNear = new YoDouble(name + "ClipNear", registry);
      clipFar = new YoDouble(name + "ClipFar", registry);
      imageWidth = new YoInteger(name + "ImageWidth", registry);
      imageHeight = new YoInteger(name + "ImageHeight", registry);

      enable.addListener(v -> notifyDefinitionConsumers.set(true));
      fieldOfView.addListener(v -> notifyDefinitionConsumers.set(true));
      clipNear.addListener(v -> notifyDefinitionConsumers.set(true));
      clipFar.addListener(v -> notifyDefinitionConsumers.set(true));
      imageWidth.addListener(v -> notifyDefinitionConsumers.set(true));
      imageHeight.addListener(v -> notifyDefinitionConsumers.set(true));
      getOffset().attachVariableChangedListener(v -> notifyDefinitionConsumers.set(true));
      getSamplingRate().addListener(v -> notifyDefinitionConsumers.set(true));
   }

   @Override
   public void update(RobotPhysicsOutput robotPhysicsOutput)
   {
      super.update(robotPhysicsOutput);

      if (notifyDefinitionConsumers.getAndSet(false))
      {
         CameraSensorDefinition newDefinition = toCameraSensorDefinition();
         for (CameraDefinitionConsumer cameraDefinitionConsumer : cameraDefinitionConsumers)
         {
            cameraDefinitionConsumer.nextDefinition(newDefinition);
         }
      }
   }

   public CameraSensorDefinition toCameraSensorDefinition()
   {
      CameraSensorDefinition newDefinition = new CameraSensorDefinition(getName(), getFrame().getTransformToParent());
      newDefinition.setEnable(enable.getValue());
      newDefinition.setFieldOfView(fieldOfView.getValue());
      newDefinition.setClipNear(clipNear.getValue());
      newDefinition.setClipFar(clipFar.getValue());
      newDefinition.setImageWidth(imageWidth.getValue());
      newDefinition.setImageHeight(imageHeight.getValue());
      if (getSamplingRate().getValue() == Double.POSITIVE_INFINITY)
         newDefinition.setUpdatePeriod(0);
      else
         newDefinition.setUpdatePeriod((int) (1000.0 / getSamplingRate().getValue()));
      return newDefinition;
   }

   public void setEnable(boolean enable)
   {
      this.enable.set(enable);
   }

   public void setImageWidth(int imageWidth)
   {
      this.imageWidth.set(imageWidth);
   }

   public void setImageHeight(int imageHeight)
   {
      this.imageHeight.set(imageHeight);
   }

   public void setResolution(int width, int height)
   {
      imageWidth.set(width);
      imageHeight.set(height);
   }

   public void setFieldOfView(double fieldOfView)
   {
      this.fieldOfView.set(fieldOfView);
   }

   public void setClipNear(double clipNear)
   {
      this.clipNear.set(clipNear);
   }

   public void setClipFar(double clipFar)
   {
      this.clipFar.set(clipFar);
   }

   public void setClip(double clipNear, double clipFar)
   {
      this.clipNear.set(clipNear);
      this.clipFar.set(clipFar);
   }

   public void addCameraFrameConsumer(CameraFrameConsumer cameraFrameConsumer)
   {
      cameraFrameConsumers.add(cameraFrameConsumer);
   }

   public boolean removeCameraFrameConsumer(CameraFrameConsumer cameraFrameConsumer)
   {
      return cameraFrameConsumers.remove(cameraFrameConsumer);
   }

   public void addCameraDefinitionConsumer(CameraDefinitionConsumer cameraDefinitionConsumer)
   {
      cameraDefinitionConsumers.add(cameraDefinitionConsumer);
   }

   public boolean removeCameraDefinitionConsumer(CameraDefinitionConsumer cameraDefinitionConsumer)
   {
      return cameraDefinitionConsumers.remove(cameraDefinitionConsumer);
   }

   public YoBoolean getEnable()
   {
      return enable;
   }

   public YoInteger getImageWidth()
   {
      return imageWidth;
   }

   public YoInteger getImageHeight()
   {
      return imageHeight;
   }

   public YoDouble getFieldOfView()
   {
      return fieldOfView;
   }

   public YoDouble getClipNear()
   {
      return clipNear;
   }

   public YoDouble getClipFar()
   {
      return clipFar;
   }

   public List<CameraFrameConsumer> getCameraFrameConsumers()
   {
      return cameraFrameConsumers;
   }

   public List<CameraDefinitionConsumer> getCameraDefinitionConsumers()
   {
      return cameraDefinitionConsumers;
   }

   public interface CameraFrameConsumer
   {
      void nextFrame(long timestamp, BufferedImage frame);
   }

   public interface CameraDefinitionConsumer
   {
      void nextDefinition(CameraSensorDefinition definition);
   }
}
