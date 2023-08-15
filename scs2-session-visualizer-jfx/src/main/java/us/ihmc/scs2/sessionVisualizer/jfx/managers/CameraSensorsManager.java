package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

import javafx.embed.swing.SwingFXUtils;
import javafx.geometry.Rectangle2D;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.SnapshotParameters;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMessagerAPI.Sensors.SensorMessage;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.SCS2JavaFXMessager;

public class CameraSensorsManager extends ObservedAnimationTimer implements Manager
{
   private final Node mainSceneRoot;
   private final YoRobotFXManager robotFXManager;

   private final List<SingleCameraSensorManager> cameras = new ArrayList<>();
   private final Map<String, Map<String, SingleCameraSensorManager>> robotNameToSensorNameToManagerMap = new HashMap<>();
   private boolean sessionLoaded = false;

   private final JavaFXMessager messager;
   private final SessionVisualizerTopics topics;

   public CameraSensorsManager(Node mainSceneRoot, SCS2JavaFXMessager messager, SessionVisualizerTopics topics, YoRobotFXManager robotFXManager)
   {
      this.mainSceneRoot = mainSceneRoot;
      this.messager = messager;
      this.topics = topics;
      this.robotFXManager = robotFXManager;
      messager.addTopicListener(topics.getCameraSensorDefinitionData(), this::handleCameraSensorDefinitionMessage);
   }

   private void handleCameraSensorDefinitionMessage(SensorMessage<CameraSensorDefinition> message)
   {
      SingleCameraSensorManager manager = robotNameToSensorNameToManagerMap.getOrDefault(message.getRobotName(), Collections.emptyMap())
                                                                           .get(message.getSensorName());

      if (manager == null)
         return;

      manager.configure(message.getMessageContent());
   }

   @Override
   public void handleImpl(long now)
   {
      for (int i = 0; i < cameras.size(); i++)
      {
         cameras.get(i).update(now);
      }
   }

   @Override
   public void startSession(Session session)
   {
      sessionLoaded = false;

      for (RobotDefinition robotDefinition : session.getRobotDefinitions())
      {
         String robotName = robotDefinition.getName();
         Map<String, SingleCameraSensorManager> robotMap = new HashMap<>();

         for (JointDefinition jointDefinition : robotDefinition.getAllJoints())
         {
            List<CameraSensorDefinition> definitions = jointDefinition.getSensorDefinitions(CameraSensorDefinition.class);

            for (CameraSensorDefinition definition : definitions)
            {
               String sensorName = definition.getName();
               RigidBodyReadOnly rootBody = robotFXManager.getRobotRootBody(robotName);
               Objects.requireNonNull(rootBody);
               JointReadOnly parentJoint = MultiBodySystemTools.findJoint(rootBody, jointDefinition.getName());
               Objects.requireNonNull(parentJoint);
               ReferenceFrame sensorFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(sensorName,
                                                                                                              parentJoint.getFrameAfterJoint(),
                                                                                                              definition.getTransformToJoint());
               SingleCameraSensorManager manager = new SingleCameraSensorManager(robotName, definition, sensorFrame);
               cameras.add(manager);
               robotMap.put(sensorName, manager);
            }
         }

         if (!robotMap.isEmpty())
            robotNameToSensorNameToManagerMap.put(robotName, robotMap);
      }

      start();
      sessionLoaded = true;
   }

   @Override
   public void stopSession()
   {
      stop();
      cameras.clear();
      robotNameToSensorNameToManagerMap.clear();
      sessionLoaded = false;
   }

   @Override
   public boolean isSessionLoaded()
   {
      return sessionLoaded;
   }

   private class SingleCameraSensorManager
   {
      private final String robotName;
      private final String sensorName;
      private final ReferenceFrame sensorFrame;
      private final RotationMatrix cameraViewOrientationOffset = new RotationMatrix();
      private final RigidBodyTransform actualCameraPose = new RigidBodyTransform();
      private long period;

      private boolean enable = false;
      private long lastFrame = -1;

      private final SnapshotParameters snapshotParameters = new SnapshotParameters();
      private final PerspectiveCamera camera = new PerspectiveCamera(true);
      private final Affine cameraTransform = new Affine();

      private WritableImage image = null;
      private BufferedImage bufferedImage;

      private int width, height;

      // Temp variables used to compute the camera orientation offset.
      private final Vector3D xAxis = new Vector3D();
      private final Vector3D yAxis = new Vector3D();
      private final Vector3D zAxis = new Vector3D();

      public SingleCameraSensorManager(String robotName, CameraSensorDefinition definition, ReferenceFrame sensorFrame)
      {
         this.robotName = robotName;
         this.sensorName = definition.getName();
         this.sensorFrame = sensorFrame;

         camera.setVerticalFieldOfView(false);
         camera.getTransforms().add(cameraTransform);
         snapshotParameters.setCamera(camera);
         snapshotParameters.setDepthBuffer(true);
         snapshotParameters.setFill(Color.TRANSPARENT);
         configure(definition);
      }

      public void configure(CameraSensorDefinition definition)
      {
         enable = definition.getEnable();

         if (enable)
         {
            period = TimeUnit.MILLISECONDS.toNanos(definition.getUpdatePeriod());

            Vector3D depthAxis = definition.getDepthAxis();
            Vector3D upAxis = definition.getUpAxis();

            if (depthAxis == null || upAxis == null)
            {
               cameraViewOrientationOffset.setIdentity();
            }
            else
            {
               zAxis.setAndNormalize(depthAxis);
               yAxis.setAndNegate(upAxis);
               yAxis.normalize();
               xAxis.cross(yAxis, zAxis);
               yAxis.cross(zAxis, xAxis);

               try
               {
                  cameraViewOrientationOffset.setColumns(xAxis, yAxis, zAxis);
               }
               catch (NotARotationMatrixException e)
               {
                  LogTools.error("Erro computing the camera view direction. depth-axis: {}, up-axis: {}", depthAxis, upAxis);
                  cameraViewOrientationOffset.setIdentity();
               }
            }

            camera.setFieldOfView(Math.toDegrees(definition.getFieldOfView()));
            camera.setNearClip(definition.getClipNear());
            camera.setFarClip(definition.getClipFar());

            if (bufferedImage == null || width != definition.getImageWidth() || height != definition.getImageHeight())
            {
               width = definition.getImageWidth();
               height = definition.getImageHeight();
               image = null;
               snapshotParameters.setViewport(new Rectangle2D(0, 0, width, height));
               bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
            }
         }
         else
         {
            period = -1;
            image = null;
            bufferedImage = null;
         }
      }

      public void update(long now)
      {
         if (!enable)
         {
            lastFrame = -1;
            return;
         }

         if (lastFrame == -1 || now - lastFrame >= period)
         {
            actualCameraPose.set(sensorFrame.getTransformToRoot());
            actualCameraPose.appendOrientation(cameraViewOrientationOffset);
            JavaFXMissingTools.convertRigidBodyTransformToAffine(actualCameraPose, cameraTransform);
            image = mainSceneRoot.snapshot(snapshotParameters, image);
            bufferedImage = SwingFXUtils.fromFXImage(image, bufferedImage);
            messager.submitMessage(topics.getCameraSensorFrame(), new SensorMessage<>(robotName, sensorName, bufferedImage));
            lastFrame = now;
         }
      }
   }
}
