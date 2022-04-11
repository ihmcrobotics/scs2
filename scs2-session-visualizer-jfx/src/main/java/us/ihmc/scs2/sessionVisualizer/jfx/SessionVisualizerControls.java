package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.File;
import java.util.Collection;

import javafx.stage.Window;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;

public interface SessionVisualizerControls
{
   Window getPrimaryWindow();

   void setCameraOrientation(double latitude, double longitude, double roll);

   void setCameraPosition(double x, double y, double z);

   void setCameraFocusPosition(double x, double y, double z);

   void setCameraZoom(double distanceFromFocus);

   void requestCameraRigidBodyTracking(String robotName, String rigidBodyName);

   default void addStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         addStaticVisual(visualDefinition);
      }
   }

   void addStaticVisual(VisualDefinition visualDefinition);
   
   void addYoGraphic(String namespace, YoGraphicDefinition yoGraphicDefinition);
   
   void addYoGraphic(YoGraphicDefinition yoGraphicDefinition);

   default void exportVideo(File file)
   {
      SceneVideoRecordingRequest request = new SceneVideoRecordingRequest();
      request.setFile(file);
      exportVideo(request);
   }

   void exportVideo(SceneVideoRecordingRequest request);

   void shutdown();

   void shutdownNow();

   void addVisualizerShutdownListener(Runnable listener);

   void waitUntilFullyUp();

   void waitUntilDown();
}
