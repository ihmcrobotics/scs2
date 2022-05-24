package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.File;
import java.util.Collection;
import java.util.Collections;

import javafx.stage.Window;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoEntry.YoEntryListDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;

public interface SessionVisualizerControls
{
   Window getPrimaryWindow();

   void setCameraOrientation(double latitude, double longitude, double roll);

   void setCameraPosition(double x, double y, double z);

   void setCameraFocusPosition(double x, double y, double z);

   void setCameraZoom(double distanceFromFocus);

   default void requestCameraRigidBodyTracking(String robotName, String rigidBodyName)
   {
      waitUntilFullyUp();
      submitMessage(getTopics().getCameraTrackObject(), new CameraObjectTrackingRequest(robotName, rigidBodyName));
   }

   default void addStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         addStaticVisual(visualDefinition);
      }
   }

   void addStaticVisual(VisualDefinition visualDefinition);

   default void removeStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         removeStaticVisual(visualDefinition);
      }
   }

   void removeStaticVisual(VisualDefinition visualDefinition);

   default void addYoGraphic(String namespace, YoGraphicDefinition yoGraphicDefinition)
   {
      String[] subNames = namespace.split(YoGraphicTools.SEPARATOR);
      if (subNames == null || subNames.length == 0)
      {
         addYoGraphic(yoGraphicDefinition);
      }
      else
      {
         for (int i = subNames.length - 1; i >= 0; i--)
         {
            yoGraphicDefinition = new YoGraphicGroupDefinition(subNames[i], yoGraphicDefinition);
         }

         addYoGraphic(yoGraphicDefinition);
      }
   }

   default void addYoGraphic(YoGraphicDefinition yoGraphicDefinition)
   {
      submitMessage(getTopics().getAddYoGraphicRequest(), yoGraphicDefinition);
   }

   default void addYoEntry(String variableName)
   {
      addYoEntry(Collections.singletonList(variableName));
   }

   default void addYoEntry(Collection<String> variableNames)
   {
      addYoEntry(null, variableNames);
   }

   default void addYoEntry(String groupName, Collection<String> variableNames)
   {
      submitMessage(getTopics().getYoEntryListAdd(), YoEntryListDefinition.newYoVariableEntryList(groupName, variableNames));
   }

   default void exportVideo(File file)
   {
      SceneVideoRecordingRequest request = new SceneVideoRecordingRequest();
      request.setFile(file);
      exportVideo(request);
   }

   void exportVideo(SceneVideoRecordingRequest request);

   default void disableUserControls()
   {
      submitMessage(getTopics().getDisableUserControls(), true);
   }

   default void enableUserControls()
   {
      submitMessage(getTopics().getDisableUserControls(), false);
   }

   SessionVisualizerTopics getTopics();

   <T> void submitMessage(Topic<T> topic, T messageContent);

   void shutdown();

   void shutdownNow();

   void addVisualizerShutdownListener(Runnable listener);

   void waitUntilFullyUp();

   void waitUntilDown();
}
