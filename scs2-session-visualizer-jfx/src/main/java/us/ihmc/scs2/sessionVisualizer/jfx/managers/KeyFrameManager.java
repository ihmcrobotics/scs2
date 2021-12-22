package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import gnu.trove.list.array.TIntArrayList;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

public class KeyFrameManager implements Manager
{
   private final ObjectProperty<int[]> keyFrameIndicesProperty = new SimpleObjectProperty<int[]>(this, "keyFrameIndices", null);
   private final TIntArrayList keyFrameIndices = new TIntArrayList();
   private final Property<YoBufferPropertiesReadOnly> bufferProperties;
   private final JavaFXMessager messager;
   private final SessionVisualizerTopics topics;

   public KeyFrameManager(JavaFXMessager messager, SessionVisualizerTopics topics)
   {
      this.messager = messager;
      this.topics = topics;
      bufferProperties = messager.createPropertyInput(topics.getYoBufferCurrentProperties(), null);
      messager.registerJavaFXSyncedTopicListener(topics.getToggleKeyFrame(), messageContent -> toggleKeyFrame());
      messager.registerJavaFXSyncedTopicListener(topics.getGoToNextKeyFrame(), messageContent -> gotToNextKeyFrame());
      messager.registerJavaFXSyncedTopicListener(topics.getGoToPreviousKeyFrame(), messageContent -> gotToPreviousKeyFrame());
      messager.registerJavaFXSyncedTopicListener(topics.getRequestCurrentKeyFrames(),
                                                 messageContent -> messager.submitMessage(topics.getCurrentKeyFrames(), keyFrameIndicesProperty.get()));

      bufferProperties.addListener((o, oldValue, newValue) ->
      {
         if (oldValue == null || oldValue.getSize() != newValue.getSize())
            clearAllKeyFrames();
      });
      keyFrameIndicesProperty.addListener((o, oldValue, newValue) -> messager.submitMessage(topics.getCurrentKeyFrames(), newValue));
   }

   @Override
   public void startSession(Session session)
   {
   }

   @Override
   public void stopSession()
   {
      clearAllKeyFrames();
   }

   @Override
   public boolean isSessionLoaded()
   {
      return true;
   }

   private void clearAllKeyFrames()
   {
      keyFrameIndices.clear();
      keyFrameIndicesProperty.set(null);
   }

   private void toggleKeyFrame()
   {
      if (bufferProperties.getValue() == null)
         return;

      int currentIndex = bufferProperties.getValue().getCurrentIndex();

      if (keyFrameIndices.isEmpty())
      {
         keyFrameIndices.add(currentIndex);
      }
      else
      {
         int result = keyFrameIndices.binarySearch(currentIndex);

         if (result < 0)
         { // No keyframe at this index, let's register it.
            int insertionIndex = -result - 1;
            keyFrameIndices.insert(insertionIndex, currentIndex);
         }
         else
         { // There's a registered keyframe, let's remove it.
            keyFrameIndices.removeAt(result);
         }
      }

      keyFrameIndicesProperty.set(keyFrameIndices.toArray());
   }

   private void gotToNextKeyFrame()
   {
      if (bufferProperties.getValue() == null)
         return;
      if (keyFrameIndices.isEmpty())
         return;

      int currentIndex = bufferProperties.getValue().getCurrentIndex();
      int result = keyFrameIndices.binarySearch(currentIndex);
      int nextKeyframeIndex;

      if (result < 0)
      { // No keyframe at the current index.
         nextKeyframeIndex = -result - 1;
      }
      else
      {
         nextKeyframeIndex = result + 1;
      }

      if (nextKeyframeIndex >= keyFrameIndices.size())
         nextKeyframeIndex = 0;
      messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), keyFrameIndices.get(nextKeyframeIndex));
   }

   private void gotToPreviousKeyFrame()
   {
      if (bufferProperties.getValue() == null)
         return;
      if (keyFrameIndices.isEmpty())
         return;

      int currentIndex = bufferProperties.getValue().getCurrentIndex();
      int result = keyFrameIndices.binarySearch(currentIndex);
      int previousKeyframeIndex;

      if (result < 0)
      { // No keyframe at the current index.
         previousKeyframeIndex = -result - 2;
      }
      else
      {
         previousKeyframeIndex = result - 1;
      }

      if (previousKeyframeIndex < 0)
         previousKeyframeIndex = keyFrameIndices.size() - 1;
      messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), keyFrameIndices.get(previousKeyframeIndex));
   }

   public ReadOnlyObjectProperty<int[]> keyFrameIndicesProperty()
   {
      return keyFrameIndicesProperty;
   }
}
