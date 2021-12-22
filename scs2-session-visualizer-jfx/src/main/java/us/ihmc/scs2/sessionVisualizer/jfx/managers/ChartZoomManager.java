package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartIntegerBounds;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

public class ChartZoomManager extends ObservedAnimationTimer
{
   private final Window owner;
   private final SessionVisualizerTopics topics;
   private final JavaFXMessager messager;

   private final Property<ChartIntegerBounds> currentBoundsProperty = new SimpleObjectProperty<>(this, "currentBoundsProperty", null);
   private final DoubleProperty zoomFactorProperty = new SimpleDoubleProperty(this, "zoomFactor", 2.0);
   private final Property<YoBufferPropertiesReadOnly> currentBufferPropertiesProperty;

   // Used to detect when the buffer is being resized and to reset the zoom.
   private int previousBufferSize = -1;

   private boolean initialize = true;

   public ChartZoomManager(Window owner, JavaFXMessager messager, SessionVisualizerTopics topics)
   {
      this.owner = owner;
      this.topics = topics;
      this.messager = messager;

      messager.registerJavaFXSyncedTopicListener(topics.getYoChartZoomFactor(), m ->
      {
         if (m.getKey() == owner)
            zoomFactorProperty.set(m.getValue());
      });
      currentBufferPropertiesProperty = messager.createPropertyInput(topics.getYoBufferCurrentProperties());
      messager.registerTopicListener(topics.getYoChartRequestZoomIn(), this::processZoomInRequest);
      messager.registerTopicListener(topics.getYoChartRequestZoomOut(), this::processZoomOutRequest);
      messager.registerTopicListener(topics.getYoChartRequestShift(), this::processShiftRequest);
   }

   @Override
   public void start()
   {
      super.start();
      initialize = true;
   }

   public boolean initializeBounds()
   {
      if (!initialize)
         return true;

      YoBufferPropertiesReadOnly currentBufferProperties = currentBufferPropertiesProperty.getValue();
      if (currentBufferProperties == null)
         return false;

      currentBoundsProperty.setValue(new ChartIntegerBounds(0, currentBufferProperties.getSize() - 1));
      initialize = false;

      return true;
   }

   @Override
   public void handleImpl(long now)
   {
      YoBufferPropertiesReadOnly currentBufferProperties = currentBufferPropertiesProperty.getValue();

      if (currentBufferProperties != null && previousBufferSize != currentBufferProperties.getSize())
      {
         // That will trigger a re-initialization and reset the zoom.
         initialize = true;
         previousBufferSize = currentBufferProperties.getSize();
      }

      if (!initializeBounds())
         return;

      ChartIntegerBounds currentBounds = currentBoundsProperty.getValue();

      if (currentBounds.getUpper() >= currentBufferProperties.getSize())
      {
         System.out.println("Reinitializing bounds");
         initialize = true;
         return;
      }

      int minIndex = 0;
      int maxIndex = currentBufferProperties.getSize() - 1;

      if (currentBounds.getLower() == minIndex && currentBounds.getUpper() == maxIndex)
         return;

      int currentIndex = currentBufferProperties.getCurrentIndex();

      if (currentBounds.isInside(currentIndex))
         return;

      currentBoundsProperty.setValue(currentBounds.center(currentIndex, minIndex, maxIndex));
   }

   private void processZoomInRequest(Pair<Window, Boolean> request)
   {
      if (request == null || request.getKey() != owner)
         return;
      if (!initializeBounds())
         return;

      YoBufferPropertiesReadOnly currentBufferProperties = currentBufferPropertiesProperty.getValue();
      int currentIndex = currentBufferProperties.getCurrentIndex();
      int minLength = 4;
      int minIndex = 0;
      int maxIndex = currentBufferProperties.getSize() - 1;

      ChartIntegerBounds oldBounds = currentBoundsProperty.getValue();
      currentBoundsProperty.setValue(oldBounds.zoom(currentIndex, minLength, minIndex, maxIndex, zoomFactorProperty.getValue()));
   }

   private void processZoomOutRequest(Pair<Window, Boolean> request)
   {
      if (request == null || request.getKey() != owner)
         return;
      if (!initializeBounds())
         return;

      YoBufferPropertiesReadOnly currentBufferProperties = currentBufferPropertiesProperty.getValue();
      int currentIndex = currentBufferProperties.getCurrentIndex();
      int minLength = 4;
      int minIndex = 0;
      int maxIndex = currentBufferProperties.getSize() - 1;

      ChartIntegerBounds oldBounds = currentBoundsProperty.getValue();
      currentBoundsProperty.setValue(oldBounds.zoom(currentIndex, minLength, minIndex, maxIndex, 1.0 / zoomFactorProperty.getValue()));
   }

   private void processShiftRequest(Pair<Window, Integer> request)
   {
      if (request == null || request.getKey() != owner)
         return;
      if (!initializeBounds())
         return;

      ChartIntegerBounds currentBounds = currentBoundsProperty.getValue();
      YoBufferPropertiesReadOnly currentBufferProperties = currentBufferPropertiesProperty.getValue();

      int minIndex = 0;
      int maxIndex = currentBufferProperties.getSize() - 1;

      if (currentBounds.getLower() == minIndex && currentBounds.getUpper() == maxIndex)
         return;

      int shiftRequest = request.getValue();
      int newLowerBound = currentBounds.getLower() + shiftRequest;
      int newUpperBound = currentBounds.getUpper() + shiftRequest;
      int distanceFromMin = newLowerBound - minIndex;
      int distanceFromMax = newUpperBound - maxIndex;

      if (distanceFromMin < 0)
      {
         newLowerBound -= distanceFromMin;
         newUpperBound -= distanceFromMin;
      }

      if (distanceFromMax > 0)
      {
         newLowerBound -= distanceFromMax;
         newUpperBound -= distanceFromMax;
      }

      int length = newUpperBound - newLowerBound;

      // Checking if the current index is about to be outside the visible range.
      // If so, we push it back towards the inside.
      // Also because the processing of the current index requests submitted below are executed on another thread,
      // we add some margin to improve our chances that it'll be updated before the index ends up outside the view
      // which would cause the handle method to re-center the view around the index.
      int margin = Math.max(length / 20, 1); // TODO Not sure if we want this parameterized.
      int lowerBoundForCurrentIndex = newLowerBound + margin;
      int upperBoundForCurrentIndex = newUpperBound - margin;

      // If the index is about to go outside view, we push it in by much more than needed preventing a glitch artifact.
      if (currentBufferProperties.getCurrentIndex() <= lowerBoundForCurrentIndex)
         messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), lowerBoundForCurrentIndex + 2 * margin);
      if (currentBufferProperties.getCurrentIndex() >= upperBoundForCurrentIndex)
         messager.submitMessage(topics.getYoBufferCurrentIndexRequest(), upperBoundForCurrentIndex - 2 * margin);

      currentBoundsProperty.setValue(new ChartIntegerBounds(newLowerBound, newUpperBound));
   }

   public Property<ChartIntegerBounds> chartBoundsProperty()
   {
      return currentBoundsProperty;
   }
}
