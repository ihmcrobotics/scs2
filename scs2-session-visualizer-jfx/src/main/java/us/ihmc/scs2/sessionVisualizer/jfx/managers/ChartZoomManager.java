package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartIntegerBounds;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.FXCoalescedUpdater;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

import java.util.function.Consumer;

public class ChartZoomManager extends ObservedAnimationTimer
{
   private final Window owner;
   private final SessionVisualizerTopics topics;
   private final JavaFXMessager messager;
   private final SessionVisualizerWindowToolkit windowToolkit;

   private final Property<ChartIntegerBounds> currentBoundsProperty = new SimpleObjectProperty<>(this, "currentBoundsProperty", null);
   private final DoubleProperty zoomFactorProperty = new SimpleDoubleProperty(this, "zoomFactor", 2.0);
   private final Property<YoBufferPropertiesReadOnly> currentBufferPropertiesProperty = new SimpleObjectProperty<>(this,
                                                                                                                    "currentBufferProperties",
                                                                                                                    null);
   private Consumer<YoBufferPropertiesReadOnly> currentBufferPropertiesListener;
   private final FXCoalescedUpdater<YoBufferPropertiesReadOnly> currentBufferPropertiesUpdater = new FXCoalescedUpdater<>(currentBufferPropertiesProperty::setValue);

   // Used to detect when the buffer is being resized and to reset the zoom.
   private int previousBufferSize = -1;

   private boolean initialize = true;

   public ChartZoomManager(Window owner, SessionVisualizerWindowToolkit windowToolkit, JavaFXMessager messager, SessionVisualizerTopics topics)
   {
      this.owner = owner;
      this.windowToolkit = windowToolkit;
      this.topics = topics;
      this.messager = messager;

      messager.addFXTopicListener(topics.getYoChartZoomFactor(), m ->
      {
         if (m.getKey() == owner)
            zoomFactorProperty.set(m.getValue());
      });
      windowToolkit.addSessionChangedListener((previousSession, newSession) ->
      {
         if (previousSession != null && currentBufferPropertiesListener != null)
            previousSession.removeCurrentBufferPropertiesListener(currentBufferPropertiesListener);

         if (newSession == null)
         {
            currentBufferPropertiesListener = null;
            return;
         }

         currentBufferPropertiesListener = currentBufferPropertiesUpdater::update;
         newSession.addCurrentBufferPropertiesListener(currentBufferPropertiesListener);
      });
      Session currentSession = windowToolkit.getSession();
      if (currentSession != null)
      {
         currentBufferPropertiesListener = currentBufferPropertiesUpdater::update;
         currentSession.addCurrentBufferPropertiesListener(currentBufferPropertiesListener);
      }
      messager.addTopicListener(topics.getYoChartRequestZoomIn(), this::processZoomInRequest);
      messager.addTopicListener(topics.getYoChartRequestZoomOut(), this::processZoomOutRequest);
      messager.addTopicListener(topics.getYoChartRequestShift(), this::processShiftRequest);
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
      Session session = windowToolkit.getSession();
      if (session != null)
      {
         if (currentBufferProperties.getCurrentIndex() <= lowerBoundForCurrentIndex)
            session.submitBufferIndexRequest(lowerBoundForCurrentIndex + 2 * margin);
         if (currentBufferProperties.getCurrentIndex() >= upperBoundForCurrentIndex)
            session.submitBufferIndexRequest(upperBoundForCurrentIndex - 2 * margin);
      }

      currentBoundsProperty.setValue(new ChartIntegerBounds(newLowerBound, newUpperBound));
   }

   public Property<ChartIntegerBounds> chartBoundsProperty()
   {
      return currentBoundsProperty;
   }
}
