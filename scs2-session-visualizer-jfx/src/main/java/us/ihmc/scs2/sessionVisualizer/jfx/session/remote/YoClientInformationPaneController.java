package us.ihmc.scs2.sessionVisualizer.jfx.session.remote;

import java.util.function.Supplier;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import us.ihmc.commons.Conversions;
import us.ihmc.scs2.session.remote.LoggerStatusUpdater;
import us.ihmc.scs2.session.remote.RemoteSession;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionInfoController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;

public class YoClientInformationPaneController extends ObservedAnimationTimer implements SessionInfoController
{
   @FXML 
   private AnchorPane mainPane;
   @FXML
   private Label delayLabel;
   @FXML
   private Label logDurationLabel;
   @FXML
   private Label cameraLabel;

   private final long refreshPeriod = Conversions.secondsToNanoseconds(0.1);
   private long lastRefreshTime = -1;

   private Supplier<String> delayValueSupplier;
   private Supplier<String> logDurationValueSupplier;
   private Supplier<String> cameraValueSupplier;

   private final ObjectProperty<RemoteSession> activeSessionProperty = new SimpleObjectProperty<>(this, "activeSession", null);

   public void initialize()
   {
      lastRefreshTime = -1;

      delayValueSupplier = () ->
      {
         RemoteSession activeSession = activeSessionProperty.get();
         if (activeSession == null)
            return null;
         else
            return Conversions.nanosecondsToMilliseconds(activeSession.getDelay()) + "ms";
      };

      logDurationValueSupplier = () ->
      {
         RemoteSession activeSession = activeSessionProperty.get();
         if (activeSession == null)
            return null;

         LoggerStatusUpdater loggerStatusUpdater = activeSession.getLoggerStatusUpdater();

         if (loggerStatusUpdater.isLogging())
            return loggerStatusUpdater.getCurrentLogDuration() + "sec";
         else
            return "Logger offline";
      };

      cameraValueSupplier = () ->
      {
         RemoteSession activeSession = activeSessionProperty.get();
         if (activeSession == null)
            return null;

         LoggerStatusUpdater loggerStatusUpdater = activeSession.getLoggerStatusUpdater();

         return loggerStatusUpdater.isCameraRecording() ? "Recording" : "Off";
      };
   }

   public ObjectProperty<RemoteSession> activeSessionProperty()
   {
      return activeSessionProperty;
   }

   @Override
   public void handleImpl(long now)
   {
      if (lastRefreshTime != -1 && (now - lastRefreshTime) < refreshPeriod)
         return;

      updateLabel(delayLabel, delayValueSupplier, "N/D");
      updateLabel(logDurationLabel, logDurationValueSupplier, "N/D");
      updateLabel(cameraLabel, cameraValueSupplier, "N/D");
      lastRefreshTime = now;
   }

   @Override
   public Pane getMainPane()
   {
      return mainPane;
   }

   private void updateLabel(Label label, Supplier<String> textSupplier, String defaultText)
   {
      if (textSupplier == null)
      {
         label.setText(defaultText);
         return;
      }

      String text = textSupplier.get();

      if (text == null)
      {
         label.setText(defaultText);
         return;
      }

      label.setText(text);
   }
}
