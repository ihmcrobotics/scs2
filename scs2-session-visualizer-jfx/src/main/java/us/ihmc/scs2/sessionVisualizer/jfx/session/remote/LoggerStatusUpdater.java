package us.ihmc.scs2.sessionVisualizer.jfx.session.remote;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;

/**
 * Simple JPanel to visualize the status of the logger
 * 
 * @author Jesper Smith
 */
public class LoggerStatusUpdater
{
   private final static int LOGGER_STATUS_TIMEOUT_SECONDS = 6;

   private final Object schedulerLock = new Object();

   private final ScheduledExecutorService offlineExecutor = Executors.newSingleThreadScheduledExecutor();
   private ScheduledFuture<?> offlineExecutorFuture = null;

   private final AtomicBoolean isLogging = new AtomicBoolean(false);
   private final AtomicInteger currentLogDuration = new AtomicInteger(-1);
   private final AtomicBoolean isCameraRecording = new AtomicBoolean(false);

   public LoggerStatusUpdater()
   {
      reportLoggerOffline();
   }

   /**
    * Receive the logger status from the command server Will only act on LOG_ACTIVE_WITH_CAMERA or
    * LOG_ACTIVE commands
    * 
    * @param command
    * @param argument
    */
   public void updateStatus(DataServerCommand command, int argument)
   {
      if (command == DataServerCommand.LOG_ACTIVE_WITH_CAMERA)
      {
         isLogging.set(true);
         isCameraRecording.set(true);
         currentLogDuration.set(argument);
         startLoggerOfflineTimeout();
      }
      else if (command == DataServerCommand.LOG_ACTIVE)
      {
         isLogging.set(true);
         isCameraRecording.set(false);
         currentLogDuration.set(argument);
         startLoggerOfflineTimeout();
      }
   }

   private void startLoggerOfflineTimeout()
   {
      synchronized (schedulerLock)
      {
         if (offlineExecutorFuture != null)
            offlineExecutorFuture.cancel(false);

         offlineExecutorFuture = offlineExecutor.schedule(() -> reportLoggerOffline(), LOGGER_STATUS_TIMEOUT_SECONDS, TimeUnit.SECONDS);
      }
   }

   private void reportLoggerOffline()
   {
      isLogging.set(false);
      isCameraRecording.set(false);
      currentLogDuration.set(-1);
   }

   public boolean isLogging()
   {
      return isLogging.get();
   }

   public boolean isCameraRecording()
   {
      return isCameraRecording.get();
   }

   public int getCurrentLogDuration()
   {
      return currentLogDuration.get();
   }
}
