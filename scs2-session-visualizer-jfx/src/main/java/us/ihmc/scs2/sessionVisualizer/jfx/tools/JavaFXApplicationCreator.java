package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;

import org.apache.commons.lang3.SystemUtils;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.stage.Modality;
import javafx.stage.Stage;

/**
 * This class allows for spinning up the JavaFX engine without having to have your code extend an
 * Application. To use, simply call createAJavaFXApplication(). Then make JavaFX windows pop up from
 * non JavaFX Applications by using JavaFX Stage, Scene, Group, etc. objects and then just doing
 * Scene.show(); Unfortunately, you can only ever have one JavaFX "Application" running at the same
 * time. This class makes it easy to ensure that you have one and only one. See the test class for
 * this class for an example of how to create and display a JavaFX scene.
 * 
 * @author JerryPratt
 */
public class JavaFXApplicationCreator extends Application
{
   static
   { // Verifies settings at startup
      verifyVSyncDisabledUbuntu();
   }

   private static JavaFXApplicationCreator mainApplication;

   private static final CountDownLatch latch = new CountDownLatch(1);
   private static JavaFXApplicationCreator startUpTest = null;

   private static List<Runnable> stopListeners = new ArrayList<>();

   public JavaFXApplicationCreator()
   {
      setStartUpTest(this);
   }

   /**
    * Verifies that VSync is disabled on Linux as this is a workaround for the ongoing issue:
    * <a href="https://bugs.java.com/bugdatabase/view_bug.do?bug_id=8291958">Java bug ticket</a>.
    * <p>
    * The issue results in frame rate drop when running a multi window application.
    * </p>
    */
   public static void verifyVSyncDisabledUbuntu()
   {

      if (SystemUtils.IS_OS_LINUX)
      {
         String prism_vsync_name = "prism.vsync";
         String gl_vsync_name = "__GL_SYNC_TO_VBLANK";

         if (System.getProperty(prism_vsync_name) == null)
         {
            System.setProperty(prism_vsync_name, "false");
         }

         if (isVSyncMisconfiguredOnLinux())
            System.err.println("%s: JavaFX performance warning: disable VSync for better multi-window performance, run with environment variable: %s=0".formatted(JavaFXApplicationCreator.class.getSimpleName(),
                                                                                                                                                                  gl_vsync_name));
      }
   }

   /**
    * Checks whether VSync is left enabled on Linux, i.e. the {@code __GL_SYNC_TO_VBLANK} environment
    * variable is not set to {@code 0}.
    * <p>
    * A {@code null}, unparseable, or non-zero value all count as misconfigured. See
    * {@link #verifyVSyncDisabledUbuntu()} for the underlying issue.
    * </p>
    *
    * @return {@code true} when running on Linux and {@code __GL_SYNC_TO_VBLANK} is not {@code 0}.
    */
   public static boolean isVSyncMisconfiguredOnLinux()
   {
      if (!SystemUtils.IS_OS_LINUX)
         return false;

      String glSyncToVBlankProperty = System.getenv("__GL_SYNC_TO_VBLANK");
      if (glSyncToVBlankProperty == null)
         return true;

      try
      {
         return Integer.parseInt(glSyncToVBlankProperty) != 0;
      }
      catch (NumberFormatException e)
      {
         e.printStackTrace();
         return true;
      }
   }

   /**
    * If VSync is misconfigured on Linux (see {@link #isVSyncMisconfiguredOnLinux()}), pops up a
    * non-blocking warning dialog explaining that multi-window playback may stutter and how to fix it.
    * <p>
    * This complements the console warning emitted by {@link #verifyVSyncDisabledUbuntu()}, which runs
    * before the JavaFX toolkit is up and thus cannot show a dialog. This method is safe to call from
    * any thread; the dialog is shown on the JavaFX Application Thread.
    * </p>
    */
   public static void showVSyncWarningDialogIfNeeded()
   {
      if (!isVSyncMisconfiguredOnLinux())
         return;

      Platform.runLater(() ->
      {
         Alert alert = new Alert(AlertType.WARNING, "", ButtonType.OK);
         alert.initModality(Modality.APPLICATION_MODAL);
         alert.setTitle("Performance Warning");
         alert.setHeaderText("VSync is not disabled (multi-window playback may stutter)");
         alert.setContentText("""
                              The environment variable __GL_SYNC_TO_VBLANK=0 is not set, so multi-window chart playback will stutter.

                              Recommended fix: launch SCS2 via the SCS2SessionVisualizer script, which sets it automatically:
                                - when running from source: build/install/.../bin/SCS2SessionVisualizer
                                - or install and run the packaged .deb

                              If running from an IDE, add the environment variable __GL_SYNC_TO_VBLANK=0 to the run configuration.""");
         alert.show();
      });
   }

   private void setStartUpTest(JavaFXApplicationCreator startUpTest)
   {
      JavaFXApplicationCreator.startUpTest = startUpTest;
      latch.countDown();
   }

   private static JavaFXApplicationCreator waitForStartUpTest()
   {
      try
      {
         latch.await();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }

      return startUpTest;
   }

   public static void attachStopListener(Runnable stopListener)
   {
      stopListeners.add(stopListener);
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
   }

   @Override
   public void stop() throws Exception
   {
      for (Runnable stopListener : stopListeners)
      {
         stopListener.run();
      }
      mainApplication = null;
   }

   /**
    * Call this method to spin up the JavaFX engine. If it is already spun up, then it will ignore the
    * call.
    * 
    * @return JavaFX Application that is being run.
    */
   public static JavaFXApplicationCreator spawnJavaFXMainApplication()
   {
      if (mainApplication != null)
         return mainApplication;

      new Thread(() -> Application.launch(JavaFXApplicationCreator.class), "JavaFX-spawner").start();

      mainApplication = JavaFXApplicationCreator.waitForStartUpTest();

      return mainApplication;
   }
}