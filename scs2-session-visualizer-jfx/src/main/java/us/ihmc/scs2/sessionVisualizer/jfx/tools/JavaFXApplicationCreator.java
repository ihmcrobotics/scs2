package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;

import org.apache.commons.lang3.SystemUtils;

import javafx.application.Application;
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

         int glSyncToVBlankIntValue;
         String glSyncToVBlankProperty = System.getenv(gl_vsync_name);
         if (glSyncToVBlankProperty == null)
         {
            glSyncToVBlankIntValue = -1;
         }
         else
         {
            try
            {
               glSyncToVBlankIntValue = Integer.parseInt(glSyncToVBlankProperty);
            }
            catch (NumberFormatException e)
            {
               e.printStackTrace();
               glSyncToVBlankIntValue = -1;
            }
         }

         if (glSyncToVBlankIntValue != 0)
            System.err.println("%s: JavaFX performance warning: disable VSync for better multi-window performance, run with environment variable: %s=0".formatted(JavaFXApplicationCreator.class.getSimpleName(),
                                                                                                                                                                  gl_vsync_name));
      }
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