package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;

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
   private static JavaFXApplicationCreator mainApplication;

   private static final CountDownLatch latch = new CountDownLatch(1);
   private static JavaFXApplicationCreator startUpTest = null;

   private static List<Runnable> stopListeners = new ArrayList<>();

   public JavaFXApplicationCreator()
   {
      setStartUpTest(this);
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