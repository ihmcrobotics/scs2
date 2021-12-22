package us.ihmc.scs2.examples.sessionVisualizer.jfx.plotter2D;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.stage.Stage;
import us.ihmc.scs2.sessionVisualizer.jfx.plotter.Plotter2D;

public class Plotter2DExample extends Application
{

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Plotter2D plotter2d = new Plotter2D();

      Scene scene = new Scene(plotter2d, 600, 400);
      primaryStage.setScene(scene);
//      plotter2d.setFieldOfView(0.0, 0.0, 10.0, 10.0);
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
