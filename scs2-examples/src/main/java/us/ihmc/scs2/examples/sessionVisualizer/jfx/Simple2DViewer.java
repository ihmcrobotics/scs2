package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import us.ihmc.scs2.sessionVisualizer.jfx.plotter.Plotter2D;

import java.util.Arrays;

public class Simple2DViewer
{
   public static void view2DObjects(Node... nodesToView)
   {
      view2DObjects(null, nodesToView);
   }

   public static void view2DObjects(Runnable updatable, Node... nodesToView)
   {
      view2DObjects(updatable, Arrays.asList(nodesToView));
   }

   public static void view2DObjects(Iterable<? extends Node> nodesToView)
   {
      view2DObjects(null, nodesToView);
   }

   public static void view2DObjects(Runnable updatable, Iterable<? extends Node> nodesToView)
   {
      ApplicationRunner.runApplication(primaryStage ->
                                       {
                                          Plotter2D plotter2d = new Plotter2D();

                                          for (Node nodeToView : nodesToView)
                                             plotter2d.getRoot().getChildren().add(nodeToView);

                                          Scene scene = new Scene(plotter2d, 600, 400, true, SceneAntialiasing.BALANCED);
                                          primaryStage.setScene(scene);
                                          plotter2d.setFieldOfView(0.0, 0.0, 1.0, 1.0);
                                          primaryStage.show();

                                          if (updatable != null)
                                          {
                                             new AnimationTimer()
                                             {
                                                @Override
                                                public void handle(long now)
                                                {
                                                   updatable.run();
                                                }
                                             }.start();
                                          }
                                       });
   }
}
