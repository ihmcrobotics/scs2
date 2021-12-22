package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple2DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPolygonFX2D;

public class YoPolygonFX2DVisualizer
{
   public static void main(String[] args)
   {
      YoPolygonFX2D yoPolygonFX2D = new YoPolygonFX2D();

      double radius = 0.3;
      int numberOfVertices = 5;

      for (int i = 0; i < numberOfVertices; i++)
      {
         double theta = 2.0 * Math.PI * i / numberOfVertices + Math.PI / 2.0;
         Point2D vertex = new Point2D(radius, 0.0);
         RotationMatrixTools.applyYawRotation(theta, vertex, vertex);
         yoPolygonFX2D.addVertex(new Tuple2DProperty(vertex.getX(), vertex.getY()));
      }

      yoPolygonFX2D.setNumberOfVertices(numberOfVertices);
      yoPolygonFX2D.setStrokeColor(Color.AQUAMARINE.darker());
      Simple2DViewer.view2DObjects(() ->
      {
         yoPolygonFX2D.render();
         yoPolygonFX2D.computeBackground();
      }, yoPolygonFX2D.getNode());
   }
}
