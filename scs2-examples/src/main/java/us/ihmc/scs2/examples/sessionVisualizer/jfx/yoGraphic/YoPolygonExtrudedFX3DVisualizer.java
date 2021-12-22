package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPolygonExtrudedFX3D;

public class YoPolygonExtrudedFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoPolygonExtrudedFX3D yoPolygonExtrudedFX3D = new YoPolygonExtrudedFX3D();

      double radius = 0.3;
      int numberOfVertices = 5;

      for (int i = 0; i < numberOfVertices; i++)
      {
         double theta = 2.0 * Math.PI * i / numberOfVertices;
         Point2D vertex = new Point2D(radius, 0.0);
         RotationMatrixTools.applyYawRotation(theta, vertex, vertex);
         yoPolygonExtrudedFX3D.addVertex(new Tuple2DProperty(vertex.getX(), vertex.getY()));
      }

      yoPolygonExtrudedFX3D.setNumberOfVertices(numberOfVertices);
      yoPolygonExtrudedFX3D.setThickness(0.15);
      yoPolygonExtrudedFX3D.setColor(Color.AQUAMARINE);
      yoPolygonExtrudedFX3D.render();
      yoPolygonExtrudedFX3D.computeBackground();
      yoPolygonExtrudedFX3D.render();
      Simple3DViewer.view3DObjects(yoPolygonExtrudedFX3D.getNode());
   }
}
