package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.paint.Color;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoConvexPolytopeFX3D;

public class YoPolygonExtrudedFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoConvexPolytopeFX3D yoPolygonExtrudedFX3D = new YoConvexPolytopeFX3D();

      ConvexPolytope3D polytope = EuclidPolytopeFactories.newIcosahedron(0.24);
      polytope.getVertices().forEach(v -> yoPolygonExtrudedFX3D.addVertex(new Tuple3DProperty(v.getX(), v.getY(), v.getZ())));
      yoPolygonExtrudedFX3D.setNumberOfVertices(polytope.getNumberOfVertices());

      yoPolygonExtrudedFX3D.setColor(Color.AQUAMARINE);
      yoPolygonExtrudedFX3D.render();
      yoPolygonExtrudedFX3D.computeBackground();
      yoPolygonExtrudedFX3D.render();
      Simple3DViewer.view3DObjects(yoPolygonExtrudedFX3D.getNode());
   }
}
