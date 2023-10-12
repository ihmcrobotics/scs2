package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.paint.Color;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoSTPCapsuleFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoSTPConvexPolytopeFX3D;
import us.ihmc.scs2.simulation.shapes.STPConvexPolytope3D;

public class YoSTPConvexPolytope3DVisualizer
{
   public static void main(String[] args)
   {
      YoSTPConvexPolytopeFX3D YoSTPConvexPolytopeFX3D = new YoSTPConvexPolytopeFX3D();

      ConvexPolytope3D polytope = EuclidPolytopeFactories.newIcosahedron(0.3);
      polytope.getVertices().forEach(v-> YoSTPConvexPolytopeFX3D.addVertex(new Tuple3DProperty(v.getX(), v.getY(), v.getZ())));
      YoSTPConvexPolytopeFX3D.setNumberOfVertices(polytope.getNumberOfVertices());

      YoSTPConvexPolytopeFX3D.setPolytope3D(polytope);
      YoSTPConvexPolytopeFX3D.setMinimumMargin(0.005);
      YoSTPConvexPolytopeFX3D.setMaximumMargin(0.015);

      YoSTPConvexPolytopeFX3D.setColor(Color.AQUAMARINE);
      YoSTPConvexPolytopeFX3D.render();
      YoSTPConvexPolytopeFX3D.computeBackground();
      YoSTPConvexPolytopeFX3D.render();
      Simple3DViewer.view3DObjects(YoSTPConvexPolytopeFX3D.getNode());
   }
}


