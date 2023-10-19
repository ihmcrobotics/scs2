package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.simulation.shapes.STPConvexPolytope3D;

public class STPConvexPolytope3DVisualizer
{
   public static void main(String[] args)
   {
      STPConvexPolytope3D stpConvexPolytope3D = new STPConvexPolytope3D();
      ConvexPolytope3D polytope = EuclidPolytopeFactories.newIcosahedron(0.3);
      polytope.getVertices().forEach(v->stpConvexPolytope3D.addVertex(new Point3D(v.getX(), v.getY(), v.getZ())));


      stpConvexPolytope3D.setMargins(0.01, 0.025);
      double smallRadius = stpConvexPolytope3D.getSmallRadius();
      double largeRadius = stpConvexPolytope3D.getLargeRadius();

      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.toSTPConvexPolytope3DMesh(stpConvexPolytope3D,
                                                                                                                                          smallRadius,
                                                                                                                                          largeRadius,
                                                                                                                                          false));
      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);

   }


}
