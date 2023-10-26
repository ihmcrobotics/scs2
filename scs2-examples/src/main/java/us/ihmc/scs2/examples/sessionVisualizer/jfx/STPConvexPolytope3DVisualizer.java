package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.geometry.STPConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;


public class STPConvexPolytope3DVisualizer
{
   public static void main(String[] args)
   {
      STPConvexPolytope3DDefinition stpConvexPolytope3D = polytope2();

      TriangleMesh3DDefinition stpConvexPolytope3DData = TriangleMesh3DFactories.stpConvexPolytope3D(stpConvexPolytope3D);
      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(stpConvexPolytope3DData);

      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);

   }

   private static STPConvexPolytope3DDefinition polytope1()
   {
      STPConvexPolytope3DDefinition stpConvexPolytope3D = new STPConvexPolytope3DDefinition();
      ConvexPolytope3D polytope = EuclidPolytopeFactories.newIcosahedron(0.3);

      stpConvexPolytope3D.setConvexPolytope(polytope);
      stpConvexPolytope3D.setMargins(0.01, 0.05);
      return stpConvexPolytope3D;
   }


   private static STPConvexPolytope3DDefinition polytope2()
   {
      STPConvexPolytope3DDefinition stpConvexPolytope3D = new STPConvexPolytope3DDefinition();
      ConvexPolytope3D polytope = new ConvexPolytope3D();
      polytope.addVertex(new Point3D(0.00, -0.050, 0.0));
      polytope.addVertex(new Point3D(0.00, +0.050, 0.0));
      polytope.addVertex(new Point3D(0.17, -0.050, 0.0));
      polytope.addVertex(new Point3D(0.17, +0.050, 0.0));
      polytope.addVertex(new Point3D(0.20, -0.025, 0.0));
      polytope.addVertex(new Point3D(0.20, +0.025, 0.0));

      polytope.addVertex(new Point3D(0.00, -0.050, 0.05));
      polytope.addVertex(new Point3D(0.00, +0.050, 0.05));
      polytope.addVertex(new Point3D(0.17, -0.050, 0.05));
      polytope.addVertex(new Point3D(0.17, +0.050, 0.05));
      polytope.addVertex(new Point3D(0.20, -0.025, 0.05));
      polytope.addVertex(new Point3D(0.20, +0.025, 0.05));

      stpConvexPolytope3D.setConvexPolytope(polytope);
      stpConvexPolytope3D.setMargins(0.01, 0.05);
      return stpConvexPolytope3D;
   }


}
