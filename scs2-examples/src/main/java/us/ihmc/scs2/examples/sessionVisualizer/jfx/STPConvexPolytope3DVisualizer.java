package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.scs2.definition.geometry.STPConvexPolytope3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;

public class STPConvexPolytope3DVisualizer
{
   public static void main(String[] args)
   {
      STPConvexPolytope3DDefinition stpConvexPolytope3D = new STPConvexPolytope3DDefinition();
//      ConvexPolytope3DDefinition polytope = EuclidPolytopeFactories.newIcosahedron(0.1);
//      ConvexPolytope3DReadOnly convexPolytope new ConvexPolytope3DReadOnly();
//      convexPolytope.getv
//      polytope.getVertices().forEach(v -> stpConvexPolytope3D.addVertex(new Tuple3DProperty(v.getX(), v.getY(), v.getZ())));
      stpConvexPolytope3D.setMargins(0.1, 0.5);
      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(stpConvexPolytope3D);
      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);

   }


}
