package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.geometry.STPConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.geometry.shapes.STPConvexPolytope3D;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;

public class STPConvexPolytope3DVisualizer
{
   private static final boolean ADD_DOTS = true;

   public static void main(String[] args)
   {
      STPConvexPolytope3DDefinition stpConvexPolytope3D = polytope2();

      TriangleMesh3DDefinition stpConvexPolytope3DData = TriangleMesh3DFactories.stpConvexPolytope3D(stpConvexPolytope3D);
      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(stpConvexPolytope3DData);

      Random random = new Random(342534);
      List<Node> dots = new ArrayList<>();

      if (ADD_DOTS)
      {
         STPConvexPolytope3D polytope = new STPConvexPolytope3D(stpConvexPolytope3D.getConvexPolytope());
         polytope.setMargins(stpConvexPolytope3D.getMinimumMargin(), stpConvexPolytope3D.getMaximumMargin());

         for (int i = 0; i < 500; i++)
         {
            Quaternion orientation = EuclidCoreRandomTools.nextQuaternion(random);
            Vector3D direction = new Vector3D(0, 0, 1);
            orientation.transform(direction);

            Vertex3DReadOnly position = polytope.getSupportingVertex(direction);
            Sphere sphere = new Sphere(0.001);
            sphere.setMaterial(new PhongMaterial(Color.RED));
            sphere.setTranslateX(position.getX());
            sphere.setTranslateY(position.getY());
            sphere.setTranslateZ(position.getZ());
            dots.add(sphere);
         }
      }

      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      dots.add(meshView);
      Simple3DViewer.view3DObjects(dots);

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
