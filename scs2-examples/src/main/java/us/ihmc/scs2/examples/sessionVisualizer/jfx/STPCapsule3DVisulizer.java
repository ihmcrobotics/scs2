package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import javafx.scene.shape.MeshView;
import javafx.scene.paint.PhongMaterial;
import us.ihmc.scs2.simulation.shapes.STPCapsule3D;

public class STPCapsule3DVisulizer
{
   public static void main(String[] args)
   {
      STPCapsule3D stpCapsule3D = new STPCapsule3D(0.5, 0.1);
      stpCapsule3D.setMargins(0.01, 0.035);
      double smallRadius = stpCapsule3D.getSmallRadius();
      double largeRadius = stpCapsule3D.getLargeRadius();


      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.toSTPCapsule3DMesh(null,
                                                                                                                                   stpCapsule3D.getRadius(),
                                                                                                                                   stpCapsule3D.getLength(),
                                                                                                                                   smallRadius,
                                                                                                                                   largeRadius,
                                                                                                                                   false));
      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);
   }

}