package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.simulation.shapes.STPCylinder3D;

public class STPCylinder3DVisualizer
{
   public static void main(String[] args)
   {
      STPCylinder3D stpCylinder3D = new STPCylinder3D(0.5, 0.1);
      stpCylinder3D.setMargins(0.01, 0.025);
      double smallRadius = stpCylinder3D.getSmallRadius();
      double largeRadius = stpCylinder3D.getLargeRadius();

      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.toSTPCylinder3DMesh(null,
                                                                                                                                    stpCylinder3D.getRadius(),
                                                                                                                                    stpCylinder3D.getLength(),
                                                                                                                                    smallRadius,
                                                                                                                                    largeRadius,
                                                                                                                                    false));
      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);
   }
}
