package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.scs2.definition.geometry.STPCylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;

public class STPCylinder3DVisualizer
{
   public static void main(String[] args)
   {
      STPCylinder3DDefinition stpCylinder3D = new STPCylinder3DDefinition(0.7, 0.5);
      stpCylinder3D.setMargins(0.01, 0.03);

      TriangleMesh3DDefinition stpCylinder3Ddata = TriangleMesh3DFactories.stpCylinder3D(stpCylinder3D);
      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(stpCylinder3Ddata);

      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);
   }
}
