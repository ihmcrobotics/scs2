package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.scs2.definition.geometry.STPCapsule3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import javafx.scene.shape.MeshView;
import javafx.scene.paint.PhongMaterial;

public class STPCapsule3DVisulizer
{
   public static void main(String[] args)
   {
      STPCapsule3DDefinition stpCapsule3D = new STPCapsule3DDefinition(0.3, 0.05);
      stpCapsule3D.setMargins(0.01, 0.05);

      TriangleMesh3DDefinition stpCapsule3Ddata = TriangleMesh3DFactories.stpCapsule3D(stpCapsule3D);
      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(stpCapsule3Ddata);

      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);
   }

}