package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.scs2.definition.geometry.STPRamp3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;


public class STPRamp3DVisualizer
{
   public static void main(String[] args)
   {
      STPRamp3DDefinition stpRamp3D = new STPRamp3DDefinition(0.5,0.25,0.25);
      stpRamp3D.setMargins(0.015,0.035);

      TriangleMesh3DDefinition stpRamp3DData = TriangleMesh3DFactories.stpRamp3D(stpRamp3D);
      TriangleMesh   mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(stpRamp3DData);

      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);
   }
}
