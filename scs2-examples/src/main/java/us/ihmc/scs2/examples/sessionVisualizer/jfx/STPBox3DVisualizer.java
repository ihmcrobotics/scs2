package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.scs2.definition.geometry.STPBox3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import javafx.scene.shape.MeshView;
import javafx.scene.paint.PhongMaterial;


public class STPBox3DVisualizer
{
   public static void main(String[] args)
   {
      STPBox3DDefinition stpBox3D = new STPBox3DDefinition(0.25, 0.25, 0.25);
      stpBox3D.setMargins(0.02, 0.05);


      TriangleMesh3DDefinition stpBox3Ddata = TriangleMesh3DFactories.stpBox3D(stpBox3D);
      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(stpBox3Ddata);


      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);
   }
}
