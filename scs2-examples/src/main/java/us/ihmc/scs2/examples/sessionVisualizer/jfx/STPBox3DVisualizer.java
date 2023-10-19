package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import javafx.scene.shape.MeshView;
import javafx.scene.paint.PhongMaterial;
import us.ihmc.scs2.simulation.shapes.STPBox3D;

public class STPBox3DVisualizer
{
   public static void main(String[] args)
   {
      STPBox3D stpBox3D = new STPBox3D(0.25, 0.25, 0.25);
      stpBox3D.setMargins(0.02, 0.05);
      double smallRadius = stpBox3D.getSmallRadius();
      double largeRadius = stpBox3D.getLargeRadius();

      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.toSTPBox3DMesh(null,
                                                                                                                               stpBox3D.getSize(),
                                                                                                                               smallRadius,
                                                                                                                               largeRadius,
                                                                                                                               false));
      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);



   }
}
