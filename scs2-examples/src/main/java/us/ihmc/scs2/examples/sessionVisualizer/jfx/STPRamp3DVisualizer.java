package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import us.ihmc.scs2.simulation.shapes.STPRamp3D;

public class STPRamp3DVisualizer
{
   public static void main(String[] args)
   {
      STPRamp3D stpRamp3D = new STPRamp3D(0.3, 0.6, 0.2);
      stpRamp3D.setMargins(0.01,0.05);
      double smallRadius = stpRamp3D.getSmallRadius();
      double largeRadius = stpRamp3D.getLargeRadius();

      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.toSTPRamp3DMesh(null,
                                                                                                                                stpRamp3D.getSizeX(),
                                                                                                                                stpRamp3D.getSizeY(),
                                                                                                                                stpRamp3D.getSizeZ(),
                                                                                                                                smallRadius,
                                                                                                                                largeRadius,
                                                                                                                                false));
      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);
   }
}
