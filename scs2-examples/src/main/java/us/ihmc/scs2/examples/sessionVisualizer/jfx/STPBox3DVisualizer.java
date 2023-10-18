package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.paint.Color;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.scs2.definition.geometry.STPBox3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;
import javafx.scene.shape.MeshView;
import javafx.scene.paint.PhongMaterial;

public class STPBox3DVisualizer
{
   public static void main(String[] args)
   {
      STPBox3DDefinition stpBox3D = new STPBox3DDefinition(0.25, 0.25, 0.25);
      stpBox3D.setMargins(0.02, 0.05);

      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(stpBox3D);
//      TriangleMesh mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.TriangleMesh(stpBox3D));
////            null,
////                                                                                                                               stpBox3D.get(),
////                                                                                                                               stpBox3D.getSmallRadius(),
////                                                                                                                               stpBox3D.getLargeRadius(),
////                                                                                                                               false));
//      mesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(TriangleMesh3DFactories.toSTPBox3DMesh(stpBox3D),
//                                                                           (stpBox3D.getSizeX(), stpBox3D.getSizeY(),stpBox3D.getSizeZ()),
//      stpBox3D.sma)
      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.AQUAMARINE));
      Simple3DViewer.view3DObjects(meshView);
//      Quaternion q = new Quaternion();
//      q.appendPitchRotation(0.4);
//      q.appendRollRotation(0.3);
//      FramePose3D pose = new FramePose3D();
//      STPBox3D stpBox3D = new STPBox3D(pose, 0.1, 0.2, 0.3);
//      stpBox3D.setMargins(0.015,0.035);
//      stpBox3D.get


   }
}
