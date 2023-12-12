package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.scs2.definition.visual.*;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;

import java.util.Random;

public class TriangleMesh3DBuilderJFXVisualizer
{
   private enum MeshToDisplay
   {
      BOX, LINE, CYLINDER, CONE
   }

   private static final MeshToDisplay MESH_TO_DISPLAY = MeshToDisplay.BOX;

   public TriangleMesh3DBuilderJFXVisualizer()
   {
      TriangleMesh3DBuilder meshBuilder = new TriangleMesh3DBuilder();
      switch (MESH_TO_DISPLAY)
      {
         case BOX:
            addRandomBoxes(meshBuilder);
            break;
         case LINE:
            addLines(meshBuilder);
            break;
         case CYLINDER:
            addCylinders(meshBuilder);
            break;
         case CONE:
            addCones(meshBuilder);
            break;
         default:
            break;
      }

      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.Cyan());
      materialDefinition.setSpecularColor(ColorDefinitions.Cyan().brighter());
      materialDefinition.setShininess(3);
      Simple3DViewer.view3DObjects(JavaFXVisualTools.toNode(new VisualDefinition(meshBuilder.generateTriangleMesh3D(), materialDefinition), null));
   }

   public void addRandomBoxes(TriangleMesh3DBuilder meshBuilder)
   {
      int count = 0;
      for (float x = -5.0f; x <= 5.0f; x += 0.055f)
      {
         for (float y = -2.0f; y <= 2.0f; y += 0.055f)
         {
            meshBuilder.addBox(0.05f, 0.05f, 0.05f, new Vector3D32(x, y, RandomNumbers.nextFloat(new Random(), -2.0f, 2.0f)));
            count++;
         }
      }
      System.out.println("Number of boxes: " + count);
   }

   private void addLines(TriangleMesh3DBuilder meshBuilder)
   {
      Point3D start = new Point3D(0.3, 0.0, -0.);
      Point3D end = new Point3D(0.0, 0.3, 0.0);
      double lineWidth = 0.01;
      meshBuilder.addLine(start, end, lineWidth);
   }

   private void addCylinders(TriangleMesh3DBuilder meshBuilder)
   {
      Point3D cylinderPosition = new Point3D(1.0, 0.0, 0.0);
      double height = 0.3;
      double radius = 0.1;
      meshBuilder.addCylinder(height, radius, cylinderPosition);
      //      meshBuilder.addMesh(TriangleMesh3DFactories.ArcTorus(0.0, 2.0 * Math.PI, 0.3, 0.01, 128));
      meshBuilder.addTriangleMesh3D(TriangleMesh3DFactories.Cylinder(radius, height, 64, true));
   }

   private void addCones(TriangleMesh3DBuilder meshBuilder)
   {
      Point3D conePosition = new Point3D(0.4, 0.0, 0.0);
      double height = 0.3;
      double radius = 0.1;
      meshBuilder.addCone(height, radius, conePosition);
      //      meshBuilder.addMesh(MeshDataGenerator.ArcTorus(0.0, 2.0 * Math.PI, 0.3, 0.01, 128));
      meshBuilder.addTriangleMesh3D(TriangleMesh3DFactories.Cone(height, radius, 64));
   }

   public static void main(String[] args)
   {
      new TriangleMesh3DBuilderJFXVisualizer();
   }
}