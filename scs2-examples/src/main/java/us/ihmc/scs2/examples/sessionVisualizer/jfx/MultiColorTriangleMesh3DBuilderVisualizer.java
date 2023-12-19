package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.paint.Color;
import javafx.scene.shape.Box;
import javafx.stage.Stage;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MultiColorTriangleMesh3DBuilder;
import us.ihmc.scs2.definition.visual.TextureDefinitionColorAdaptivePalette;
import us.ihmc.scs2.sessionVisualizer.jfx.Scene3DBuilder;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class MultiColorTriangleMesh3DBuilderVisualizer
{
   private enum MeshToDisplay
   {
      BOX, LINE, MULTI_LINE
   }

   private static final MeshToDisplay MESH_TO_DISPLAY = MeshToDisplay.BOX;
   private static final boolean USE_RANDOM_COLORS = true;

   private final Random random = new Random(453);

   public MultiColorTriangleMesh3DBuilderVisualizer(Stage primaryStage)
   {
      primaryStage.setTitle(getClass().getSimpleName());

      Scene3DBuilder scene3DBuilder = new Scene3DBuilder();
      Scene scene = new Scene(scene3DBuilder.getRoot(), 600, 400, true, SceneAntialiasing.BALANCED);
      scene.setFill(Color.GREY);
      Simple3DViewer.setupCamera(scene, scene3DBuilder.getRoot());
      scene3DBuilder.addCoordinateSystem(0.3);

      ColorDefinition[] colors;
      if (USE_RANDOM_COLORS)
      {
         colors = new ColorDefinition[1024 * 1024];
         for (int i = 0; i < colors.length; i++)
            colors[i] = ColorDefinition.rgb(random.nextInt());
      }
      else
      {
         colors = new ColorDefinition[] {ColorDefinitions.Red(),
                                         ColorDefinitions.Yellow(),
                                         ColorDefinitions.Beige(),
                                         ColorDefinitions.Chocolate(),
                                         ColorDefinitions.AntiqueWhite(),
                                         ColorDefinitions.Cyan()};
      }

      MultiColorTriangleMesh3DBuilder meshBuilder = new MultiColorTriangleMesh3DBuilder(new TextureDefinitionColorAdaptivePalette());
      //      MultiColorTriangleMesh3DBuilder meshBuilder = new MultiColorTriangleMesh3DBuilder(new TextureDefinitionColorPalette2D());

      switch (MESH_TO_DISPLAY)
      {
         case BOX:
            scene3DBuilder.addNodesToView(addRandomBoxes(colors, meshBuilder));
            break;
         case LINE:
            addLine(meshBuilder);
         case MULTI_LINE:
            addMultiLine(meshBuilder);
         default:
            break;
      }
      //      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.Cyan());
      //      materialDefinition.setSpecularColor(ColorDefinitions.Cyan().brighter());
      //      materialDefinition.setShininess(3);
      //      view3dFactory.addNodeToView(JavaFXVisualTools.toNode(new VisualDefinition(meshBuilder.generateTriangleMesh3D(), materialDefinition), null));

      scene3DBuilder.addNodeToView(JavaFXVisualTools.toNode(meshBuilder.generateVisual(), null));
      primaryStage.setScene(scene);
      primaryStage.show();
   }

   private void addMultiLine(MultiColorTriangleMesh3DBuilder meshBuilder)
   {
      List<Point3D> points = new ArrayList<>();
      double radius = 0.4;
      Random random = new Random();

      for (double angle = 0.0; angle < 2.0 * Math.PI; angle += 2.0 * Math.PI / 50.0)
      {
         double x = radius * random.nextDouble() * Math.cos(angle);
         double y = radius * Math.sin(angle);
         double z = 0.1 * random.nextDouble();
         points.add(new Point3D(x, y, z));
      }
      meshBuilder.addMultiLine(points, 0.01, true, ColorDefinitions.YellowGreen());
   }

   private void addLine(MultiColorTriangleMesh3DBuilder meshBuilder)
   {
      Point3D start = new Point3D(0.3, 0.0, -0.);
      Point3D end = new Point3D(0.0, 0.3, 0.0);
      double lineWidth = 0.01;
      meshBuilder.addLine(start, end, lineWidth, ColorDefinitions.Red());
   }

   public List<Box> addRandomBoxes(ColorDefinition[] colors, MultiColorTriangleMesh3DBuilder meshBuilder)
   {
      int count = 0;
      Random random = new Random();
      List<Box> boxes = new ArrayList<>();

      for (float x = -1.0f; x <= 1.0f; x += 0.055f)
      {
         for (float y = -1.0f; y <= 1.0f; y += 0.055f)
         {
            for (float z = -0.0f; z <= 0.01f; z += 0.055f)
            {
               ColorDefinition color = colors[count % colors.length];
               Vector3D32 pointsOffset = new Vector3D32(x, y, 0 * RandomNumbers.nextFloat(random, -5.0f, 5.0f));
               meshBuilder.addBox(0.05, 0.05, 0.05, pointsOffset, color);
               //               Box box = new Box(0.025f, 0.025f, 0.025f);
               //               box.setTranslateX(pointsOffset.getX());
               //               box.setTranslateY(pointsOffset.getY());
               //               box.setTranslateZ(0.05f + pointsOffset.getZ());
               //               box.setMaterial(new PhongMaterial(JavaFXVisualTools.toColor(color)));
               //               boxes.add(box);
               count++;
            }
         }
      }

      System.out.println("Number of boxes: " + count);
      return boxes;
   }

   public static void main(String[] args)
   {
      ApplicationRunner.runApplication(MultiColorTriangleMesh3DBuilderVisualizer::new);
   }
}