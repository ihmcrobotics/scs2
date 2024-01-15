package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TextureDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.Scene3DBuilder;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.PerspectiveCameraController;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;

import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

public class VisualDefinitionFactoryJFXVisualizer
{
   private static final boolean USE_TEXTURE = true;

   private final Random random = new Random(232);

   private final Scene3DBuilder scene3DBuilder = new Scene3DBuilder();
   private final MaterialDefinition material;

   public VisualDefinitionFactoryJFXVisualizer(Stage primaryStage)
   {
      primaryStage.setTitle(getClass().getSimpleName());

      Scene scene = new Scene(scene3DBuilder.getRoot(), 600, 400, true, SceneAntialiasing.BALANCED);
      scene.setFill(Color.GREY);
      PerspectiveCameraController cameraController = Simple3DViewer.setupCamera(scene, scene3DBuilder.getRoot());
      cameraController.setFocalPoint(0.0, 0.0, 0.15, false);
      cameraController.setCameraPosition(1.0, 1.0, 0.25);
      scene3DBuilder.addCoordinateSystem(0.25);
      scene3DBuilder.addNodeToView(createAxisLabels());

      if (USE_TEXTURE)
      {
         material = new MaterialDefinition();
         //         material.setDiffuseMap(new TextureDefinition("debuggingTextureGrid.jpg"));
         material.setDiffuseMap(new TextureDefinition("textures/ground_grid.png"));
      }
      else
      {
         material = new MaterialDefinition(ColorDefinitions.Cyan());
      }

      List<? extends Point2DReadOnly> polygon2D = createPolygon2D(0.9, 7);
      List<? extends Point3DReadOnly> polygon3D = toPolygon3D(createPolygon2D(0.9, 7), 0.03, 0.075);
      List<? extends Point2DReadOnly> extrudedPolygon2D = createPolygon2D(0.4, 7);

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.setDefaultMaterial(material);

      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addCoordinateSystem(0.25, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addArrow(Axis3D.Z, 0.3, ColorDefinitions.Tomato(), ColorDefinitions.BlueViolet());

      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      TriangleMesh3DDefinition boxMesh = TriangleMesh3DFactories.Box(100.0, 100.0, 0.1, true);
      for (int i = 0; i < boxMesh.getVertices().length; i++)
      {
         boxMesh.getTextures()[i].set(boxMesh.getVertices()[i].getX(), boxMesh.getVertices()[i].getY());
      }

      //      Box3DDefinition geometry = new Box3DDefinition(100.0, 100.0, 0.3, true);
      factory.addGeometryDefinition(boxMesh, material);

      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addBox(0.1, 0.2, 0.3, false, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addRamp(0.3, 0.2, 0.1, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addSphere(0.15, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addCapsule(0.2, 0.05, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addEllipsoid(0.025, 0.2, 0.1, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addCylinder(0.2, 0.05, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addCone(0.2, 0.1, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addTruncatedCone(0.2, 0.1, 0.04, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addTruncatedCone(0.175, 0.12, 0.075, 0.03, 0.06, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addHemiEllipsoid(0.15, 0.05, 0.225, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addTorus(0.2, 0.025, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addArcTorus(0.25 * Math.PI, 1.75 * Math.PI, 0.2, 0.025, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addPyramidBox(0.15, 0.075, 0.15, 0.1, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addPolygon3D(polygon3D, true, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addPolygon2D(polygon2D, true, material);
      //      factory.getCurrentTransform().setTranslation(nextVisualPosition());
      //      factory.addExtrudedPolygon(extrudedPolygon2D, true, 0.1, material);

      List<VisualDefinition> visualDefinitions = factory.getVisualDefinitions();
      Node node = JavaFXVisualTools.collectNodes(visualDefinitions);
      scene3DBuilder.addNodeToView(node);

      primaryStage.setMaximized(true);
      primaryStage.setScene(scene);
      primaryStage.show();
   }

   public static Node createAxisLabels()
   {
      AxisAngle orientation = new AxisAngle();
      orientation.appendYawRotation(0.75 * Math.PI);
      orientation.appendRollRotation(-0.5 * Math.PI);
      // @formatter:off
      Node xLabel = Simple3DViewer.createLabel("x", Color.RED  , 0.10, 0.001, new Point3D(350,- 75,   0), orientation);
      Node yLabel = Simple3DViewer.createLabel("y", Color.GREEN, 0.10, 0.001, new Point3D( 25, 275,   0), orientation);
      Node zLabel = Simple3DViewer.createLabel("z", Color.BLUE , 0.10, 0.001, new Point3D( 25,- 50, 300), orientation);
      // @formatter:off

      return new Group(xLabel, yLabel, zLabel);
   }

   private List<? extends Point2DReadOnly> createPolygon2D(double maxEdgeLength, int numberOfVertices)
   {
      List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, new Point2D(), maxEdgeLength, numberOfVertices);
      Collections.reverse(vertices);
      return vertices;
   }
   
   private List<? extends Point3DReadOnly> toPolygon3D(List<? extends Point2DReadOnly> polygon2D, double minZ, double maxZ)
   {
      return polygon2D.stream().map(Point3D::new).peek(p -> p.setZ(EuclidCoreRandomTools.nextDouble(random, minZ, maxZ))).collect(Collectors.toList());
   }

   private final int gridWidth = 2;
   private int currentX = 0, currentY = 0;
   private final double cellSize = 0.5;

   private Point3D nextVisualPosition()
   {
      Point3D next;

      if (currentX == 0 && currentY == 0)
         next = new Point3D();
      else
         next = new Point3D(currentX * cellSize, currentY * cellSize, 0.0);

      currentY = nextIndex(currentY);

      if (currentY > gridWidth)
      {
         currentY = 0;
         currentX = nextIndex(currentX);
      }

      return next;
   }

   private static int nextIndex(int index)
   {
      if (index == 0)
         return 1;
      else if (index > 0)
         return -index;
      else
         return -index + 1;
   }

   public static void main(String[] args)
   {
      ApplicationRunner.runApplication(VisualDefinitionFactoryJFXVisualizer::new);
   }
}