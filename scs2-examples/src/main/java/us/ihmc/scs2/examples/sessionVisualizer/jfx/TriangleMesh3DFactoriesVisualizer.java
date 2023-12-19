package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Shape3D;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.sessionVisualizer.jfx.Scene3DBuilder;

import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import static us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition;

public class TriangleMesh3DFactoriesVisualizer
{
   private static final boolean USE_TEXTURE = false;

   private final Scene3DBuilder scene3DBuilder = new Scene3DBuilder();
   private final PhongMaterial defaultMaterial;

   public TriangleMesh3DFactoriesVisualizer(Stage primaryStage)
   {
      primaryStage.setTitle(getClass().getSimpleName());

      Scene scene = new Scene(scene3DBuilder.getRoot(), 600, 400, true, SceneAntialiasing.BALANCED);
      scene.setFill(Color.GREY);
      Simple3DViewer.setupCamera(scene, scene3DBuilder.getRoot());
      scene3DBuilder.addCoordinateSystem(0.25);
      scene3DBuilder.addNodeToView(Simple3DViewer.createAxisLabels());

      if (USE_TEXTURE)
      {
         defaultMaterial = new PhongMaterial();
         Image image = new Image(getClass().getClassLoader().getResourceAsStream("debuggingTextureGrid.jpg"));
         defaultMaterial.setDiffuseMap(image);
      }
      else
      {
         defaultMaterial = new PhongMaterial(Color.CYAN);
      }

      List<? extends Point2DReadOnly> polygonVertices = createPolygon();

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.ArcTorus(0.0, 1.0 * Math.PI, 0.15, 0.05, 64));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.Cylinder(0.1, 0.3, 64, true));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.FlatRectangle(0.2, 0.3, 0.0));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.Cone(0.3, 0.1, 64));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.Ramp(0.3, 0.2, 0.2));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.Ellipsoid(0.15, 0.05, 0.2, 64, 64));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.ExtrudedPolygon(polygonVertices, 0.1));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.Line(-0.05, -0.15, 0.2, 0.25, 0.20, -0.10, 0.01));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.PyramidBox(0.1, 0.2, 0.1, 0.25));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.Capsule(0.2, 0.025, 0.05, 0.07, 16, 16));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.TruncatedCone(0.25, 0.2, 0.07, 0.05, 0.1, 64, false));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.HemiEllipsoid(0.10, 0.20, 0.2, 64, 64));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.Tetrahedron(0.3));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.PolygonCounterClockwise(null, polygonVertices));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.ConvexPolytope(EuclidPolytopeFactories.newIcosahedron(0.2)));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.Box(0.1, 0.2, 0.3, true));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.PolygonCounterClockwise(polygonVertices.stream().map(p2D ->
                                                                                                                      {
                                                                                                                         Point3D p3D = new Point3D(p2D);
                                                                                                                         p3D.setZ(0.1 * (Math.random() - 0.5));
                                                                                                                         return p3D;
                                                                                                                      }).collect(Collectors.toList())));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.Torus(0.15, 0.05, 64));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.toSTPBox3DMesh(null, 0.3, 0.2, 0.1, 0.01, 2.5, true));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.toSTPCylinder3DMesh(null, 0.05, 0.3, 0.005, 1.135, true));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.toSTPCapsule3DMesh(null, 0.05, 0.3, 0.055, 0.8125, true));

      addTriangleMesh3DDefinitionToScene(TriangleMesh3DFactories.toSTPRamp3DMesh(null, 0.3, 0.2, 0.1, 0.01, 2.5, true));

      primaryStage.setMaximized(true);
      primaryStage.setScene(scene);
      primaryStage.show();
   }

   private List<? extends Point2DReadOnly> createPolygon()
   {
      ConvexPolygon2D polygon = EuclidGeometryRandomTools.nextConvexPolygon2D(new Random(), 0.3, 10);
      List<Point2D> vertices = polygon.getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList());
      Collections.reverse(vertices);
      return vertices;
   }

   private void addTriangleMesh3DDefinitionToScene(TriangleMesh3DDefinition definition)
   {
      addShape3DToScene(new MeshView(interpretDefinition(definition, false)));
   }

   private void addShape3DToScene(Shape3D node)
   {
      Translate translate = nextNodePosition();

      node.setMaterial(defaultMaterial);

      Node coordinateSystem = Scene3DBuilder.coordinateSystem(0.3);

      if (translate != null)
      {
         node.getTransforms().add(translate);
         coordinateSystem.getTransforms().add(translate);
         scene3DBuilder.addNodeToView(coordinateSystem);
      }

      scene3DBuilder.addNodeToView(node);
   }

   private final int gridWidth = 2;
   private int currentX = 0, currentY = 0;
   private final double cellSize = 0.5;

   private Translate nextNodePosition()
   {
      Translate next;
      if (currentX == 0 && currentY == 0)
         next = null;
      else
         next = new Translate(currentX * cellSize, currentY * cellSize);

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
      ApplicationRunner.runApplication(TriangleMesh3DFactoriesVisualizer::new);
   }
}