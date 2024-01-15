package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import javafx.event.Event;
import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import org.fxyz3d.shapes.primitives.Text3DMesh;
import org.fxyz3d.shapes.primitives.TexturedMesh;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.scs2.sessionVisualizer.jfx.Scene3DBuilder;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.PerspectiveCameraController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

import java.util.Arrays;
import java.util.Collection;

public class Simple3DViewer
{
   public static void view3DObjects(Node... nodesToView)
   {
      view3DObjects(Arrays.asList(nodesToView));
   }

   public static void view3DObjects(Collection<? extends Node> nodesToView)
   {
      ApplicationRunner.runApplication(primaryStage ->
                                       {
                                          Scene3DBuilder scene3DBuilder = new Scene3DBuilder();
                                          Scene scene = new Scene(scene3DBuilder.getRoot(), 600, 400, true, SceneAntialiasing.BALANCED);
                                          scene.setFill(Color.GREY);
                                          setupCamera(scene, scene3DBuilder.getRoot());
                                          scene3DBuilder.addNodesToView(nodesToView);
                                          double ambientValue = 0.2;
                                          double pointValue = 0.3;
                                          double pointDistance = 100.0;
                                          Color ambientColor = Color.color(ambientValue, ambientValue, ambientValue);
                                          scene3DBuilder.addNodeToView(new AmbientLight(ambientColor));
                                          Color indoorColor = Color.color(pointValue, pointValue, pointValue);
                                          scene3DBuilder.addPointLight(pointDistance, pointDistance, pointDistance, indoorColor);
                                          scene3DBuilder.addPointLight(-pointDistance, pointDistance, pointDistance, indoorColor);
                                          scene3DBuilder.addPointLight(-pointDistance, -pointDistance, pointDistance, indoorColor);
                                          scene3DBuilder.addPointLight(pointDistance, -pointDistance, pointDistance, indoorColor);

                                          primaryStage.setMaximized(true);
                                          primaryStage.setScene(scene);
                                          primaryStage.show();
                                       });
   }

   public static PerspectiveCameraController setupCamera(Scene scene, Group root)
   {
      PerspectiveCamera camera = new PerspectiveCamera(true);
      camera.setNearClip(0.05);
      camera.setFarClip(100000.0);
      scene.setCamera(camera);
      PerspectiveCameraController cameraController = new PerspectiveCameraController(scene.widthProperty(), scene.heightProperty(), camera, Axis3D.Z, Axis3D.X);
      cameraController.setCameraPosition(-1.0, -1.0, 1.0);
      cameraController.enableShiftClickFocusTranslation();
      cameraController.start();
      scene.addEventHandler(Event.ANY, cameraController);

      Sphere focusPointViz = cameraController.getFocalPointViz();
      if (focusPointViz != null)
      {
         root.getChildren().add(focusPointViz);
         focusPointViz.visibleProperty().bind(scene.getCamera().focusedProperty());
      }
      return cameraController;
   }

   public static Node createAxisLabels()
   {
      AxisAngle xAxisAngle = new AxisAngle();
      xAxisAngle.appendYawRotation(-0.25 * Math.PI);
      xAxisAngle.appendRollRotation(-0.5 * Math.PI);
      Node xLabel = Simple3DViewer.createLabel("x", Color.RED, 0.10, 0.001, new Point3D(300, 0, 25), xAxisAngle);

      AxisAngle yAxisAngle = new AxisAngle();
      yAxisAngle.appendYawRotation(-0.25 * Math.PI);
      yAxisAngle.appendRollRotation(-0.5 * Math.PI);
      Node yLabel = Simple3DViewer.createLabel("y", Color.GREEN, 0.10, 0.001, new Point3D(0, 300, 25), yAxisAngle);

      AxisAngle zAxisAngle = new AxisAngle();
      zAxisAngle.appendYawRotation(-0.25 * Math.PI);
      zAxisAngle.appendRollRotation(-0.5 * Math.PI);
      Node zLabel = Simple3DViewer.createLabel("z", Color.BLUE, 0.10, 0.001, new Point3D(0, 0, 325), zAxisAngle);
      return new Group(xLabel, yLabel, zLabel);
   }

   public static Node createLabel(String text, Color color, double height, double thickness, Point3DReadOnly position, Orientation3DReadOnly orientation)
   {
      Text3D label = new Text3D(text);
      label.setFontHeight(height);
      label.setFontThickness(thickness);
      ((Text3DMesh) label.getNode()).getChildren().forEach(child -> ((TexturedMesh) child).setMaterial(new PhongMaterial(color)));
      label.getNode().getTransforms().add(JavaFXMissingTools.createAffineFromOrientation3DAndTuple(orientation, position));

      return label.getNode();
   }
}
