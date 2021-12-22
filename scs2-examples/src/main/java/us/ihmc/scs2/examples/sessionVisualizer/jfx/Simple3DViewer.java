package us.ihmc.scs2.examples.sessionVisualizer.jfx;

import java.util.Arrays;

import org.fxyz3d.shapes.primitives.Text3DMesh;
import org.fxyz3d.shapes.primitives.TexturedMesh;

import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.javaFXToolkit.text.Text3D;

public class Simple3DViewer
{
   public static void view3DObjects(Node... nodesToView)
   {
      view3DObjects(Arrays.asList(nodesToView));
   }

   public static void view3DObjects(Iterable<? extends Node> nodesToView)
   {
      ApplicationRunner.runApplication(primaryStage ->
      {
         View3DFactory view3dFactory = new View3DFactory(600, 400);
         FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);
         cameraController.changeCameraPosition(-1.0, -1.0, 1.0);
         view3dFactory.addNodesToView(nodesToView);
         double ambientValue = 0.2;
         double pointValue = 0.3;
         double pointDistance = 100.0;
         Color ambientColor = Color.color(ambientValue, ambientValue, ambientValue);
         view3dFactory.addNodeToView(new AmbientLight(ambientColor));
         Color indoorColor = Color.color(pointValue, pointValue, pointValue);
         view3dFactory.addPointLight(pointDistance, pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(-pointDistance, pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(-pointDistance, -pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(pointDistance, -pointDistance, pointDistance, indoorColor);

         primaryStage.setMaximized(true);
         primaryStage.setScene(view3dFactory.getScene());
         primaryStage.show();
      });
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
      label.getNode().getTransforms().add(JavaFXTools.createAffineFromOrientation3DAndTuple(orientation, position));

      return label.getNode();
   }
}
