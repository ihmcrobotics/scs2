package us.ihmc.scs2.sessionVisualizer.jfx;

import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.PointLight;
import javafx.scene.paint.Color;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;

public class Scene3DBuilder
{
   private final Group root = new Group();

   public Scene3DBuilder()
   {
   }

   /**
    * Adds some nice ambient and point lighting to the scene.
    */
   public void addDefaultLighting()
   {
      double ambientValue = 0.7;
      double pointValue = 0.2;
      double pointDistance = 1000.0;
      Color ambientColor = Color.color(ambientValue, ambientValue, ambientValue);
      addNodeToView(new AmbientLight(ambientColor));
      Color indoorColor = Color.color(pointValue, pointValue, pointValue);
      addPointLight(pointDistance, pointDistance, pointDistance, indoorColor);
      addPointLight(-pointDistance, pointDistance, pointDistance, indoorColor);
      addPointLight(-pointDistance, -pointDistance, pointDistance, indoorColor);
      addPointLight(pointDistance, -pointDistance, pointDistance, indoorColor);
   }

   /**
    * Add a {@link Color#WHITE} point light to the 3D view at the given coordinate.
    * 
    * @param x the light x-coordinate.
    * @param y the light y-coordinate.
    * @param z the light z-coordinate.
    */
   public void addPointLight(double x, double y, double z)
   {
      addPointLight(x, y, z, Color.WHITE);
   }

   /**
    * Add a point light to the 3D view at the given coordinate.
    * 
    * @param x     the light x-coordinate.
    * @param y     the light y-coordinate.
    * @param z     the light z-coordinate.
    * @param color the light color.
    */
   public void addPointLight(double x, double y, double z, Color color)
   {
      PointLight light = new PointLight(color);
      light.setTranslateX(x);
      light.setTranslateY(y);
      light.setTranslateZ(z);
      addNodeToView(light);
   }

   /**
    * Display the world coordinate system.
    * 
    * @param arrowLength length of each axis of the coordinate system.
    */
   public void addWorldCoordinateSystem(double arrowLength)
   {
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(arrowLength);
      worldCoordinateSystem.setMouseTransparent(true);
      addNodeToView(worldCoordinateSystem);
   }

   /**
    * Add a set of nodes to the 3D view.
    * 
    * @param nodes the nodes to display.
    */
   public void addNodesToView(Iterable<? extends Node> nodes)
   {
      nodes.forEach(this::addNodeToView);
   }

   /**
    * Add a node to the 3D view.
    * 
    * @param node the node to display.
    */
   public void addNodeToView(Node node)
   {
      root.getChildren().add(node);
   }

   /**
    * If true, this node (together with all its children) is completely transparent to mouse events.
    * When choosing target for mouse event, nodes with {@code mouseTransparent} set to {@code true} and
    * their subtrees won't be taken into account greatly reducing computation load especially when
    * rendering many nodes.
    * 
    * @param value whether to make the entire scene mouse transparent or not.
    */
   public void setRootMouseTransparent(boolean value)
   {
      root.setMouseTransparent(value);
   }

   /**
    * @return the root node of the scene in creation.
    */
   public Group getRoot()
   {
      return root;
   }
}
