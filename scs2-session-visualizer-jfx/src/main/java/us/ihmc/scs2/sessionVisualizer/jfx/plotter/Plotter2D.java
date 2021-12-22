package us.ihmc.scs2.sessionVisualizer.jfx.plotter;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.Region;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXToEuclidConversions;

public class Plotter2D extends Region
{
   private final Group root = new Group();

   private final PlotterGrid2D grid2D = new PlotterGrid2D(root.localToSceneTransformProperty());

   private final Translate rootTranslation = new Translate();
   private final ObjectProperty<MouseButton> mouseButtonForTranslation = new SimpleObjectProperty<>(this, "mouseButtonForTranslation", MouseButton.PRIMARY);
   private final Scale rootScale = new Scale(1.0, -1.0);
   private final DoubleProperty scaleModifier = new SimpleDoubleProperty(this, "scaleModifier", -0.0025);
   private final DoubleProperty minScale = new SimpleDoubleProperty(this, "minScale", 0.01);

   private final Point2D center = new Point2D();

   public Plotter2D()
   {
      getChildren().add(grid2D.getRoot());
      getChildren().add(root);
      root.getTransforms().addAll(rootTranslation, rootScale);

      addEventHandler(MouseEvent.ANY, createTranslationEventHandler());
      addEventHandler(ScrollEvent.ANY, createScaleEventHandler());
      requestParentLayout();
      setManaged(false);
   }

   public void setScale(double scale)
   {
      rootScale.setX(scale);
      rootScale.setY(-scale);
   }

   public void setFieldOfView(double centerX, double centerY, double rangeX, double rangeY)
   {
      setFieldOfView(centerX, centerY, rangeX, rangeY, true);
   }

   public void setFieldOfView(double centerX, double centerY, double rangeX, double rangeY, boolean axisEquals)
   {
      double top = snappedTopInset();
      double left = snappedLeftInset();
      double bottom = snappedBottomInset();
      double right = snappedRightInset();
      double contentWidth = getWidth() - (left + right);
      double contentHeight = getHeight() - (top + bottom);

      double scaleX = contentWidth / rangeX;
      double scaleY = contentHeight / rangeY;

      if (axisEquals)
      {
         double scale = Math.min(scaleX, scaleY);
         scaleX = scale;
         scaleY = scale;
      }

      rootScale.setX(scaleX);
      rootScale.setY(-scaleY);

      rootTranslation.setX(0.5 * contentWidth - centerX * rootScale.getX());
      rootTranslation.setY(0.5 * contentHeight - centerY * rootScale.getY());
   }

   public EventHandler<MouseEvent> createTranslationEventHandler()
   {
      return new EventHandler<MouseEvent>()
      {
         Point2D oldMouseLocation;

         @Override
         public void handle(MouseEvent event)
         {
            if (event.getButton() != mouseButtonForTranslation.get())
               return;

            if (event.isStillSincePress())
               return;

            if (event.getEventType() == MouseEvent.MOUSE_PRESSED)
            {
               oldMouseLocation = new Point2D(event.getX(), event.getY());
               return;
            }

            if (event.getEventType() != MouseEvent.MOUSE_DRAGGED)
               return;

            Point2D newMouseLocation = new Point2D(event.getX(), event.getY());

            if (oldMouseLocation != null)
            {
               Vector2D drag = new Vector2D();
               drag.sub(newMouseLocation, oldMouseLocation);
               JavaFXMissingTools.addEquals(rootTranslation, drag);
               center.set(computeCenterLocal());
               updateGrid();
            }
            oldMouseLocation = newMouseLocation;
         }
      };
   }

   public EventHandler<ScrollEvent> createScaleEventHandler()
   {
      return new EventHandler<ScrollEvent>()
      {
         @Override
         public void handle(ScrollEvent event)
         {

            double verticalDrag = event.getDeltaY();
            double oldScale = rootScale.getX();
            double newScale = oldScale * (1.0 - scaleModifier.get() * verticalDrag);
            newScale = Math.max(minScale.get(), newScale);

            javafx.geometry.Point2D preScaleLocation = root.sceneToLocal(event.getX(), event.getY());
            rootScale.setX(newScale);
            rootScale.setY(-newScale);
            javafx.geometry.Point2D postScaleLocation = root.sceneToLocal(event.getX(), event.getY());

            rootTranslation.setX(rootTranslation.getX() + rootScale.getX() * (postScaleLocation.getX() - preScaleLocation.getX()));
            rootTranslation.setY(rootTranslation.getY() + rootScale.getY() * (postScaleLocation.getY() - preScaleLocation.getY()));

            center.set(computeCenterLocal());
            updateGrid();
         }
      };
   }

   private Point2D computeCenterLocal()
   {
      double top = snappedTopInset();
      double left = snappedLeftInset();
      double bottom = snappedBottomInset();
      double right = snappedRightInset();
      double contentWidth = getWidth() - (left + right);
      double contentHeight = getHeight() - (top + bottom);

      Point2D newCenter = JavaFXToEuclidConversions.convertPoint2D(root.sceneToLocal(0.5 * contentWidth, 0.5 * contentHeight));
      return newCenter;
   }

   public void updateGrid()
   {
      double top = snappedTopInset();
      double left = snappedLeftInset();
      double bottom = snappedBottomInset();
      double right = snappedRightInset();
      double contentWidth = getWidth() - (left + right);
      double contentHeight = getHeight() - (top + bottom);
      grid2D.update(top, left, contentWidth, contentHeight);
   }

   @Override
   protected void layoutChildren()
   {
      recenter();
      updateGrid();
   }

   private void recenter()
   {
      Vector2D translation = new Vector2D();
      translation.sub(computeCenterLocal(), center);
      translation.scale(rootScale.getX(), rootScale.getY());
      JavaFXMissingTools.addEquals(rootTranslation, translation);
   }

   public ObjectProperty<MouseButton> mouseButtonForTranslationProperty()
   {
      return mouseButtonForTranslation;
   }

   public Group getRoot()
   {
      return root;
   }

   public PlotterGrid2D getGrid2D()
   {
      return grid2D;
   }
}
