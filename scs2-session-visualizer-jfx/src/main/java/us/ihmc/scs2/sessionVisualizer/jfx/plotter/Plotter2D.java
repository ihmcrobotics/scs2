package us.ihmc.scs2.sessionVisualizer.jfx.plotter;

import javafx.animation.AnimationTimer;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
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
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;

public class Plotter2D extends Region
{
   private final Group root = new Group();

   private final PlotterGrid2D grid2D = new PlotterGrid2D(root.localToSceneTransformProperty());

   private final Translate rootTranslation = new Translate();
   private final Translate trackingTranslation = new Translate();
   private final ObjectProperty<MouseButton> mouseButtonForTranslation = new SimpleObjectProperty<>(this, "mouseButtonForTranslation", MouseButton.PRIMARY);
   private final Scale rootScale = new Scale(1.0, -1.0);
   private final DoubleProperty scaleModifier = new SimpleDoubleProperty(this, "scaleModifier", -0.0025);
   private final DoubleProperty minScale = new SimpleDoubleProperty(this, "minScale", 0.01);

   private final Point2D center = new Point2D();

   private final Property<Tuple2DProperty> coordinateToTrack = new SimpleObjectProperty<>(this, "coordinateToTrackProperty", null);
   private final AnimationTimer trackingAnimation = new AnimationTimer()
   {
      private boolean initialized = false;

      @Override
      public void start()
      {
         initialized = false;
         super.start();
      }

      @Override
      public void handle(long now)
      {
         Tuple2DProperty coordinateProperty = coordinateToTrack.getValue();
         if (coordinateProperty == null)
         {
            stop();
            return;
         }

         Point2D coordinateInWorld = coordinateProperty.toPoint2DInWorld();

         if (coordinateInWorld.containsNaN())
            return;

         if (!initialized)
         {
            rootTranslation.setX(0.5 * contentWidth());
            rootTranslation.setY(0.5 * contentHeight());
            trackingTranslation.setX(-coordinateInWorld.getX());
            trackingTranslation.setY(-coordinateInWorld.getY());
            center.set(coordinateInWorld);
            initialized = true;
         }
         else
         {
            trackingTranslation.setX(-coordinateInWorld.getX());
            trackingTranslation.setY(-coordinateInWorld.getY());
            center.set(computeCenterLocal());
         }
      }
   };

   public Plotter2D()
   {
      getChildren().add(grid2D.getRoot());
      getChildren().add(root);
      root.getTransforms().addAll(rootTranslation, rootScale, trackingTranslation);

      addEventHandler(MouseEvent.ANY, createTranslationEventHandler());
      addEventHandler(ScrollEvent.ANY, createScaleEventHandler());
      requestParentLayout();
      setManaged(false);

      coordinateToTrack.addListener((o, oldValue, newValue) ->
      {
         trackingAnimation.stop();

         if (oldValue != null)
         {
            trackingTranslation.setX(0.0);
            trackingTranslation.setY(0.0);
         }

         if (newValue != null)
         {
            trackingAnimation.start();
         }
      });
   }

   public Property<Tuple2DProperty> coordinateToTrackProperty()
   {
      return coordinateToTrack;
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
      double contentWidth = contentWidth();
      double contentHeight = contentHeight();

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
      return new EventHandler<>()
      {
         Point2D oldMouseLocation;

         @Override
         public void handle(MouseEvent event)
         {
            if ((event.getButton() != mouseButtonForTranslation.get()) || event.isStillSincePress())
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
      return new EventHandler<>()
      {
         @Override
         public void handle(ScrollEvent event)
         {
            double verticalDrag = event.getDeltaY();
            double oldScale = rootScale.getX();
            double newScale = oldScale * (1.0 - scaleModifier.get() * verticalDrag);
            newScale = Math.max(minScale.get(), newScale);

            double trackedLocationX = event.getX();
            double trackedLocationY = event.getY();

            javafx.geometry.Point2D preScaleLocation = root.sceneToLocal(trackedLocationX, trackedLocationY);
            rootScale.setX(newScale);
            rootScale.setY(-newScale);
            javafx.geometry.Point2D postScaleLocation = root.sceneToLocal(trackedLocationX, trackedLocationY);

            rootTranslation.setX(rootTranslation.getX() + rootScale.getX() * (postScaleLocation.getX() - preScaleLocation.getX()));
            rootTranslation.setY(rootTranslation.getY() + rootScale.getY() * (postScaleLocation.getY() - preScaleLocation.getY()));

            center.set(computeCenterLocal());
            updateGrid();
         }
      };
   }

   private Point2D computeCenterLocal()
   {
      return JavaFXToEuclidConversions.convertPoint2D(root.sceneToLocal(0.5 * contentWidth(), 0.5 * contentHeight()));
   }

   public void updateGrid()
   {
      grid2D.update(snappedTopInset(), snappedLeftInset(), contentWidth(), contentHeight());
   }

   private double contentWidth()
   {
      return getWidth() - (snappedLeftInset() + snappedRightInset());
   }

   private double contentHeight()
   {
      return getHeight() - (snappedTopInset() + snappedBottomInset());
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
