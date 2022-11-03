package us.ihmc.scs2.sessionVisualizer.jfx.plotter;

import static us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXItem.YO_GRAPHICFX_ITEM_KEY;

import javafx.animation.AnimationTimer;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableMap;
import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.control.Tooltip;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.Region;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import javafx.util.Duration;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXToEuclidConversions;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXItem;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

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

   private final ObjectProperty<Tooltip> activeTooltip = new SimpleObjectProperty<>(this, "activeTooltipProperty", null);

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

      root.getChildren().addListener(new GroupTooltipHandler());

      activeTooltip.addListener((o, oldValue, newValue) ->
      {
         if (oldValue != null)
            oldValue.hide();
      });
   }

   private class GroupTooltipHandler implements ListChangeListener<Node>
   {
      private static final String TOOLTIP_LISTENER_PROPERTY_KEY = GroupTooltipHandler.class.getName() + " - TooltipListener";
      private final ObservableMap<Node, Tooltip> installedTooltips = FXCollections.observableHashMap();

      @Override
      public void onChanged(Change<? extends Node> change)
      {
         while (change.next())
         {
            for (Node node : change.getAddedSubList())
            {
               installTooltip(node);
            }

            for (Node node : change.getRemoved())
            {
               Tooltip tooltip = installedTooltips.remove(node);

               if (tooltip == null)
               {
                  if (node instanceof Group)
                     ((Group) node).getChildren().removeListener(this);
               }
               else
               {
                  tooltip.textProperty().unbind();
                  Tooltip.uninstall(node, tooltip);
               }
            }
         }
      }

      public void installTooltip(Node node)
      {
         YoGraphicFXItem yoGraphicFX = (YoGraphicFXItem) node.getProperties().get(YO_GRAPHICFX_ITEM_KEY);
         if (yoGraphicFX == null || yoGraphicFX instanceof YoGroupFX)
         {
            if (node instanceof Group group)
            {
               installTooltipsToDescendants(group);
            }
         }
         else
         {
            Tooltip tooltip = new Tooltip(node.getId());
            tooltip.setShowDelay(Duration.millis(200));
            tooltip.setShowDuration(Duration.seconds(20));
            tooltip.textProperty().bind(node.idProperty());

            ChangeListener<Boolean> listener = (o, oldValue, newValue) ->
            {
               if (newValue)
                  activeTooltip.set(tooltip);
            };
            tooltip.getProperties().put(TOOLTIP_LISTENER_PROPERTY_KEY, listener);
            tooltip.showingProperty().addListener(listener);
            ChangeListener<? super Boolean> tooltipShowingListener = (o, oldValue, newValue) ->
            {
               if (newValue)
                  activeTooltip.set(tooltip);
            };
            tooltip.showingProperty().addListener(tooltipShowingListener);
            node.setPickOnBounds(true);
            installedTooltips.put(node, tooltip);
            Tooltip.install(node, tooltip);
         }
      }

      public void installTooltipsToDescendants(Group group)
      {
         for (Node child : group.getChildren())
         {
            installTooltip(child);
         }

         group.getChildren().addListener(this);
      }

      public void uninstallTooltip(Node node)
      {
         Tooltip tooltip = installedTooltips.remove(node);

         if (tooltip == null)
         {
            if (node instanceof Group group)
               uninstallTooltipsFromDescendants(group);
         }
         else
         {
            tooltip.textProperty().unbind();
            @SuppressWarnings("unchecked")
            ChangeListener<Boolean> listener = (ChangeListener<Boolean>) tooltip.getProperties().remove(TOOLTIP_LISTENER_PROPERTY_KEY);
            tooltip.showingProperty().removeListener(listener);
            Tooltip.uninstall(node, tooltip);
         }
      }

      public void uninstallTooltipsFromDescendants(Group group)
      {
         group.getChildren().removeListener(this);

         for (Node child : group.getChildren())
         {
            uninstallTooltip(child);
         }
      }
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
      activeTooltip.set(null);
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
               oldMouseLocation = getMouseSceneLocation(event);
               return;
            }

            if (event.getEventType() != MouseEvent.MOUSE_DRAGGED)
               return;

            Point2D newMouseLocation = getMouseSceneLocation(event);

            if (oldMouseLocation != null)
            {
               Vector2D drag = new Vector2D();
               drag.sub(newMouseLocation, oldMouseLocation);
               JavaFXMissingTools.addEquals(rootTranslation, drag);
               center.set(computeCenterLocal());
               updateGrid();
            }
            oldMouseLocation = newMouseLocation;
            activeTooltip.set(null);
         }
      };
   }

   private Point2D getMouseSceneLocation(MouseEvent event)
   { // The presence of a Tooltip in the plotter appears to cause event.getX() and event.getY() to return Double.NaN
      if (Double.isNaN(event.getX()) || Double.isNaN(event.getY()))
      {
         javafx.geometry.Point2D local = root.screenToLocal(event.getScreenX(), event.getScreenY());
         javafx.geometry.Point2D scene = root.localToScene(local);
         return new Point2D(scene.getX(), scene.getY());
      }
      else
      {
         return new Point2D(event.getX(), event.getY());
      }
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

            javafx.geometry.Point2D preScaleLocation;
            javafx.geometry.Point2D postScaleLocation;

            if (Double.isNaN(trackedLocationX) || Double.isNaN(trackedLocationY))
            {
               // Apparently this can happen with Tooltips in the plotter.
               trackedLocationX = event.getScreenX();
               trackedLocationY = event.getScreenY();
               preScaleLocation = root.screenToLocal(trackedLocationX, trackedLocationY);
               rootScale.setX(newScale);
               rootScale.setY(-newScale);
               postScaleLocation = root.screenToLocal(trackedLocationX, trackedLocationY);
            }
            else
            {
               preScaleLocation = root.sceneToLocal(trackedLocationX, trackedLocationY);
               rootScale.setX(newScale);
               rootScale.setY(-newScale);
               postScaleLocation = root.sceneToLocal(trackedLocationX, trackedLocationY);
            }

            rootTranslation.setX(rootTranslation.getX() + rootScale.getX() * (postScaleLocation.getX() - preScaleLocation.getX()));
            rootTranslation.setY(rootTranslation.getY() + rootScale.getY() * (postScaleLocation.getY() - preScaleLocation.getY()));
            activeTooltip.set(null);

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
