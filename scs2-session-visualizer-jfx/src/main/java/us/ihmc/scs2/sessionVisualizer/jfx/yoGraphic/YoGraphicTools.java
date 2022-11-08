package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.Shape;
import javafx.scene.shape.Shape3D;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.SingularValueDecomposition3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrudedPolygon2DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Point3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.STPBox3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoListDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicArrow3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicBox3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCapsule3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCone3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCoordinateSystem3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCylinder3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicEllipsoid3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicLine2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicListDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPointcloud2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPointcloud3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolygon2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolygonExtruded3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolynomial3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicRamp3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicSTPBox3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.YoVariableDatabase;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.QuaternionProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;

public class YoGraphicTools
{
   public static final String GUI_ROOT_NAME = "root";
   public static final String SESSION_ROOT_NAME = "session";
   public static final String SEPARATOR = ":";

   public static List<String> collectAllExistingNamespaces(YoGroupFX group)
   {
      List<String> namespaces = new ArrayList<>();
      if (!group.isRoot())
         namespaces.add(group.getFullname());
      for (YoGroupFX child : group.getChildren())
         namespaces.addAll(collectAllExistingNamespaces(child));
      return namespaces;
   }

   public static boolean yoGraphicFXGroupContainsItem(String itemName, Class<? extends YoGraphicFXItem> itemType, YoGroupFX group)
   {
      if (YoGraphicFX2D.class.isAssignableFrom(itemType))
         return group.containsYoGraphicFX2D(itemName);
      else if (YoGraphicFX3D.class.isAssignableFrom(itemType))
         return group.containsYoGraphicFX3D(itemName);
      else if (YoGroupFX.class.isAssignableFrom(itemType))
         return group.containsChild(itemName);
      else
         throw new RuntimeException("Unexpected item type: " + itemType.getSimpleName());
   }

   public static <I extends YoGraphicFXItem> I findYoGraphicFXItem(YoGroupFX root, String namespace, String itemName, Class<I> itemType)
   {
      YoGroupFX graphicGroup = findYoGraphicFXGroup(root, namespace);
      if (graphicGroup == null)
         return null;
      else
      {
         if (YoGraphicFX2D.class.isAssignableFrom(itemType))
            return itemType.cast(graphicGroup.getYoGraphicFX2D(itemName));
         else if (YoGraphicFX3D.class.isAssignableFrom(itemType))
            return itemType.cast(graphicGroup.getYoGraphicFX3D(itemName));
         else if (YoGroupFX.class.isAssignableFrom(itemType))
            return itemType.cast(graphicGroup.getChild(itemName));
         else
            throw new RuntimeException("Unexpected item type: " + itemType.getSimpleName());
      }
   }

   public static YoGraphicFX2D findYoGraphicFX2D(YoGroupFX root, String namespace, String graphicName)
   {
      YoGroupFX graphicGroup = findYoGraphicFXGroup(root, namespace);
      if (graphicGroup == null)
         return null;
      else
         return graphicGroup.getYoGraphicFX2D(graphicName);
   }

   public static YoGraphicFX3D findYoGraphicFX3D(YoGroupFX root, String namespace, String graphicName)
   {
      YoGroupFX graphicGroup = findYoGraphicFXGroup(root, namespace);
      if (graphicGroup == null)
         return null;
      else
         return graphicGroup.getYoGraphicFX3D(graphicName);
   }

   public static YoGroupFX findYoGraphicFXGroup(YoGroupFX root, String namespace)
   {
      String[] groupNames;
      if (namespace.contains(YoGraphicTools.SEPARATOR))
         groupNames = namespace.split(YoGraphicTools.SEPARATOR);
      else
         groupNames = new String[] {namespace};

      if (groupNames == null || groupNames.length == 0)
         return null;

      int startIndex = 1;

      if (!groupNames[0].equals(root.getName()))
         startIndex = 0;

      YoGroupFX parent = root;

      for (int i = startIndex; i < groupNames.length; i++)
      {
         String groupName = groupNames[i];
         YoGroupFX child = parent.getChild(groupName);

         if (child == null)
            return null;

         parent = child;
      }

      return parent;
   }

   public static YoGroupFX findOrCreateYoGraphicFXGroup(YoGroupFX root, String namespace)
   {
      String[] groupNames;
      if (namespace.contains(YoGraphicTools.SEPARATOR))
         groupNames = namespace.split(YoGraphicTools.SEPARATOR);
      else
         groupNames = new String[] {namespace};

      if (groupNames == null || groupNames.length == 0)
         return null;

      int startIndex = groupNames[0].equals(root.getName()) ? 1 : 0;

      YoGroupFX parent = root;

      for (int i = startIndex; i < groupNames.length; i++)
      {
         String groupName = groupNames[i];
         YoGroupFX child = parent.getChild(groupName);

         if (child == null)
         {
            child = new YoGroupFX(groupName);
            parent.addChild(child);
         }

         parent = child;
      }

      return parent;
   }

   public static List<YoGraphicFXItem> createYoGraphicFXs(YoVariableDatabase yoVariableDatabase,
                                                          YoGroupFX parentGroup,
                                                          YoGraphicFXResourceManager resourceManager,
                                                          ReferenceFrameManager referenceFrameManager,
                                                          YoGraphicListDefinition yoGraphicListDefinition)
   {
      if (yoGraphicListDefinition.getYoGraphics() == null)
         return null;

      List<YoGraphicFXItem> items = new ArrayList<>();

      for (YoGraphicDefinition definition : yoGraphicListDefinition.getYoGraphics())
      {
         YoGraphicFXItem item = createYoGraphicFX(yoVariableDatabase, parentGroup, resourceManager, referenceFrameManager, definition);
         if (item != null)
            items.add(item);
      }

      return items;
   }

   public static YoGraphicFXItem createYoGraphicFX(YoVariableDatabase yoVariableDatabase,
                                                   YoGroupFX parentGroup,
                                                   YoGraphicFXResourceManager resourceManager,
                                                   ReferenceFrameManager referenceFrameManager,
                                                   YoGraphicDefinition yoGraphicDefinition)
   {
      YoGraphicFXItem yoGraphicFX = toYoGraphicFX(yoVariableDatabase, resourceManager, referenceFrameManager, yoGraphicDefinition);

      if (yoGraphicFX == null)
      {
         LogTools.error("Failed to load YoGraphicDefinition: " + yoGraphicDefinition);
         return null;
      }

      boolean isValidName;
      if (yoGraphicFX instanceof YoGroupFX)
         isValidName = !parentGroup.containsChild(yoGraphicFX.getName());
      else if (yoGraphicFX instanceof YoGraphicFX2D)
         isValidName = !parentGroup.containsYoGraphicFX2D(yoGraphicFX.getName());
      else if (yoGraphicFX instanceof YoGraphicFX3D)
         isValidName = !parentGroup.containsYoGraphicFX3D(yoGraphicFX.getName());
      else
         throw new RuntimeException("The following " + YoGraphicFX.class.getSimpleName() + " has unexpected hierarchy "
               + yoGraphicFX.getClass().getSimpleName());

      if (!isValidName)
         yoGraphicFX.setName(YoGraphicFXControllerTools.createAvailableYoGraphicFXItemName(parentGroup, yoGraphicFX.getName(), yoGraphicFX.getClass()));

      return yoGraphicFX;
   }

   public static YoGraphicFXItem toYoGraphicFX(YoVariableDatabase yoVariableDatabase,
                                               YoGraphicFXResourceManager resourceManager,
                                               ReferenceFrameManager referenceFrameManager,
                                               YoGraphicDefinition definition)
   {
      try
      {
         if (definition instanceof YoGraphicGroupDefinition)
            return toYoGroupFX(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicGroupDefinition) definition);
         else if (definition instanceof YoGraphicPoint2DDefinition)
            return toYoPointFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicPoint2DDefinition) definition);
         else if (definition instanceof YoGraphicPolygon2DDefinition)
            return toYoGraphicPolygonFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicPolygon2DDefinition) definition);
         else if (definition instanceof YoGraphicPointcloud2DDefinition)
            return toYoPointcloudFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicPointcloud2DDefinition) definition);
         else if (definition instanceof YoGraphicLine2DDefinition)
            return toYoLineFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicLine2DDefinition) definition);
         else if (definition instanceof YoGraphicPoint3DDefinition)
            return toYoPointFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicPoint3DDefinition) definition);
         else if (definition instanceof YoGraphicArrow3DDefinition)
            return toYoArrowFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicArrow3DDefinition) definition);
         else if (definition instanceof YoGraphicCapsule3DDefinition)
            return toYoCapsuleFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicCapsule3DDefinition) definition);
         else if (definition instanceof YoGraphicCone3DDefinition)
            return toYoConeFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicCone3DDefinition) definition);
         else if (definition instanceof YoGraphicCylinder3DDefinition)
            return toYoCylinderFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicCylinder3DDefinition) definition);
         else if (definition instanceof YoGraphicPointcloud3DDefinition)
            return toYoPointcloudFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicPointcloud3DDefinition) definition);
         else if (definition instanceof YoGraphicPolynomial3DDefinition)
            return toYoPolynomialFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicPolynomial3DDefinition) definition);
         else if (definition instanceof YoGraphicCoordinateSystem3DDefinition)
            return toYoCoordinateSystemFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicCoordinateSystem3DDefinition) definition);
         else if (definition instanceof YoGraphicPolygonExtruded3DDefinition)
            return toYoPolygonExtrudedFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicPolygonExtruded3DDefinition) definition);
         else if (definition instanceof YoGraphicConvexPolytope3DDefinition)
            return toYoConvexPolytopeFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicConvexPolytope3DDefinition) definition);
         else if (definition instanceof YoGraphicBox3DDefinition)
            return toYoBoxFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicBox3DDefinition) definition);
         else if (definition instanceof YoGraphicSTPBox3DDefinition)
            return toYoSTPBoxFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicSTPBox3DDefinition) definition);
         else if (definition instanceof YoGraphicEllipsoid3DDefinition)
            return toYoEllipsoidFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, (YoGraphicEllipsoid3DDefinition) definition);

         LogTools.error("Unhandled graphic type: {}", definition.getClass().getSimpleName());
         return null;
      }
      catch (NullPointerException e)
      {
         LogTools.error("Could not load: " + definition.getClass().getSimpleName() + ", " + definition.getName() + ", reason: " + e.getMessage());
         return null;
      }
   }

   private static void toYoGraphicFX(YoVariableDatabase yoVariableDatabase,
                                     YoGraphicFXResourceManager resourceManager,
                                     ReferenceFrameManager referenceFrameManager,
                                     YoGraphicDefinition definition,
                                     YoGraphicFX yoGraphicFXToPack)
   {
      yoGraphicFXToPack.setName(definition.getName());
      yoGraphicFXToPack.setVisible(definition.isVisible());
   }

   private static void toYoGraphicFX2D(YoVariableDatabase yoVariableDatabase,
                                       YoGraphicFXResourceManager resourceManager,
                                       ReferenceFrameManager referenceFrameManager,
                                       YoGraphic2DDefinition definition,
                                       YoGraphicFX2D yoGraphicFXToPack)
   {
      toYoGraphicFX(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      Color fillColor = JavaFXVisualTools.toColor(definition.getFillColor(), null);
      if (fillColor != null)
         yoGraphicFXToPack.setFillColor(fillColor);
      else
         yoGraphicFXToPack.setStrokeColor(JavaFXVisualTools.toColor(definition.getStrokeColor()));
      yoGraphicFXToPack.setStrokeWidth(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getStrokeWidth()));
   }

   public static YoGroupFX toYoGroupFX(YoVariableDatabase yoVariableDatabase,
                                       YoGraphicFXResourceManager resourceManager,
                                       ReferenceFrameManager referenceFrameManager,
                                       YoGraphicGroupDefinition definition)
   {
      YoGroupFX yoGroupFX = new YoGroupFX(definition.getName());
      yoGroupFX.setVisible(definition.isVisible());
      for (YoGraphicDefinition child : definition.getChildren())
      {
         if (child == null)
            continue;

         YoGraphicFXItem yoGraphicFX = toYoGraphicFX(yoVariableDatabase, resourceManager, referenceFrameManager, child);
         if (yoGraphicFX != null)
            yoGroupFX.addYoGraphicFXItem(yoGraphicFX);
      }
      // If the group definition is not visible, we override the descendants to not be visible by setting the visible property after they were added.. 
      // Without this, when adding a visible item, the group visible property would be overridden.
      if (!definition.isVisible())
         yoGroupFX.setVisible(false);
      return yoGroupFX;
   }

   public static YoPointFX2D toYoPointFX2D(YoVariableDatabase yoVariableDatabase,
                                           YoGraphicFXResourceManager resourceManager,
                                           ReferenceFrameManager referenceFrameManager,
                                           YoGraphicPoint2DDefinition definition)
   {
      YoPointFX2D yoGraphicFX = new YoPointFX2D();
      toYoPointFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoPointFX2D(YoVariableDatabase yoVariableDatabase,
                                    YoGraphicFXResourceManager resourceManager,
                                    ReferenceFrameManager referenceFrameManager,
                                    YoGraphicPoint2DDefinition definition,
                                    YoPointFX2D yoGraphicFXToPack)
   {
      toYoGraphicFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPosition(CompositePropertyTools.toTuple2DProperty(yoVariableDatabase, referenceFrameManager, definition.getPosition()));
      yoGraphicFXToPack.setSize(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getSize()));
      yoGraphicFXToPack.setGraphicResource(resourceManager.loadGraphic2DResource(definition.getGraphicName()));
   }

   public static YoPolygonFX2D toYoGraphicPolygonFX2D(YoVariableDatabase yoVariableDatabase,
                                                      YoGraphicFXResourceManager resourceManager,
                                                      ReferenceFrameManager referenceFrameManager,
                                                      YoGraphicPolygon2DDefinition definition)
   {
      YoPolygonFX2D yoGraphicFX = new YoPolygonFX2D();
      toYoGraphicPolygonFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoGraphicPolygonFX2D(YoVariableDatabase yoVariableDatabase,
                                             YoGraphicFXResourceManager resourceManager,
                                             ReferenceFrameManager referenceFrameManager,
                                             YoGraphicPolygon2DDefinition definition,
                                             YoPolygonFX2D yoGraphicFXToPack)
   {
      toYoGraphicFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setVertices(CompositePropertyTools.toTuple2DPropertyList(yoVariableDatabase, referenceFrameManager, definition.getVertices()));
      yoGraphicFXToPack.setNumberOfVertices(CompositePropertyTools.toIntegerProperty(yoVariableDatabase, definition.getNumberOfVertices()));
   }

   public static YoPointcloudFX2D toYoPointcloudFX2D(YoVariableDatabase yoVariableDatabase,
                                                     YoGraphicFXResourceManager resourceManager,
                                                     ReferenceFrameManager referenceFrameManager,
                                                     YoGraphicPointcloud2DDefinition definition)
   {
      YoPointcloudFX2D yoGraphicFX = new YoPointcloudFX2D();
      toYoPointcloudFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoPointcloudFX2D(YoVariableDatabase yoVariableDatabase,
                                         YoGraphicFXResourceManager resourceManager,
                                         ReferenceFrameManager referenceFrameManager,
                                         YoGraphicPointcloud2DDefinition definition,
                                         YoPointcloudFX2D yoGraphicFXToPack)
   {
      toYoGraphicFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPoints(CompositePropertyTools.toTuple2DPropertyList(yoVariableDatabase, referenceFrameManager, definition.getPoints()));
      yoGraphicFXToPack.setNumberOfPoints(CompositePropertyTools.toIntegerProperty(yoVariableDatabase, definition.getNumberOfPoints()));
      yoGraphicFXToPack.setSize(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getSize()));
      yoGraphicFXToPack.setGraphicResource(resourceManager.loadGraphic2DResource(definition.getGraphicName()));
   }

   public static YoLineFX2D toYoLineFX2D(YoVariableDatabase yoVariableDatabase,
                                         YoGraphicFXResourceManager resourceManager,
                                         ReferenceFrameManager referenceFrameManager,
                                         YoGraphicLine2DDefinition definition)
   {
      YoLineFX2D yoGraphicFX = new YoLineFX2D();
      toYoLineFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoLineFX2D(YoVariableDatabase yoVariableDatabase,
                                   YoGraphicFXResourceManager resourceManager,
                                   ReferenceFrameManager referenceFrameManager,
                                   YoGraphicLine2DDefinition definition,
                                   YoLineFX2D yoGraphicFXToPack)
   {
      toYoGraphicFX2D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setOrigin(CompositePropertyTools.toTuple2DProperty(yoVariableDatabase, referenceFrameManager, definition.getOrigin()));
      yoGraphicFXToPack.setDirection(CompositePropertyTools.toTuple2DProperty(yoVariableDatabase, referenceFrameManager, definition.getDirection()));
      yoGraphicFXToPack.setDestination(CompositePropertyTools.toTuple2DProperty(yoVariableDatabase, referenceFrameManager, definition.getDestination()));
   }

   private static void toYoGraphicFX3D(YoVariableDatabase yoVariableDatabase,
                                       YoGraphicFXResourceManager resourceManager,
                                       ReferenceFrameManager referenceFrameManager,
                                       YoGraphic3DDefinition definition,
                                       YoGraphicFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setColor(JavaFXVisualTools.toColor(definition.getColor()));
   }

   public static YoPointFX3D toYoPointFX3D(YoVariableDatabase yoVariableDatabase,
                                           YoGraphicFXResourceManager resourceManager,
                                           ReferenceFrameManager referenceFrameManager,
                                           YoGraphicPoint3DDefinition definition)
   {
      YoPointFX3D yoGraphicFX = new YoPointFX3D();
      toYoPointFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoPointFX3D(YoVariableDatabase yoVariableDatabase,
                                    YoGraphicFXResourceManager resourceManager,
                                    ReferenceFrameManager referenceFrameManager,
                                    YoGraphicPoint3DDefinition definition,
                                    YoPointFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPosition(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getPosition()));
      yoGraphicFXToPack.setSize(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getSize()));
      if (definition.getGraphicName() != null)
         yoGraphicFXToPack.setGraphicResource(resourceManager.loadGraphic3DResource(definition.getGraphicName()));
   }

   public static YoArrowFX3D toYoArrowFX3D(YoVariableDatabase yoVariableDatabase,
                                           YoGraphicFXResourceManager resourceManager,
                                           ReferenceFrameManager referenceFrameManager,
                                           YoGraphicArrow3DDefinition definition)
   {
      YoArrowFX3D yoGraphicFX = new YoArrowFX3D();
      toYoArrowFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoArrowFX3D(YoVariableDatabase yoVariableDatabase,
                                    YoGraphicFXResourceManager resourceManager,
                                    ReferenceFrameManager referenceFrameManager,
                                    YoGraphicArrow3DDefinition definition,
                                    YoArrowFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setOrigin(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getOrigin()));
      yoGraphicFXToPack.setDirection(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getDirection()));
      yoGraphicFXToPack.setScaleLength(definition.isScaleLength());
      yoGraphicFXToPack.setBodyLength(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getBodyLength()));
      yoGraphicFXToPack.setHeadLength(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getHeadLength()));
      yoGraphicFXToPack.setScaleRadius(definition.isScaleRadius());
      yoGraphicFXToPack.setBodyRadius(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getBodyRadius()));
      yoGraphicFXToPack.setHeadRadius(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getHeadRadius()));
   }

   public static YoCapsuleFX3D toYoCapsuleFX3D(YoVariableDatabase yoVariableDatabase,
                                               YoGraphicFXResourceManager resourceManager,
                                               ReferenceFrameManager referenceFrameManager,
                                               YoGraphicCapsule3DDefinition definition)
   {
      YoCapsuleFX3D yoGraphicFX = new YoCapsuleFX3D();
      toYoCapsuleFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoCapsuleFX3D(YoVariableDatabase yoVariableDatabase,
                                      YoGraphicFXResourceManager resourceManager,
                                      ReferenceFrameManager referenceFrameManager,
                                      YoGraphicCapsule3DDefinition definition,
                                      YoCapsuleFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setCenter(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getCenter()));
      yoGraphicFXToPack.setAxis(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getAxis()));
      yoGraphicFXToPack.setLength(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getLength()));
      yoGraphicFXToPack.setRadius(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getRadius()));
   }

   public static YoCylinderFX3D toYoCylinderFX3D(YoVariableDatabase yoVariableDatabase,
                                                 YoGraphicFXResourceManager resourceManager,
                                                 ReferenceFrameManager referenceFrameManager,
                                                 YoGraphicCylinder3DDefinition definition)
   {
      YoCylinderFX3D yoGraphicFX = new YoCylinderFX3D();
      toYoCylinderFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoCylinderFX3D(YoVariableDatabase yoVariableDatabase,
                                       YoGraphicFXResourceManager resourceManager,
                                       ReferenceFrameManager referenceFrameManager,
                                       YoGraphicCylinder3DDefinition definition,
                                       YoCylinderFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setCenter(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getCenter()));
      yoGraphicFXToPack.setAxis(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getAxis()));
      yoGraphicFXToPack.setLength(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getLength()));
      yoGraphicFXToPack.setRadius(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getRadius()));
   }

   public static YoConeFX3D toYoConeFX3D(YoVariableDatabase yoVariableDatabase,
                                         YoGraphicFXResourceManager resourceManager,
                                         ReferenceFrameManager referenceFrameManager,
                                         YoGraphicCone3DDefinition definition)
   {
      YoConeFX3D yoGraphicFX = new YoConeFX3D();
      toYoConeFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoConeFX3D(YoVariableDatabase yoVariableDatabase,
                                   YoGraphicFXResourceManager resourceManager,
                                   ReferenceFrameManager referenceFrameManager,
                                   YoGraphicCone3DDefinition definition,
                                   YoConeFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPosition(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getPosition()));
      yoGraphicFXToPack.setAxis(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getAxis()));
      yoGraphicFXToPack.setHeight(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getHeight()));
      yoGraphicFXToPack.setRadius(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getRadius()));
   }

   public static YoEllipsoidFX3D toYoEllipsoidFX3D(YoVariableDatabase yoVariableDatabase,
                                                   YoGraphicFXResourceManager resourceManager,
                                                   ReferenceFrameManager referenceFrameManager,
                                                   YoGraphicEllipsoid3DDefinition definition)
   {
      YoEllipsoidFX3D yoGraphicFX = new YoEllipsoidFX3D();
      toYoEllipsoidFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoEllipsoidFX3D(YoVariableDatabase yoVariableDatabase,
                                        YoGraphicFXResourceManager resourceManager,
                                        ReferenceFrameManager referenceFrameManager,
                                        YoGraphicEllipsoid3DDefinition definition,
                                        YoEllipsoidFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPosition(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getPosition()));
      yoGraphicFXToPack.setOrientation(CompositePropertyTools.toOrientation3DProperty(yoVariableDatabase, referenceFrameManager, definition.getOrientation()));
      yoGraphicFXToPack.setRadii(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getRadii()));
   }

   public static YoPointcloudFX3D toYoPointcloudFX3D(YoVariableDatabase yoVariableDatabase,
                                                     YoGraphicFXResourceManager resourceManager,
                                                     ReferenceFrameManager referenceFrameManager,
                                                     YoGraphicPointcloud3DDefinition definition)
   {
      YoPointcloudFX3D yoGraphicFX = new YoPointcloudFX3D();
      toYoPointcloudFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoPointcloudFX3D(YoVariableDatabase yoVariableDatabase,
                                         YoGraphicFXResourceManager resourceManager,
                                         ReferenceFrameManager referenceFrameManager,
                                         YoGraphicPointcloud3DDefinition definition,
                                         YoPointcloudFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPoints(CompositePropertyTools.toTuple3DPropertyList(yoVariableDatabase, referenceFrameManager, definition.getPoints()));
      yoGraphicFXToPack.setNumberOfPoints(CompositePropertyTools.toIntegerProperty(yoVariableDatabase, definition.getNumberOfPoints()));
      yoGraphicFXToPack.setSize(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getSize()));
      yoGraphicFXToPack.setGraphicResource(resourceManager.loadGraphic3DResource(definition.getGraphicName()));
   }

   public static YoPolynomialFX3D toYoPolynomialFX3D(YoVariableDatabase yoVariableDatabase,
                                                     YoGraphicFXResourceManager resourceManager,
                                                     ReferenceFrameManager referenceFrameManager,
                                                     YoGraphicPolynomial3DDefinition definition)
   {
      YoPolynomialFX3D yoGraphicFX = new YoPolynomialFX3D();
      toYoPolynomialFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoPolynomialFX3D(YoVariableDatabase yoVariableDatabase,
                                         YoGraphicFXResourceManager resourceManager,
                                         ReferenceFrameManager referenceFrameManager,
                                         YoGraphicPolynomial3DDefinition definition,
                                         YoPolynomialFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      YoListDefinition coefficientsX = definition.getCoefficientsX();
      YoListDefinition coefficientsY = definition.getCoefficientsY();
      YoListDefinition coefficientsZ = definition.getCoefficientsZ();
      yoGraphicFXToPack.setCoefficientsX(CompositePropertyTools.toDoublePropertyList(yoVariableDatabase,
                                                                                     coefficientsX == null ? null : coefficientsX.getElements()));
      yoGraphicFXToPack.setCoefficientsY(CompositePropertyTools.toDoublePropertyList(yoVariableDatabase,
                                                                                     coefficientsY == null ? null : coefficientsY.getElements()));
      yoGraphicFXToPack.setCoefficientsZ(CompositePropertyTools.toDoublePropertyList(yoVariableDatabase,
                                                                                     coefficientsZ == null ? null : coefficientsZ.getElements()));
      yoGraphicFXToPack.setNumberOfCoefficientsX(CompositePropertyTools.toIntegerProperty(yoVariableDatabase,
                                                                                          coefficientsX == null ? null : coefficientsX.getSize()));
      yoGraphicFXToPack.setNumberOfCoefficientsY(CompositePropertyTools.toIntegerProperty(yoVariableDatabase,
                                                                                          coefficientsY == null ? null : coefficientsY.getSize()));
      yoGraphicFXToPack.setNumberOfCoefficientsZ(CompositePropertyTools.toIntegerProperty(yoVariableDatabase,
                                                                                          coefficientsZ == null ? null : coefficientsZ.getSize()));
      yoGraphicFXToPack.setReferenceFrame(CompositePropertyTools.toReferenceFrameProperty(yoVariableDatabase,
                                                                                          referenceFrameManager,
                                                                                          definition.getReferenceFrame()));
      yoGraphicFXToPack.setStartTime(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getStartTime()));
      yoGraphicFXToPack.setEndTime(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getEndTime()));
      yoGraphicFXToPack.setSize(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getSize()));
      yoGraphicFXToPack.setTimeResolution(CompositePropertyTools.toIntegerProperty(yoVariableDatabase, definition.getTimeResolution()));
      yoGraphicFXToPack.setNumberOfDivisions(CompositePropertyTools.toIntegerProperty(yoVariableDatabase, definition.getNumberOfDivisions()));
   }

   public static YoCoordinateSystemFX3D toYoCoordinateSystemFX3D(YoVariableDatabase yoVariableDatabase,
                                                                 YoGraphicFXResourceManager resourceManager,
                                                                 ReferenceFrameManager referenceFrameManager,
                                                                 YoGraphicCoordinateSystem3DDefinition definition)
   {
      YoCoordinateSystemFX3D yoGraphicFX = new YoCoordinateSystemFX3D();
      toYoCoordinateSystemFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoCoordinateSystemFX3D(YoVariableDatabase yoVariableDatabase,
                                               YoGraphicFXResourceManager resourceManager,
                                               ReferenceFrameManager referenceFrameManager,
                                               YoGraphicCoordinateSystem3DDefinition definition,
                                               YoCoordinateSystemFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPosition(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getPosition()));
      yoGraphicFXToPack.setOrientation(CompositePropertyTools.toOrientation3DProperty(yoVariableDatabase, referenceFrameManager, definition.getOrientation()));
      yoGraphicFXToPack.setBodyLength(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getBodyLength()));
      yoGraphicFXToPack.setHeadLength(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getHeadLength()));
      yoGraphicFXToPack.setBodyRadius(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getBodyRadius()));
      yoGraphicFXToPack.setHeadRadius(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getHeadRadius()));
   }

   public static YoPolygonExtrudedFX3D toYoPolygonExtrudedFX3D(YoVariableDatabase yoVariableDatabase,
                                                               YoGraphicFXResourceManager resourceManager,
                                                               ReferenceFrameManager referenceFrameManager,
                                                               YoGraphicPolygonExtruded3DDefinition definition)
   {
      YoPolygonExtrudedFX3D yoGraphicFX = new YoPolygonExtrudedFX3D();
      toYoPolygonExtrudedFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoPolygonExtrudedFX3D(YoVariableDatabase yoVariableDatabase,
                                              YoGraphicFXResourceManager resourceManager,
                                              ReferenceFrameManager referenceFrameManager,
                                              YoGraphicPolygonExtruded3DDefinition definition,
                                              YoPolygonExtrudedFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPosition(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getPosition()));
      yoGraphicFXToPack.setOrientation(CompositePropertyTools.toOrientation3DProperty(yoVariableDatabase, referenceFrameManager, definition.getOrientation()));
      yoGraphicFXToPack.setVertices(CompositePropertyTools.toTuple2DPropertyList(yoVariableDatabase, referenceFrameManager, definition.getVertices()));
      yoGraphicFXToPack.setNumberOfVertices(CompositePropertyTools.toIntegerProperty(yoVariableDatabase, definition.getNumberOfVertices()));
      yoGraphicFXToPack.setThickness(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getThickness()));
   }

   public static YoConvexPolytopeFX3D toYoConvexPolytopeFX3D(YoVariableDatabase yoVariableDatabase,
                                                             YoGraphicFXResourceManager resourceManager,
                                                             ReferenceFrameManager referenceFrameManager,
                                                             YoGraphicConvexPolytope3DDefinition definition)
   {
      YoConvexPolytopeFX3D yoGraphicFX = new YoConvexPolytopeFX3D();
      toYoConvexPolytopeFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoConvexPolytopeFX3D(YoVariableDatabase yoVariableDatabase,
                                             YoGraphicFXResourceManager resourceManager,
                                             ReferenceFrameManager referenceFrameManager,
                                             YoGraphicConvexPolytope3DDefinition definition,
                                             YoConvexPolytopeFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPosition(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getPosition()));
      yoGraphicFXToPack.setOrientation(CompositePropertyTools.toOrientation3DProperty(yoVariableDatabase, referenceFrameManager, definition.getOrientation()));
      yoGraphicFXToPack.setVertices(CompositePropertyTools.toTuple3DPropertyList(yoVariableDatabase, referenceFrameManager, definition.getVertices()));
      yoGraphicFXToPack.setNumberOfVertices(CompositePropertyTools.toIntegerProperty(yoVariableDatabase, definition.getNumberOfVertices()));
   }

   public static YoBoxFX3D toYoBoxFX3D(YoVariableDatabase yoVariableDatabase,
                                       YoGraphicFXResourceManager resourceManager,
                                       ReferenceFrameManager referenceFrameManager,
                                       YoGraphicBox3DDefinition definition)
   {
      YoBoxFX3D yoGraphicFX = new YoBoxFX3D();
      toYoBoxFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoBoxFX3D(YoVariableDatabase yoVariableDatabase,
                                  YoGraphicFXResourceManager resourceManager,
                                  ReferenceFrameManager referenceFrameManager,
                                  YoGraphicBox3DDefinition definition,
                                  YoBoxFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPosition(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getPosition()));
      yoGraphicFXToPack.setOrientation(CompositePropertyTools.toOrientation3DProperty(yoVariableDatabase, referenceFrameManager, definition.getOrientation()));
      yoGraphicFXToPack.setSize(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getSize()));
   }

   public static YoSTPBoxFX3D toYoSTPBoxFX3D(YoVariableDatabase yoVariableDatabase,
                                             YoGraphicFXResourceManager resourceManager,
                                             ReferenceFrameManager referenceFrameManager,
                                             YoGraphicSTPBox3DDefinition definition)
   {
      YoSTPBoxFX3D yoGraphicFX = new YoSTPBoxFX3D();
      toYoSTPBoxFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoSTPBoxFX3D(YoVariableDatabase yoVariableDatabase,
                                     YoGraphicFXResourceManager resourceManager,
                                     ReferenceFrameManager referenceFrameManager,
                                     YoGraphicSTPBox3DDefinition definition,
                                     YoSTPBoxFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPosition(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getPosition()));
      yoGraphicFXToPack.setOrientation(CompositePropertyTools.toOrientation3DProperty(yoVariableDatabase, referenceFrameManager, definition.getOrientation()));
      yoGraphicFXToPack.setSize(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getSize()));
      yoGraphicFXToPack.setMinimumMargin(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getMinimumMargin()));
      yoGraphicFXToPack.setMaximumMargin(CompositePropertyTools.toDoubleProperty(yoVariableDatabase, definition.getMaximumMargin()));
   }

   public static YoRampFX3D toYoRampFX3D(YoVariableDatabase yoVariableDatabase,
                                         YoGraphicFXResourceManager resourceManager,
                                         ReferenceFrameManager referenceFrameManager,
                                         YoGraphicRamp3DDefinition definition)
   {
      YoRampFX3D yoGraphicFX = new YoRampFX3D();
      toYoRampFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFX);
      return yoGraphicFX;
   }

   public static void toYoRampFX3D(YoVariableDatabase yoVariableDatabase,
                                   YoGraphicFXResourceManager resourceManager,
                                   ReferenceFrameManager referenceFrameManager,
                                   YoGraphicRamp3DDefinition definition,
                                   YoRampFX3D yoGraphicFXToPack)
   {
      toYoGraphicFX3D(yoVariableDatabase, resourceManager, referenceFrameManager, definition, yoGraphicFXToPack);
      yoGraphicFXToPack.setPosition(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getPosition()));
      yoGraphicFXToPack.setOrientation(CompositePropertyTools.toOrientation3DProperty(yoVariableDatabase, referenceFrameManager, definition.getOrientation()));
      yoGraphicFXToPack.setSize(CompositePropertyTools.toTuple3DProperty(yoVariableDatabase, referenceFrameManager, definition.getSize()));
   }

   public static YoGroupFX convertRobotCollisionShapeDefinitions(RigidBodyReadOnly rootBody, RobotDefinition robotDefinition)
   {
      Color color = Color.AQUAMARINE.deriveColor(0.0, 1.0, 1.0, 0.4); // Transparent aquamarine
      return convertRobotCollisionShapeDefinitions(rootBody, robotDefinition, color);
   }

   public static YoGroupFX convertRobotCollisionShapeDefinitions(RigidBodyReadOnly rootBody, RobotDefinition robotDefinition, Color color)
   {
      YoGroupFX robotCollisionGroup = new YoGroupFX(robotDefinition.getName());

      List<RigidBodyDefinition> allRigidBodies = robotDefinition.getAllRigidBodies();

      for (RigidBodyDefinition rigidBodyDefinition : allRigidBodies)
      {
         if (rigidBodyDefinition.getCollisionShapeDefinitions() == null)
            continue;
         if (rigidBodyDefinition.getCollisionShapeDefinitions().isEmpty())
            continue;

         RigidBodyReadOnly rigidBody = MultiBodySystemTools.findRigidBody(rootBody, rigidBodyDefinition.getName());
         ReferenceFrame referenceFrame = rigidBody.isRootBody() ? rigidBody.getBodyFixedFrame() : rigidBody.getParentJoint().getFrameAfterJoint();
         YoGroupFX collisionGroup = convertRigidBodyCollisionShapeDefinitions(referenceFrame, rigidBodyDefinition, color);

         if (collisionGroup != null)
            robotCollisionGroup.addChild(collisionGroup);
      }

      return robotCollisionGroup;
   }

   public static YoGroupFX convertRigidBodyCollisionShapeDefinitions(ReferenceFrame referenceFrame, RigidBodyDefinition rigidBodyDefinition, Color color)
   {
      List<CollisionShapeDefinition> collisionShapeDefinitions = rigidBodyDefinition.getCollisionShapeDefinitions();

      if (collisionShapeDefinitions == null || collisionShapeDefinitions.isEmpty())
         return null;

      YoGroupFX yoGroupFX = new YoGroupFX(rigidBodyDefinition.getName());

      for (CollisionShapeDefinition collisionShapeDefinition : collisionShapeDefinitions)
      {
         YoGraphicFX3D convertedGraphic;
         try
         {
            convertedGraphic = convertCollisionShapeDefinition(referenceFrame, collisionShapeDefinition);
         }
         catch (Exception e)
         {
            e.printStackTrace();
            continue;
         }

         String graphicName = collisionShapeDefinition.getName();
         if (graphicName == null || graphicName.trim().isEmpty())
            graphicName = collisionShapeDefinition.getGeometryDefinition().getClass().getSimpleName();
         graphicName = YoGraphicFXControllerTools.createAvailableYoGraphicFX3DName(yoGroupFX, graphicName);
         convertedGraphic.setName(graphicName);
         convertedGraphic.setColor(color);
         yoGroupFX.addYoGraphicFX3D(convertedGraphic);
      }

      if (yoGroupFX.getYoGraphicFX3DSet().isEmpty())
         return null;

      return yoGroupFX;
   }

   public static YoGroupFX convertTerrainObjectsCollisionShapeDefinitions(ReferenceFrame worldFrame, List<TerrainObjectDefinition> terrainObjectDefinitions)
   {
      Color color = Color.BLUEVIOLET.deriveColor(0.0, 1.0, 1.0, 0.4); // Transparent blueviolet
      return convertTerrainObjectsCollisionShapeDefinitions(worldFrame, terrainObjectDefinitions, color);
   }

   public static YoGroupFX convertTerrainObjectsCollisionShapeDefinitions(ReferenceFrame worldFrame,
                                                                          List<TerrainObjectDefinition> terrainObjectDefinitions,
                                                                          Color color)
   {
      YoGroupFX terrainsCollisionGroup = new YoGroupFX("Terrain objects - collisions");

      for (TerrainObjectDefinition terrainObjectDefinition : terrainObjectDefinitions)
      {
         YoGroupFX terrainCollisionGroup = convertTerrainObjectCollisionShapeDefinitions(worldFrame, terrainObjectDefinition, color);

         if (terrainCollisionGroup == null || terrainCollisionGroup.getItemChildren().isEmpty())
            continue;

         String adjustedName = YoGraphicFXControllerTools.createAvailableYoGraphicFXGroupName(terrainsCollisionGroup, terrainCollisionGroup.getName());
         terrainCollisionGroup.setName(adjustedName);
         terrainsCollisionGroup.addChild(terrainCollisionGroup);
      }

      return terrainsCollisionGroup;
   }

   public static YoGroupFX convertTerrainObjectCollisionShapeDefinitions(ReferenceFrame worldFrame,
                                                                         TerrainObjectDefinition terrainObjectDefinition,
                                                                         Color color)
   {
      List<CollisionShapeDefinition> collisionShapeDefinitions = terrainObjectDefinition.getCollisionShapeDefinitions();

      if (collisionShapeDefinitions == null || collisionShapeDefinitions.isEmpty())
         return null;

      String name = terrainObjectDefinition.getName();
      if (name == null || name.isEmpty())
         name = "TerrainObject";

      YoGroupFX yoGroupFX = new YoGroupFX(name);

      for (CollisionShapeDefinition collisionShapeDefinition : collisionShapeDefinitions)
      {
         YoGraphicFX3D convertedGraphic;
         try
         {
            convertedGraphic = convertCollisionShapeDefinition(worldFrame, collisionShapeDefinition);
         }
         catch (Exception e)
         {
            e.printStackTrace();
            continue;
         }

         String graphicName = collisionShapeDefinition.getName();
         if (graphicName == null || graphicName.trim().isEmpty())
            graphicName = collisionShapeDefinition.getGeometryDefinition().getClass().getSimpleName();
         graphicName = YoGraphicFXControllerTools.createAvailableYoGraphicFX3DName(yoGroupFX, graphicName);
         convertedGraphic.setName(graphicName);
         convertedGraphic.setColor(color);
         yoGroupFX.addYoGraphicFX3D(convertedGraphic);
      }

      if (yoGroupFX.getYoGraphicFX3DSet().isEmpty())
         return null;

      return yoGroupFX;
   }

   public static YoGraphicFX3D convertCollisionShapeDefinition(ReferenceFrame referenceFrame, CollisionShapeDefinition collisionShapeDefinition)
   {
      return convertGeometryDefinition(referenceFrame, collisionShapeDefinition.getOriginPose(), collisionShapeDefinition.getGeometryDefinition());
   }

   public static YoGraphicFX3D convertGeometryDefinition(ReferenceFrame referenceFrame,
                                                         RigidBodyTransformReadOnly originPose,
                                                         GeometryDefinition geometryDefinition)
   {
      if (geometryDefinition instanceof Box3DDefinition)
         return convertBox3DDefinition(referenceFrame, originPose, (Box3DDefinition) geometryDefinition);
      else if (geometryDefinition instanceof STPBox3DDefinition)
         return convertSTPBox3DDefinition(referenceFrame, originPose, (STPBox3DDefinition) geometryDefinition);
      else if (geometryDefinition instanceof Capsule3DDefinition)
         return convertCapsule3DDefinition(referenceFrame, originPose, (Capsule3DDefinition) geometryDefinition);
      else if (geometryDefinition instanceof Cone3DDefinition)
         return convertCone3DDefinition(referenceFrame, originPose, (Cone3DDefinition) geometryDefinition);
      else if (geometryDefinition instanceof Cylinder3DDefinition)
         return convertCylinder3DDefinition(referenceFrame, originPose, (Cylinder3DDefinition) geometryDefinition);
      else if (geometryDefinition instanceof ExtrudedPolygon2DDefinition)
         return convertExtrudedPolygon2DDefinition(referenceFrame, originPose, (ExtrudedPolygon2DDefinition) geometryDefinition);
      else if (geometryDefinition instanceof ConvexPolytope3DDefinition)
         return convertConvexPolytope3DDefinition(referenceFrame, originPose, (ConvexPolytope3DDefinition) geometryDefinition);
      else if (geometryDefinition instanceof Point3DDefinition)
         return convertPoint3DDefinition(referenceFrame, originPose, (Point3DDefinition) geometryDefinition);
      else if (geometryDefinition instanceof Ramp3DDefinition)
         return convertRamp3DDefinition(referenceFrame, originPose, (Ramp3DDefinition) geometryDefinition);
      else if (geometryDefinition instanceof Sphere3DDefinition)
         return convertSphere3DDefinition(referenceFrame, originPose, (Sphere3DDefinition) geometryDefinition);
      else
         throw new UnsupportedOperationException("Unsupported " + GeometryDefinition.class.getSimpleName() + ": "
               + geometryDefinition.getClass().getSimpleName());
   }

   public static YoGroupFX convertRobotMassPropertiesShapeDefinitions(RigidBodyReadOnly rootBody, RobotDefinition robotDefinition)
   {
      Color color = Color.DARKSEAGREEN.deriveColor(0.0, 1.0, 1.0, 0.4); // Transparent
      return convertRobotMassPropertiesShapeDefinitions(rootBody, robotDefinition, color);
   }

   public static YoGroupFX convertRobotMassPropertiesShapeDefinitions(RigidBodyReadOnly rootBody, RobotDefinition robotDefinition, Color color)
   {
      YoGroupFX robotMassPropertiesGroup = new YoGroupFX(robotDefinition.getName());

      List<RigidBodyDefinition> allRigidBodies = robotDefinition.getAllRigidBodies();

      for (RigidBodyDefinition rigidBodyDefinition : allRigidBodies)
      {
         if (rigidBodyDefinition.getInertiaPose() == null)
            continue;
         if (rigidBodyDefinition.getMomentOfInertia() == null)
            continue;

         SingularValueDecomposition3D svd = new SingularValueDecomposition3D();
         if (!svd.decompose(rigidBodyDefinition.getMomentOfInertia()))
            continue;

         RigidBodyReadOnly rigidBody = MultiBodySystemTools.findRigidBody(rootBody, rigidBodyDefinition.getName());

         if (rigidBody.isRootBody())
            continue;

         ReferenceFrame referenceFrame = rigidBody.getParentJoint().getFrameAfterJoint();

         RigidBodyTransform ellipsoidPose = new RigidBodyTransform(rigidBodyDefinition.getInertiaPose());
         ellipsoidPose.appendOrientation(svd.getU());
         Vector3D radii = computeInertiaEllipsoidRadii(svd.getW(), rigidBodyDefinition.getMass());
         YoEllipsoidFX3D ellipsoid = convertEllipsoid3DDefinition(referenceFrame, ellipsoidPose, new Ellipsoid3DDefinition(radii));
         ellipsoid.setName(rigidBody.getName() + " inertia");
         ellipsoid.setColor(color);
         robotMassPropertiesGroup.addYoGraphicFX3D(ellipsoid);
      }

      return robotMassPropertiesGroup;
   }

   /**
    * Returns the radii of an ellipsoid given the inertia parameters, assuming a uniform mass
    * distribution.
    * 
    * @param principalMomentsOfInertia principal moments of inertia {Ixx, Iyy, Izz}
    * @param mass                      mass of the link
    * @return the three radii of the inertia ellipsoid
    */
   public static Vector3D computeInertiaEllipsoidRadii(Vector3DReadOnly principalMomentsOfInertia, double mass)
   {
      double Ixx = principalMomentsOfInertia.getX();
      double Iyy = principalMomentsOfInertia.getY();
      double Izz = principalMomentsOfInertia.getZ();

      //    http://en.wikipedia.org/wiki/Ellipsoid#Mass_properties
      Vector3D radii = new Vector3D();
      radii.setX(Math.sqrt(5.0 / 2.0 * (Iyy + Izz - Ixx) / mass));
      radii.setY(Math.sqrt(5.0 / 2.0 * (Izz + Ixx - Iyy) / mass));
      radii.setZ(Math.sqrt(5.0 / 2.0 * (Ixx + Iyy - Izz) / mass));

      return radii;
   }

   public static YoBoxFX3D convertBox3DDefinition(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly originPose, Box3DDefinition geometryDefinition)
   {
      YoBoxFX3D yoGraphicFX = new YoBoxFX3D();
      if (!geometryDefinition.isCentered())
      {
         RigidBodyTransform temp = new RigidBodyTransform();
         temp.set(originPose);
         temp.appendTranslation(0.0, 0.0, 0.5 * geometryDefinition.getSizeZ());
         originPose = temp;
      }
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setPosition(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      Quaternion orientation = new Quaternion(originPose.getRotation());
      yoGraphicFX.setOrientation(new QuaternionProperty(referenceFrame, orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS()));
      yoGraphicFX.setSize(new Tuple3DProperty(referenceFrame, geometryDefinition.getSizeX(), geometryDefinition.getSizeY(), geometryDefinition.getSizeZ()));
      return yoGraphicFX;
   }

   public static YoSTPBoxFX3D convertSTPBox3DDefinition(ReferenceFrame referenceFrame,
                                                        RigidBodyTransformReadOnly originPose,
                                                        STPBox3DDefinition geometryDefinition)
   {
      YoSTPBoxFX3D yoGraphicFX = new YoSTPBoxFX3D();
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setPosition(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      Quaternion orientation = new Quaternion(originPose.getRotation());
      yoGraphicFX.setOrientation(new QuaternionProperty(referenceFrame, orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS()));
      yoGraphicFX.setSize(new Tuple3DProperty(referenceFrame, geometryDefinition.getSizeX(), geometryDefinition.getSizeY(), geometryDefinition.getSizeZ()));
      yoGraphicFX.setMinimumMargin(geometryDefinition.getMinimumMargin());
      yoGraphicFX.setMaximumMargin(geometryDefinition.getMaximumMargin());
      return yoGraphicFX;
   }

   public static YoCapsuleFX3D convertCapsule3DDefinition(ReferenceFrame referenceFrame,
                                                          RigidBodyTransformReadOnly originPose,
                                                          Capsule3DDefinition geometryDefinition)
   {
      if (!geometryDefinition.isRegular())
         throw new UnsupportedOperationException("Irregular capsules are not supported.");

      YoCapsuleFX3D yoGraphicFX = new YoCapsuleFX3D();
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setCenter(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      Vector3D axis = new Vector3D();
      originPose.transform(Axis3D.Z, axis);
      yoGraphicFX.setAxis(new Tuple3DProperty(referenceFrame, axis.getX(), axis.getY(), axis.getZ()));
      yoGraphicFX.setRadius(geometryDefinition.getRadiusX());
      yoGraphicFX.setLength(geometryDefinition.getLength());
      return yoGraphicFX;
   }

   public static YoConeFX3D convertCone3DDefinition(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly originPose, Cone3DDefinition geometryDefinition)
   {
      YoConeFX3D yoGraphicFX = new YoConeFX3D();
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setPosition(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      Vector3D axis = new Vector3D();
      originPose.transform(Axis3D.Z, axis);
      yoGraphicFX.setAxis(new Tuple3DProperty(referenceFrame, axis.getX(), axis.getY(), axis.getZ()));
      yoGraphicFX.setRadius(geometryDefinition.getRadius());
      yoGraphicFX.setHeight(geometryDefinition.getHeight());
      return yoGraphicFX;
   }

   public static YoCylinderFX3D convertCylinder3DDefinition(ReferenceFrame referenceFrame,
                                                            RigidBodyTransformReadOnly originPose,
                                                            Cylinder3DDefinition geometryDefinition)
   {
      if (!geometryDefinition.isCentered())
         throw new UnsupportedOperationException("Un-centered cylinders are not supported.");

      YoCylinderFX3D yoGraphicFX = new YoCylinderFX3D();
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setCenter(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      Vector3D axis = new Vector3D();
      originPose.transform(Axis3D.Z, axis);
      yoGraphicFX.setAxis(new Tuple3DProperty(referenceFrame, axis.getX(), axis.getY(), axis.getZ()));
      yoGraphicFX.setRadius(geometryDefinition.getRadius());
      yoGraphicFX.setLength(geometryDefinition.getLength());
      return yoGraphicFX;
   }

   public static YoEllipsoidFX3D convertEllipsoid3DDefinition(ReferenceFrame referenceFrame,
                                                              RigidBodyTransformReadOnly originPose,
                                                              Ellipsoid3DDefinition geometryDefinition)
   {
      YoEllipsoidFX3D yoGraphicFX = new YoEllipsoidFX3D();
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setPosition(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      Quaternion orientation = new Quaternion(originPose.getRotation());
      yoGraphicFX.setOrientation(new QuaternionProperty(referenceFrame, orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS()));
      yoGraphicFX.setRadii(new Tuple3DProperty(referenceFrame,
                                               geometryDefinition.getRadiusX(),
                                               geometryDefinition.getRadiusY(),
                                               geometryDefinition.getRadiusZ()));
      return yoGraphicFX;
   }

   public static YoPolygonExtrudedFX3D convertExtrudedPolygon2DDefinition(ReferenceFrame referenceFrame,
                                                                          RigidBodyTransformReadOnly originPose,
                                                                          ExtrudedPolygon2DDefinition geometryDefinition)
   {
      if (geometryDefinition.getBottomZ() != 0.0)
         throw new UnsupportedOperationException("Only supports extruded polygon with bottom z coordinate at zero.");

      YoPolygonExtrudedFX3D yoGraphicFX = new YoPolygonExtrudedFX3D();
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setPosition(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      Quaternion orientation = new Quaternion(originPose.getRotation());
      yoGraphicFX.setOrientation(new QuaternionProperty(referenceFrame, orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS()));
      yoGraphicFX.setThickness(geometryDefinition.getTopZ() - geometryDefinition.getBottomZ());
      yoGraphicFX.setVertices(geometryDefinition.getPolygonVertices()
                                                .stream()
                                                .map(v -> new Tuple2DProperty(referenceFrame, v.getX(), v.getY()))
                                                .collect(Collectors.toList()));
      yoGraphicFX.setNumberOfVertices(geometryDefinition.getPolygonVertices().size());
      return yoGraphicFX;
   }

   public static YoConvexPolytopeFX3D convertConvexPolytope3DDefinition(ReferenceFrame referenceFrame,
                                                                        RigidBodyTransformReadOnly originPose,
                                                                        ConvexPolytope3DDefinition geometryDefinition)
   {
      YoConvexPolytopeFX3D yoGraphicFX = new YoConvexPolytopeFX3D();
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setPosition(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      Quaternion orientation = new Quaternion(originPose.getRotation());
      yoGraphicFX.setOrientation(new QuaternionProperty(referenceFrame, orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS()));
      yoGraphicFX.setVertices(geometryDefinition.getConvexPolytope()
                                                .getVertices()
                                                .stream()
                                                .map(v -> new Tuple3DProperty(referenceFrame, v.getX(), v.getY(), v.getZ()))
                                                .collect(Collectors.toList()));
      yoGraphicFX.setNumberOfVertices(geometryDefinition.getConvexPolytope().getNumberOfVertices());
      return yoGraphicFX;
   }

   public static YoPointFX3D convertPoint3DDefinition(ReferenceFrame referenceFrame,
                                                      RigidBodyTransformReadOnly originPose,
                                                      Point3DDefinition geometryDefinition)
   {
      YoPointFX3D yoGraphicFX = new YoPointFX3D();
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setPosition(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      yoGraphicFX.setSize(0.01);
      return yoGraphicFX;
   }

   public static YoPointFX3D convertSphere3DDefinition(ReferenceFrame referenceFrame,
                                                       RigidBodyTransformReadOnly originPose,
                                                       Sphere3DDefinition geometryDefinition)
   {
      YoPointFX3D yoGraphicFX = new YoPointFX3D();
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setPosition(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      yoGraphicFX.setSize(geometryDefinition.getRadius());
      return yoGraphicFX;
   }

   public static YoRampFX3D convertRamp3DDefinition(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly originPose, Ramp3DDefinition geometryDefinition)
   {
      YoRampFX3D yoGraphicFX = new YoRampFX3D();
      Tuple3DReadOnly position = originPose.getTranslation();
      yoGraphicFX.setPosition(new Tuple3DProperty(referenceFrame, position.getX(), position.getY(), position.getZ()));
      Quaternion orientation = new Quaternion(originPose.getRotation());
      yoGraphicFX.setOrientation(new QuaternionProperty(referenceFrame, orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS()));
      yoGraphicFX.setSize(new Tuple3DProperty(referenceFrame, geometryDefinition.getSizeX(), geometryDefinition.getSizeY(), geometryDefinition.getSizeZ()));
      return yoGraphicFX;
   }

   public static YoGraphicListDefinition toYoGraphicListDefinition(Collection<? extends YoGraphicFXItem> yoGraphicFXs)
   {
      return new YoGraphicListDefinition(yoGraphicFXs.stream().map(YoGraphicTools::toYoGraphicDefinition).filter(Objects::nonNull).toList());
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicFXItem yoGraphicFX)
   {
      if (yoGraphicFX instanceof YoGroupFX)
         return toYoGraphicGroupDefinition((YoGroupFX) yoGraphicFX);
      else if (yoGraphicFX instanceof YoPointFX2D)
         return toYoGraphicPoint2DDefinition((YoPointFX2D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoPolygonFX2D)
         return toYoGraphicPolygon2DDefinition((YoPolygonFX2D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoPointcloudFX2D)
         return toYoGraphicPointcloud2DDefinition((YoPointcloudFX2D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoLineFX2D)
         return toYoGraphicLine2DDefinition((YoLineFX2D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoArrowFX3D)
         return toYoGraphicArrow3DDefinition((YoArrowFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoCapsuleFX3D)
         return toYoGraphicCapsule3DDefinition((YoCapsuleFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoConeFX3D)
         return toYoGraphicCone3DDefinition((YoConeFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoCylinderFX3D)
         return toYoGraphicCylinder3DDefinition((YoCylinderFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoPointFX3D)
         return toYoGraphicPoint3DDefinition((YoPointFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoPointcloudFX3D)
         return toYoGraphicPointcloud3DDefinition((YoPointcloudFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoPolynomialFX3D)
         return toYoGraphicPolynomial3DDefinition((YoPolynomialFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoCoordinateSystemFX3D)
         return toYoGraphicCoordinateSystem3DDefinition((YoCoordinateSystemFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoPolygonExtrudedFX3D)
         return toYoGraphicPolygonExtruded3DDefinition((YoPolygonExtrudedFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoConvexPolytopeFX3D)
         return toYoGraphicConvexPolytope3DDefinition((YoConvexPolytopeFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoBoxFX3D)
         return toYoGraphicBox3DDefinition((YoBoxFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoSTPBoxFX3D)
         return toYoGraphicSTPBox3DDefinition((YoSTPBoxFX3D) yoGraphicFX);
      else if (yoGraphicFX instanceof YoEllipsoidFX3D)
         return toYoGraphicEllipsoid3DDefinition((YoEllipsoidFX3D) yoGraphicFX);

      LogTools.error("Unsupported {}: {}", YoGraphicFX.class.getSimpleName(), yoGraphicFX.getClass().getSimpleName());
      return null;
   }

   public static YoGraphicGroupDefinition toYoGraphicGroupDefinition(YoGroupFX yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicGroupDefinition definition = new YoGraphicGroupDefinition();
      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setChildren(yoGraphicFX.getItemChildren().stream().map(YoGraphicTools::toYoGraphicDefinition).collect(Collectors.toList()));
      return definition;
   }

   public static YoGraphicPoint2DDefinition toYoGraphicPoint2DDefinition(YoPointFX2D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicPoint2DDefinition definition = new YoGraphicPoint2DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPosition(CompositePropertyTools.toYoTuple2DDefinition(yoGraphicFX.getPosition()));
      definition.setSize(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getSize()));
      definition.setFillColor(toColorDefinition(yoGraphicFX.getFillColor()));
      definition.setStrokeColor(toColorDefinition(yoGraphicFX.getStrokeColor()));
      definition.setStrokeWidth(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getStrokeWidth()));
      definition.setGraphicName(yoGraphicFX.getGraphicResource().getResourceName());

      return definition;
   }

   public static YoGraphicPolygon2DDefinition toYoGraphicPolygon2DDefinition(YoPolygonFX2D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicPolygon2DDefinition definition = new YoGraphicPolygon2DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setVertices(yoGraphicFX.getVertices().stream().map(CompositePropertyTools::toYoTuple2DDefinition).collect(Collectors.toList()));
      definition.setNumberOfVertices(CompositePropertyTools.toIntegerPropertyName(yoGraphicFX.getNumberOfVertices()));
      definition.setFillColor(toColorDefinition(yoGraphicFX.getFillColor()));
      definition.setStrokeColor(toColorDefinition(yoGraphicFX.getStrokeColor()));
      definition.setStrokeWidth(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getStrokeWidth()));

      return definition;
   }

   public static YoGraphicPointcloud2DDefinition toYoGraphicPointcloud2DDefinition(YoPointcloudFX2D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicPointcloud2DDefinition definition = new YoGraphicPointcloud2DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPoints(yoGraphicFX.getPoints().stream().map(CompositePropertyTools::toYoTuple2DDefinition).collect(Collectors.toList()));
      definition.setNumberOfPoints(CompositePropertyTools.toIntegerPropertyName(yoGraphicFX.getNumberOfPoints()));
      definition.setSize(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getSize()));
      definition.setGraphicName(yoGraphicFX.getGraphicResource().getResourceName());
      definition.setFillColor(toColorDefinition(yoGraphicFX.getFillColor()));
      definition.setStrokeColor(toColorDefinition(yoGraphicFX.getStrokeColor()));
      definition.setStrokeWidth(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getStrokeWidth()));

      return definition;
   }

   public static YoGraphicLine2DDefinition toYoGraphicLine2DDefinition(YoLineFX2D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicLine2DDefinition definition = new YoGraphicLine2DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setOrigin(CompositePropertyTools.toYoTuple2DDefinition(yoGraphicFX.getOrigin()));
      definition.setDirection(CompositePropertyTools.toYoTuple2DDefinition(yoGraphicFX.getDirection()));
      definition.setDestination(CompositePropertyTools.toYoTuple2DDefinition(yoGraphicFX.getDestination()));
      definition.setFillColor(toColorDefinition(yoGraphicFX.getFillColor()));
      definition.setStrokeColor(toColorDefinition(yoGraphicFX.getStrokeColor()));
      definition.setStrokeWidth(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getStrokeWidth()));

      return definition;
   }

   public static YoGraphicArrow3DDefinition toYoGraphicArrow3DDefinition(YoArrowFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicArrow3DDefinition definition = new YoGraphicArrow3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setOrigin(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getOrigin()));
      definition.setDirection(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getDirection()));
      definition.setScaleLength(yoGraphicFX.getScaleLength());
      definition.setScaleRadius(yoGraphicFX.getScaleRadius());
      definition.setBodyLength(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getBodyLength()));
      definition.setHeadLength(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getHeadLength()));
      definition.setBodyRadius(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getBodyRadius()));
      definition.setHeadRadius(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getHeadRadius()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicCapsule3DDefinition toYoGraphicCapsule3DDefinition(YoCapsuleFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicCapsule3DDefinition definition = new YoGraphicCapsule3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setCenter(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getCenter()));
      definition.setAxis(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getAxis()));
      definition.setLength(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getLength()));
      definition.setRadius(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getRadius()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicCone3DDefinition toYoGraphicCone3DDefinition(YoConeFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicCone3DDefinition definition = new YoGraphicCone3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPosition(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getPosition()));
      definition.setAxis(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getAxis()));
      definition.setHeight(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getHeight()));
      definition.setRadius(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getRadius()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicCylinder3DDefinition toYoGraphicCylinder3DDefinition(YoCylinderFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicCylinder3DDefinition definition = new YoGraphicCylinder3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setCenter(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getCenter()));
      definition.setAxis(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getAxis()));
      definition.setLength(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getLength()));
      definition.setRadius(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getRadius()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicPoint3DDefinition toYoGraphicPoint3DDefinition(YoPointFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicPoint3DDefinition definition = new YoGraphicPoint3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPosition(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getPosition()));
      definition.setSize(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getSize()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));
      definition.setGraphicName(yoGraphicFX.getGraphicResource().getResourceName());

      return definition;
   }

   public static YoGraphicPointcloud3DDefinition toYoGraphicPointcloud3DDefinition(YoPointcloudFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicPointcloud3DDefinition definition = new YoGraphicPointcloud3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPoints(yoGraphicFX.getPoints().stream().map(CompositePropertyTools::toYoTuple3DDefinition).collect(Collectors.toList()));
      definition.setNumberOfPoints(CompositePropertyTools.toIntegerPropertyName(yoGraphicFX.getNumberOfPoints()));
      definition.setSize(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getSize()));
      definition.setGraphicName(yoGraphicFX.getGraphicResource().getResourceName());
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicPolynomial3DDefinition toYoGraphicPolynomial3DDefinition(YoPolynomialFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicPolynomial3DDefinition definition = new YoGraphicPolynomial3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setCoefficientsX(toYoDoubleListDefinition(yoGraphicFX.getCoefficientsX(), yoGraphicFX.getNumberOfCoefficientsX()));
      definition.setCoefficientsY(toYoDoubleListDefinition(yoGraphicFX.getCoefficientsY(), yoGraphicFX.getNumberOfCoefficientsY()));
      definition.setCoefficientsZ(toYoDoubleListDefinition(yoGraphicFX.getCoefficientsZ(), yoGraphicFX.getNumberOfCoefficientsZ()));
      definition.setReferenceFrame(CompositePropertyTools.toReferenceFramePropertyName(yoGraphicFX.getReferenceFrame()));
      definition.setStartTime(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getStartTime()));
      definition.setEndTime(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getEndTime()));
      definition.setSize(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getSize()));
      definition.setTimeResolution(CompositePropertyTools.toIntegerPropertyName(yoGraphicFX.getTimeResolution()));
      definition.setNumberOfDivisions(CompositePropertyTools.toIntegerPropertyName(yoGraphicFX.getNumberOfDivisions()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoListDefinition toYoDoubleListDefinition(List<? extends DoubleProperty> elements, IntegerProperty size)
   {
      YoListDefinition definition = new YoListDefinition();
      definition.setElements(CompositePropertyTools.toDoublePropertyNames(elements));
      definition.setSize(CompositePropertyTools.toIntegerPropertyName(size));
      return definition;
   }

   public static YoGraphicCoordinateSystem3DDefinition toYoGraphicCoordinateSystem3DDefinition(YoCoordinateSystemFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicCoordinateSystem3DDefinition definition = new YoGraphicCoordinateSystem3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPosition(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getPosition()));
      definition.setOrientation(CompositePropertyTools.toYoOrientation3DDefinition(yoGraphicFX.getOrientation()));
      definition.setBodyLength(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getBodyLength()));
      definition.setHeadLength(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getHeadLength()));
      definition.setBodyRadius(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getBodyRadius()));
      definition.setHeadRadius(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getHeadRadius()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicPolygonExtruded3DDefinition toYoGraphicPolygonExtruded3DDefinition(YoPolygonExtrudedFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicPolygonExtruded3DDefinition definition = new YoGraphicPolygonExtruded3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPosition(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getPosition()));
      definition.setOrientation(CompositePropertyTools.toYoOrientation3DDefinition(yoGraphicFX.getOrientation()));
      definition.setVertices(yoGraphicFX.getVertices().stream().map(CompositePropertyTools::toYoTuple2DDefinition).collect(Collectors.toList()));
      definition.setNumberOfVertices(CompositePropertyTools.toIntegerPropertyName(yoGraphicFX.getNumberOfVertices()));
      definition.setThickness(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getThickness()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicConvexPolytope3DDefinition toYoGraphicConvexPolytope3DDefinition(YoConvexPolytopeFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicConvexPolytope3DDefinition definition = new YoGraphicConvexPolytope3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPosition(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getPosition()));
      definition.setOrientation(CompositePropertyTools.toYoOrientation3DDefinition(yoGraphicFX.getOrientation()));
      definition.setVertices(yoGraphicFX.getVertices().stream().map(CompositePropertyTools::toYoTuple3DDefinition).collect(Collectors.toList()));
      definition.setNumberOfVertices(CompositePropertyTools.toIntegerPropertyName(yoGraphicFX.getNumberOfVertices()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicBox3DDefinition toYoGraphicBox3DDefinition(YoBoxFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicBox3DDefinition definition = new YoGraphicBox3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPosition(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getPosition()));
      definition.setOrientation(CompositePropertyTools.toYoOrientation3DDefinition(yoGraphicFX.getOrientation()));
      definition.setSize(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getSize()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicEllipsoid3DDefinition toYoGraphicEllipsoid3DDefinition(YoEllipsoidFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicEllipsoid3DDefinition definition = new YoGraphicEllipsoid3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPosition(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getPosition()));
      definition.setOrientation(CompositePropertyTools.toYoOrientation3DDefinition(yoGraphicFX.getOrientation()));
      definition.setRadii(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getRadii()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicSTPBox3DDefinition toYoGraphicSTPBox3DDefinition(YoSTPBoxFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicSTPBox3DDefinition definition = new YoGraphicSTPBox3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPosition(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getPosition()));
      definition.setOrientation(CompositePropertyTools.toYoOrientation3DDefinition(yoGraphicFX.getOrientation()));
      definition.setSize(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getSize()));
      definition.setMinimumMargin(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getMinimumMargin()));
      definition.setMaximumMargin(CompositePropertyTools.toDoublePropertyName(yoGraphicFX.getMaximumMargin()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static YoGraphicRamp3DDefinition toYoGraphicRamp3DDefinition(YoRampFX3D yoGraphicFX)
   {
      if (yoGraphicFX == null)
         return null;

      YoGraphicRamp3DDefinition definition = new YoGraphicRamp3DDefinition();

      definition.setName(yoGraphicFX.getName());
      definition.setVisible(yoGraphicFX.isVisible());
      definition.setPosition(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getPosition()));
      definition.setOrientation(CompositePropertyTools.toYoOrientation3DDefinition(yoGraphicFX.getOrientation()));
      definition.setSize(CompositePropertyTools.toYoTuple3DDefinition(yoGraphicFX.getSize()));
      definition.setColor(toColorDefinition(yoGraphicFX.getColor()));

      return definition;
   }

   public static ColorDefinition toColorDefinition(Supplier<Color> color)
   {
      if (color == null || color.get() == null)
         return null;
      double red = color.get().getRed();
      double green = color.get().getGreen();
      double blue = color.get().getBlue();
      double alpha = color.get().getOpacity();
      return new ColorDefinition(red, green, blue, alpha);
   }

   public static List<Shape> extractShapes(Group group)
   {
      if (group == null || group.getChildren().isEmpty())
         return Collections.emptyList();

      List<Shape> shapes = new ArrayList<>();

      for (Node child : group.getChildren())
      {
         if (child instanceof Shape)
            shapes.add((Shape) child);
         else if (child instanceof Group)
            shapes.addAll(extractShapes((Group) child));
      }

      return shapes;
   }

   public static List<Shape3D> extractShape3Ds(Collection<? extends Node> nodes)
   {
      if (nodes == null || nodes.isEmpty())
         return Collections.emptyList();

      List<Shape3D> shapes = new ArrayList<>();

      for (Node node : nodes)
      {
         if (node instanceof Shape3D)
            shapes.add((Shape3D) node);
         else if (node instanceof Group)
            shapes.addAll(extractShape3Ds(((Group) node).getChildren()));
      }

      return shapes;
   }

   public static boolean isAnyNull(Object... objects)
   {
      for (Object object : objects)
      {
         if (object == null)
            return true;
      }
      return false;
   }
}
