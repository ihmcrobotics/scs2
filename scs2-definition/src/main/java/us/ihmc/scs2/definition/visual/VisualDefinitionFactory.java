package us.ihmc.scs2.definition.visual;

import java.awt.image.BufferedImage;
import java.net.URL;
import java.net.URLClassLoader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.geometry.ArcTorus3DDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrudedPolygon2DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrusionDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.HemiEllipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition.SubMeshDefinition;
import us.ihmc.scs2.definition.geometry.Polygon2DDefinition;
import us.ihmc.scs2.definition.geometry.Polygon3DDefinition;
import us.ihmc.scs2.definition.geometry.PyramidBox3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.TruncatedCone3DDefinition;

/**
 * This class is a factory that simplifies the construction of {@code VisualDefinition}s.
 */
public class VisualDefinitionFactory
{
   private static final MaterialDefinition DEFAULT_MATERIAL = new MaterialDefinition(ColorDefinitions.Black());

   /**
    * The transform that is used for creating the next visual definition.
    */
   private final AffineTransform currentTransform = new AffineTransform();
   /**
    * Output of this factory: the list of visual definitions created so far.
    */
   private final List<VisualDefinition> visualDefinitions = new ArrayList<>();

   /**
    * Default material used for the method without an explicit material. It can be modified by the user
    * after construction of the factory.
    */
   private MaterialDefinition defaultMaterial = new MaterialDefinition(DEFAULT_MATERIAL);

   /**
    * New factory, ready to create visuals.
    */
   public VisualDefinitionFactory()
   {
   }

   /**
    * Output of this factory: the list of visual definitions created so far.
    * 
    * @return the visual definitions this factory has created.
    */
   public List<VisualDefinition> getVisualDefinitions()
   {
      return visualDefinitions;
   }

   /**
    * The default material to use with this factory.
    * <p>
    * This material is used when adding a visual to this factory without specifying any material or
    * color.
    * </p>
    * 
    * @return the default material.
    */
   public MaterialDefinition getDefaultMaterial()
   {
      return defaultMaterial;
   }

   /**
    * Changes the default material to a new material that only has the given {@code diffuseColor}
    * defined.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * 
    * @param diffuseColor the diffuse color used to create the new default material.
    * @see ColorDefinitions
    */
   public void setDefaultMaterial(ColorDefinition diffuseColor)
   {
      setDefaultMaterial(new MaterialDefinition(diffuseColor));
   }

   /**
    * Changes the default material.
    * 
    * @param defaultMaterial the new default material.
    */
   public void setDefaultMaterial(MaterialDefinition defaultMaterial)
   {
      this.defaultMaterial = defaultMaterial;
   }

   /**
    * The transform that is used to create the visual definitions. Modifying it results in modifying
    * the pose/scale of the next visual definitions, the ones that are already created do not change.
    * 
    * @return the current transform.
    */
   public AffineTransform getCurrentTransform()
   {
      return currentTransform;
   }

   /**
    * Sets the transform to identity, so the next visuals will have no additional offset regarding
    * their pose.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * 
    * @see AffineTransform#setIdentity()
    */
   public void identity()
   {
      identity(false);
   }

   /**
    * Sets the transform to identity, so the next visuals will have no additional offset regarding
    * their pose.
    * 
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    * @see AffineTransform#setIdentity()
    */
   public void identity(boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().setIdentity();
      }
      currentTransform.setIdentity();
   }

   /**
    * Appends the given {@code transform} to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param transform the rigid-body transform to append to the factory's transform. Not modified.
    * @see AffineTransform#multiply(RigidBodyTransformReadOnly)
    */
   public void appendTransform(RigidBodyTransformReadOnly transform)
   {
      appendTransform(transform, false);
   }

   /**
    * Appends the given {@code transform} to the factory's transform.
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param transform         the rigid-body transform to append to the factory's transform. Not
    *                          modified.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    * @see AffineTransform#multiply(RigidBodyTransformReadOnly)
    */
   public void appendTransform(RigidBodyTransformReadOnly transform, boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().multiply(transform);
      }
      currentTransform.multiply(transform);
   }

   /**
    * Appends a translation to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * Use this method if the translation (x, y, z) is expressed in the local coordinates described by
    * the factory's transform. Otherwise, use {@link #prependTranslation(double, double, double)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    * @param z the translation distance along the z-axis.
    * @see AffineTransform#appendTranslation(double, double, double)
    */
   public void appendTranslation(double x, double y, double z)
   {
      appendTranslation(x, y, z, false);
   }

   /**
    * Appends a translation to the factory's transform.
    * <p>
    * Use this method if the translation (x, y, z) is expressed in the local coordinates described by
    * the factory's transform. Otherwise, use {@link #prependTranslation(double, double, double)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param x                 the translation distance along the x-axis.
    * @param y                 the translation distance along the y-axis.
    * @param z                 the translation distance along the z-axis.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    * @see AffineTransform#appendTranslation(double, double, double)
    */
   public void appendTranslation(double x, double y, double z, boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().appendTranslation(x, y, z);
      }
      currentTransform.appendTranslation(x, y, z);
   }

   /**
    * Appends a translation to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by the
    * factory's transform. Otherwise, use {@link #prependTranslation(Tuple3DReadOnly)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param translation tuple containing the translation to apply to the factory's transform. Not
    *                    modified.
    * @see AffineTransform#appendTranslation(Tuple3DReadOnly)
    */
   public void appendTranslation(Tuple3DReadOnly translation)
   {
      appendTranslation(translation, false);
   }

   /**
    * Appends a translation to the factory's transform.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by the
    * factory's transform. Otherwise, use {@link #prependTranslation(Tuple3DReadOnly)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param translation       tuple containing the translation to apply to the factory's transform.
    *                          Not modified.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    * @see AffineTransform#appendTranslation(Tuple3DReadOnly)
    */
   public void appendTranslation(Tuple3DReadOnly translation, boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().appendTranslation(translation);
      }
      currentTransform.appendTranslation(translation);
   }

   /**
    * Convenience method that creates an axis-angle and appends it to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param rotationAngle the rotation angle magnitude in radians.
    * @param rotationAxis  the axis of rotation, expected to be a unit-vector. Not modified.
    * @see Orientation3DBasics#append(Orientation3DReadOnly)
    */
   public void appendRotation(double rotationAngle, Vector3DReadOnly rotationAxis)
   {
      appendRotation(rotationAngle, rotationAxis, false);
   }

   /**
    * Convenience method that creates an axis-angle and appends it to the factory's transform.
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param rotationAngle     the rotation angle magnitude in radians.
    * @param rotationAxis      the axis of rotation, expected to be a unit-vector. Not modified.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    * @see Orientation3DBasics#append(Orientation3DReadOnly)
    */
   public void appendRotation(double rotationAngle, Vector3DReadOnly rotationAxis, boolean applyToAllVisuals)
   {
      appendRotation(new AxisAngle(rotationAxis, rotationAngle), applyToAllVisuals);
   }

   /**
    * Appends the given orientation to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param orientation the orientation to append to the factory's transform. Not modified.
    * @see Orientation3DBasics#append(Orientation3DReadOnly)
    */
   public void appendRotation(Orientation3DReadOnly orientation)
   {
      appendRotation(orientation, false);
   }

   /**
    * Appends the given orientation to the factory's transform.
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param orientation       the orientation to append to the factory's transform. Not modified.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    * @see Orientation3DBasics#append(Orientation3DReadOnly)
    */
   public void appendRotation(Orientation3DReadOnly orientation, boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().appendOrientation(orientation);
      }
      currentTransform.appendOrientation(orientation);
   }

   /**
    * Appends the scale factor to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param scaleFactor the scale to apply.
    */
   public void appendScale(double scaleFactor)
   {
      appendScale(scaleFactor, false);
   }

   /**
    * Appends the scale factor to the factory's transform.
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param scaleFactor       the scale to apply.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    */
   public void appendScale(double scaleFactor, boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().appendScale(scaleFactor);
      }
      currentTransform.appendScale(scaleFactor);
   }

   /**
    * Appends the scale factors to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * If the scale factor are different for each axis, pay attention to whether you should use append
    * or prepend. When using append, the scale factors are applied to the local axes that the factory's
    * transform describe. If the scale factors are meant to scale the axes of the world frame, consider
    * using {@link #prependScale(Tuple3DReadOnly)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param scaleFactors the scale to append.
    */
   public void appendScale(Tuple3DReadOnly scaleFactors)
   {
      currentTransform.appendScale(scaleFactors);
   }

   /**
    * Appends the scale factors to the factory's transform.
    * <p>
    * If the scale factor are different for each axis, pay attention to whether you should use append
    * or prepend. When using append, the scale factors are applied to the local axes that the factory's
    * transform describe. If the scale factors are meant to scale the axes of the world frame, consider
    * using {@link #prependScale(Tuple3DReadOnly)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param scaleFactors      the scale to append.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    */
   public void appendScale(Tuple3DReadOnly scaleFactors, boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().appendScale(scaleFactors);
      }
      currentTransform.appendScale(scaleFactors);
   }

   /**
    * Prepends the given {@code transform} to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param transform the rigid-body transform to prepend to the factory's transform. Not modified.
    * @see AffineTransform#multiply(RigidBodyTransformReadOnly)
    */
   public void prependTransform(RigidBodyTransformReadOnly transform)
   {
      prependTransform(transform, false);
   }

   /**
    * Prepends the given {@code transform} to the factory's transform.
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param transform         the rigid-body transform to prepend to the factory's transform. Not
    *                          modified.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    * @see AffineTransform#multiply(RigidBodyTransformReadOnly)
    */
   public void prependTransform(RigidBodyTransformReadOnly transform, boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().preMultiply(transform);
      }
      currentTransform.preMultiply(transform);
   }

   /**
    * Adds the translation (x, y, z) to the factory's transform assuming it is expressed in the world
    * coordinates.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * If the translation is expressed in the local coordinates described by the factory's transform,
    * use {@link #appendTranslation(double, double, double)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    * @param z the translation distance along the z-axis.
    */
   public void prependTranslation(double x, double y, double z)
   {
      prependTranslation(x, y, z, false);
   }

   /**
    * Adds the translation (x, y, z) to the factory's transform assuming it is expressed in the world
    * coordinates.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * If the translation is expressed in the local coordinates described by the factory's transform,
    * use {@link #appendTranslation(double, double, double)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param x                 the translation distance along the x-axis.
    * @param y                 the translation distance along the y-axis.
    * @param z                 the translation distance along the z-axis.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    */
   public void prependTranslation(double x, double y, double z, boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().prependTranslation(x, y, z);
      }
      currentTransform.prependTranslation(x, y, z);
   }

   /**
    * Adds the given {@code translation} to the factory's transform assuming it is expressed in the
    * world frame.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by the factory's
    * transform, use {@link #appendTranslation(Tuple3DReadOnly)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    */
   public void prependTranslation(Tuple3DReadOnly translation)
   {
      prependTranslation(translation, false);
   }

   /**
    * Adds the given {@code translation} to the factory's transform assuming it is expressed in the
    * world frame.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by the factory's
    * transform, use {@link #appendTranslation(Tuple3DReadOnly)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param translation       tuple containing the translation to apply to this pose 3D. Not modified.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    */

   public void prependTranslation(Tuple3DReadOnly translation, boolean applyToAllVisuals)
   {
      prependTranslation(translation.getX(), translation.getY(), translation.getZ(), applyToAllVisuals);
   }

   /**
    * Convenience method that creates an axis-angle and prepends it to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param rotationAngle the rotation angle magnitude in radians.
    * @param rotationAxis  the axis of rotation, expected to be a unit-vector. Not modified.
    * @see Orientation3DBasics#prepend(Orientation3DReadOnly)
    */
   public void prependRotation(double rotationAngle, Vector3DReadOnly rotationAxis)
   {
      prependRotation(rotationAngle, rotationAxis);
   }

   /**
    * Convenience method that creates an axis-angle and prepends it to the factory's transform.
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param rotationAngle     the rotation angle magnitude in radians.
    * @param rotationAxis      the axis of rotation, expected to be a unit-vector. Not modified.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    * @see Orientation3DBasics#prepend(Orientation3DReadOnly)
    */
   public void prependRotation(double rotationAngle, Vector3DReadOnly rotationAxis, boolean applyToAllVisuals)
   {
      prependRotation(new AxisAngle(rotationAxis, rotationAngle), applyToAllVisuals);
   }

   /**
    * Prepends the given orientation to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param orientation the orientation to prepend to the factory's transform. Not modified.
    * @see Orientation3DBasics#prepend(Orientation3DReadOnly)
    */
   public void prependRotation(Orientation3DReadOnly orientation)
   {
      prependRotation(orientation, false);
   }

   /**
    * Prepends the given orientation to the factory's transform.
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    *
    * @param orientation       the orientation to prepend to the factory's transform. Not modified.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    * @see Orientation3DBasics#prepend(Orientation3DReadOnly)
    */
   public void prependRotation(Orientation3DReadOnly orientation, boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().prependOrientation(orientation);
      }
      currentTransform.prependOrientation(orientation);
   }

   /**
    * Prepends the scale factor to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param scaleFactor the scale to apply.
    */
   public void prependScale(double scaleFactor)
   {
      prependScale(scaleFactor, false);
   }

   /**
    * Prepends the scale factor to the factory's transform.
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param scaleFactor       the scale to apply.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    */
   public void prependScale(double scaleFactor, boolean applyToAllVisuals)
   {
      prependScale(new Vector3D(scaleFactor, scaleFactor, scaleFactor), applyToAllVisuals);
   }

   /**
    * Prepends the scale factors to the factory's transform.
    * <p>
    * Only the next visuals to be created are affected, the ones already created do not change.
    * </p>
    * <p>
    * If the scale factor are different for each axis, pay attention to whether you should use append
    * or prepend. When using prepend, the scale factors are applied to the world frame axes. If the
    * scale factors are meant to scale the local axes that the factory's transform describe, consider
    * using {@link #appendScale(Tuple3DReadOnly)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param scaleFactors the scales to prepend.
    */
   public void prependScale(Tuple3DReadOnly scaleFactors)
   {
      prependScale(scaleFactors, false);
   }

   /**
    * Prepends the scale factors to the factory's transform.
    * <p>
    * If the scale factor are different for each axis, pay attention to whether you should use append
    * or prepend. When using prepend, the scale factors are applied to the world frame axes. If the
    * scale factors are meant to scale the local axes that the factory's transform describe, consider
    * using {@link #appendScale(Tuple3DReadOnly)}.
    * </p>
    * <p>
    * The factory's transform can be reset to identity at any time using {@link #identity()}.
    * </p>
    * 
    * @param scaleFactors      the scales to prepend.
    * @param applyToAllVisuals whether this operation should be applied to the existing visuals as
    *                          well.
    */
   public void prependScale(Tuple3DReadOnly scaleFactors, boolean applyToAllVisuals)
   {
      if (applyToAllVisuals)
      {
         for (int i = 0; i < visualDefinitions.size(); i++)
            visualDefinitions.get(i).getOriginPose().prependScale(scaleFactors);
      }
      currentTransform.prependScale(scaleFactors);
   }

   /**
    * Adds a visual definition to this factory. It is not modified, but it is stored in this factory
    * and is subject to future modifications done through this factory interface.
    * 
    * @param visual the visual definition to add to this factory. The visual will be part of
    *               {@link #getVisualDefinitions()}.
    * @return the visual itself.
    */
   public VisualDefinition addVisualDefinition(VisualDefinition visual)
   {
      visualDefinitions.add(visual);
      return visual;
   }

   /**
    * Creates and adds a new visual definition for the given geometry.
    * <p>
    * The pose of the new visual is initialized to the current value of {@link #getCurrentTransform()}.
    * </p>
    * <p>
    * The new visual will use {@link #getDefaultMaterial()} for its material definition.
    * </p>
    * 
    * @param geometryDefinition the geometry of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addGeometryDefinition(GeometryDefinition geometryDefinition)
   {
      return addGeometryDefinition(geometryDefinition, defaultMaterial);
   }

   /**
    * Creates and adds a new visual definition for the given geometry.
    * <p>
    * The pose of the new visual is initialized to the current value of {@link #getCurrentTransform()}.
    * </p>
    * 
    * @param geometryDefinition     the geometry of the new visual.
    * @param diffuseColorDefinition the diffuse color of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addGeometryDefinition(GeometryDefinition geometryDefinition, ColorDefinition diffuseColorDefinition)
   {
      return addGeometryDefinition(geometryDefinition, new MaterialDefinition(diffuseColorDefinition));
   }

   /**
    * Creates and adds a new visual definition for the given geometry.
    * <p>
    * The pose of the new visual is initialized to the current value of {@link #getCurrentTransform()}.
    * </p>
    * 
    * @param geometryDefinition the geometry of the new visual.
    * @param materialDefinition the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addGeometryDefinition(GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      VisualDefinition visual = new VisualDefinition(new AffineTransform(currentTransform), geometryDefinition, materialDefinition);
      return addVisualDefinition(visual);
   }

   /**
    * Creates and adds a new visual that can be used to import the model file.
    * 
    * @param fileURL the URL pointing to the model file.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(URL fileURL)
   {
      return addModelFile(fileURL, null);
   }

   /**
    * Creates and adds a new visual that can be used to import the model file.
    * <p>
    * The given material is typically used to override the model's material.
    * </p>
    * 
    * @param fileURL            the URL pointing to the model file.
    * @param materialDefinition the material expected to be used to override the model's material.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(URL fileURL, MaterialDefinition materialDefinition)
   {
      if (fileURL == null)
      {
         LogTools.error("The given fileURL is null.");
         return null;
      }

      return addModelFile(fileURL.getFile(), materialDefinition);
   }

   /**
    * Creates and adds a new visual that can be used to import the model file.
    * 
    * @param fileName the path to the model file.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(String fileName)
   {
      return addModelFile(fileName, null);
   }

   /**
    * Creates and adds a new visual that can be used to import the model file.
    * 
    * @param fileName            the path to the model file.
    * @param resourceDirectories the directories where resources potentially needed by the model file
    *                            can be found.
    * @param resourceClassLoader allows to provide a custom resource loader. Particularly useful for
    *                            loading a model file that is not on the class path. See
    *                            {@link URLClassLoader}.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(String fileName, List<String> resourceDirectories, ClassLoader resourceClassLoader)
   {
      return addModelFile(fileName, resourceDirectories, resourceClassLoader, null);
   }

   /**
    * Creates and adds a new visual that can be used to import the model file.
    * <p>
    * The given material is typically used to override the model's material.
    * </p>
    * 
    * @param fileName            the path to the model file.
    * @param submesh             the name of the submesh. The submesh is expected to be defined in the
    *                            model file.
    * @param centerSubmesh       when {@code true}, the vertices of the submesh are expected to be
    *                            centered at (0,0,0), removing any transform on the submesh.
    * @param resourceDirectories the directories where resources potentially needed by the model file
    *                            can be found.
    * @param resourceClassLoader allows to provide a custom resource loader. Particularly useful for
    *                            loading a model file that is not on the class path. See
    *                            {@link URLClassLoader}.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(String fileName,
                                        String submesh,
                                        boolean centerSubmesh,
                                        List<String> resourceDirectories,
                                        ClassLoader resourceClassLoader)
   {
      return addModelFile(fileName, submesh, centerSubmesh, resourceDirectories, resourceClassLoader, null);
   }

   /**
    * Creates and adds a new visual that can be used to import the model file.
    * <p>
    * The given material is typically used to override the model's material.
    * </p>
    * 
    * @param fileName           the path to the model file.
    * @param materialDefinition the material expected to be used to override the model's material.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(String fileName, MaterialDefinition materialDefinition)
   {
      if (fileName == null || fileName.equals(""))
      {
         LogTools.error("Error importing model file, filename is null or empty");
         return null;
      }

      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition();
      modelFileGeometryDefinition.setFileName(fileName);
      return addGeometryDefinition(modelFileGeometryDefinition, materialDefinition);
   }

   /**
    * Creates and adds a new visual that can be used to import the model file.
    * <p>
    * The given material is typically used to override the model's material.
    * </p>
    * 
    * @param fileName            the path to the model file.
    * @param resourceDirectories the directories where resources potentially needed by the model file
    *                            can be found.
    * @param resourceClassLoader allows to provide a custom resource loader. Particularly useful for
    *                            loading a model file that is not on the class path. See
    *                            {@link URLClassLoader}.
    * @param materialDefinition  the material expected to be used to override the model's material.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(String fileName,
                                        List<String> resourceDirectories,
                                        ClassLoader resourceClassLoader,
                                        MaterialDefinition materialDefinition)
   {
      return addModelFile(fileName, null, false, resourceDirectories, resourceClassLoader, materialDefinition);
   }

   /**
    * Creates and adds a new visual that can be used to import the model file.
    * <p>
    * The given material is typically used to override the model's material.
    * </p>
    * 
    * @param fileName           the path to the model file.
    * @param materialDefinition the material expected to be used to override the model's material.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(String fileName,
                                        String submesh,
                                        boolean centerSubmesh,
                                        List<String> resourceDirectories,
                                        ClassLoader resourceClassLoader,
                                        MaterialDefinition materialDefinition)
   {
      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition(fileName);
      modelFileGeometryDefinition.setResourceDirectories(resourceDirectories);
      if (submesh != null)
         modelFileGeometryDefinition.setSubmeshes(Collections.singletonList(new SubMeshDefinition(submesh, centerSubmesh)));
      modelFileGeometryDefinition.setResourceClassLoader(resourceClassLoader);
      return addGeometryDefinition(modelFileGeometryDefinition, materialDefinition);
   }

   /**
    * Creates and adds a new 3D coordinate system.
    * 
    * @param length
    */
   public void addCoordinateSystem(double length)
   {
      addCoordinateSystem(length, defaultMaterial);
   }

   public void addCoordinateSystem(double length, MaterialDefinition materialDefinition)
   {
      addCoordinateSystem(length,
                          new MaterialDefinition(ColorDefinitions.Red()),
                          new MaterialDefinition(ColorDefinitions.White()),
                          new MaterialDefinition(ColorDefinitions.Blue()),
                          materialDefinition);
   }

   public void addCoordinateSystem(double length,
                                   MaterialDefinition xAxisMaterial,
                                   MaterialDefinition yAxisMaterial,
                                   MaterialDefinition zAxisMaterial,
                                   MaterialDefinition arrowMaterial)
   {
      appendRotation(Math.PI / 2.0, Axis3D.Y);
      addArrow(length, xAxisMaterial, arrowMaterial);
      appendRotation(-Math.PI / 2.0, Axis3D.Y);
      appendRotation(-Math.PI / 2.0, Axis3D.X);
      addArrow(length, yAxisMaterial, arrowMaterial);
      appendRotation(Math.PI / 2.0, Axis3D.X);
      addArrow(length, zAxisMaterial, arrowMaterial);
   }

   public void add(Shape3DReadOnly shape, MaterialDefinition materialDefinition)
   {
      if (shape instanceof Box3DReadOnly)
      {
         Box3DReadOnly box = (Box3DReadOnly) shape;
         appendTransform(box.getPose());
         addBox(box.getSizeX(), box.getSizeY(), box.getSizeZ(), true, materialDefinition);
      }
      else if (shape instanceof Capsule3DReadOnly)
      {
         Capsule3DReadOnly capsule = (Capsule3DReadOnly) shape;
         appendTranslation(capsule.getPosition());
         appendRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(capsule.getAxis()));
         addCapsule(capsule.getRadius(),
                    capsule.getLength() + 2.0 * capsule.getRadius(), // the 2nd term is removed internally.
                    materialDefinition);
      }
      else if (shape instanceof Cylinder3DReadOnly)
      {
         Cylinder3DReadOnly cylinder = (Cylinder3DReadOnly) shape;
         appendTranslation(cylinder.getPosition());
         appendRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(cylinder.getAxis()));
         appendTranslation(0.0, 0.0, -cylinder.getHalfLength());
         addCylinder(cylinder.getLength(), cylinder.getRadius(), materialDefinition);
      }
      else if (shape instanceof Ellipsoid3DReadOnly)
      {
         Ellipsoid3DReadOnly ellipsoid = (Ellipsoid3DReadOnly) shape;
         appendTransform(ellipsoid.getPose());
         addEllipsoid(ellipsoid.getRadiusX(), ellipsoid.getRadiusY(), ellipsoid.getRadiusZ(), materialDefinition);
      }
      else if (shape instanceof PointShape3DReadOnly)
      {
         PointShape3DReadOnly pointShape = (PointShape3DReadOnly) shape;
         appendTranslation(pointShape);
         addSphere(0.005, materialDefinition); // Arbitrary radius
      }
      else if (shape instanceof Ramp3DReadOnly)
      {
         Ramp3DReadOnly ramp = (Ramp3DReadOnly) shape;
         appendTransform(ramp.getPose());
         appendTranslation(-0.5 * ramp.getSizeX(), 0.0, 0.0);
         addRamp(ramp.getSizeX(), ramp.getSizeY(), ramp.getSizeZ(), materialDefinition);
      }
      else if (shape instanceof Sphere3DReadOnly)
      {
         Sphere3DReadOnly sphere = (Sphere3DReadOnly) shape;
         appendTranslation(sphere.getPosition());
         addSphere(sphere.getRadius(), materialDefinition);
      }
      else if (shape instanceof Torus3DReadOnly)
      {
         Torus3DReadOnly torus = (Torus3DReadOnly) shape;
         appendTranslation(torus.getPosition());
         appendRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(torus.getAxis()));
         addArcTorus(0.0, 2.0 * Math.PI, torus.getRadius(), torus.getTubeRadius(), materialDefinition);
      }
      else
      {
         // TODO Implement for ConvexPolytope3D
         throw new UnsupportedOperationException("Unsupported shape: " + shape);
      }
   }

   public void addArrow(double length, MaterialDefinition baseMaterial, MaterialDefinition headMaterial)
   {
      double coneHeight = 0.1 * length;
      double cylinderHeight = length - coneHeight;
      double radius = 0.02 * length;
      double coneRadius = 2.0 * radius;

      appendTranslation(0.0, 0.0, 0.5 * cylinderHeight);
      addCylinder(cylinderHeight, radius, baseMaterial);
      appendTranslation(0.0, 0.0, 0.5 * cylinderHeight);
      addCone(coneHeight, coneRadius, headMaterial);
      appendTranslation(0.0, 0.0, -cylinderHeight);
   }

   public VisualDefinition addBox(double lengthX, double widthY, double heightZ)
   {
      return addBox(lengthX, widthY, heightZ, defaultMaterial);
   }

   public VisualDefinition addBox(double lengthX, double widthY, double heightZ, MaterialDefinition materialDefinition)
   {
      return addBox(lengthX, widthY, heightZ, true, materialDefinition);
   }

   public VisualDefinition addBox(double lengthX, double widthY, double heightZ, boolean centeredInTheCenter, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Box3DDefinition(lengthX, widthY, heightZ, centeredInTheCenter), materialDefinition);
   }

   public VisualDefinition addRamp(double lengthX, double widthY, double heightZ)
   {
      return addRamp(lengthX, widthY, heightZ, defaultMaterial);
   }

   public VisualDefinition addRamp(double lengthX, double widthY, double heightZ, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Ramp3DDefinition(lengthX, widthY, heightZ), materialDefinition);
   }

   public VisualDefinition addSphere(double radius)
   {
      return addSphere(radius, defaultMaterial);
   }

   public VisualDefinition addSphere(double radius, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Sphere3DDefinition(radius), materialDefinition);
   }

   public VisualDefinition addCapsule(double radius, double height)
   {
      return addCapsule(radius, height, defaultMaterial);
   }

   public VisualDefinition addCapsule(double radius, double height, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Capsule3DDefinition(height, radius), materialDefinition);
   }

   public VisualDefinition addEllipsoid(double xRadius, double yRadius, double zRadius)
   {
      return addEllipsoid(xRadius, yRadius, zRadius, defaultMaterial);
   }

   public VisualDefinition addEllipsoid(double xRadius, double yRadius, double zRadius, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Ellipsoid3DDefinition(xRadius, yRadius, zRadius), materialDefinition);
   }

   public VisualDefinition addCylinder(double height, double radius)
   {
      return addCylinder(height, radius, defaultMaterial);
   }

   public VisualDefinition addCylinder(double height, double radius, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Cylinder3DDefinition(height, radius, false), materialDefinition);
   }

   public VisualDefinition addCone(double height, double radius)
   {
      return addCone(height, radius, defaultMaterial);
   }

   public VisualDefinition addCone(double height, double radius, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Cone3DDefinition(height, radius), materialDefinition);
   }

   public VisualDefinition addGenTruncatedCone(double height, double bx, double by, double tx, double ty)
   {
      return addGenTruncatedCone(height, bx, by, tx, ty, defaultMaterial);
   }

   public VisualDefinition addGenTruncatedCone(double height, double bx, double by, double tx, double ty, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new TruncatedCone3DDefinition(height, tx, ty, bx, by), materialDefinition);
   }

   public VisualDefinition addHemiEllipsoid(double xRad, double yRad, double zRad)
   {
      return addHemiEllipsoid(xRad, yRad, zRad, defaultMaterial);
   }

   public VisualDefinition addHemiEllipsoid(double xRad, double yRad, double zRad, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new HemiEllipsoid3DDefinition(xRad, yRad, zRad), materialDefinition);
   }

   public VisualDefinition addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius)
   {
      return addArcTorus(startAngle, endAngle, majorRadius, minorRadius, defaultMaterial);
   }

   public VisualDefinition addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new ArcTorus3DDefinition(startAngle, endAngle, majorRadius, minorRadius), materialDefinition);
   }

   public VisualDefinition addPyramidCube(double lx, double ly, double lz, double lh)
   {
      return addPyramidCube(lx, ly, lz, lh, defaultMaterial);
   }

   public VisualDefinition addPyramidCube(double lx, double ly, double lz, double lh, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new PyramidBox3DDefinition(lx, ly, lz, lh), materialDefinition);
   }

   public VisualDefinition addPolygon(List<? extends Point3DReadOnly> polygonPoints)
   {
      return addPolygon(polygonPoints, defaultMaterial);
   }

   public VisualDefinition addPolygon(List<? extends Point3DReadOnly> polygonPoints, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Polygon3DDefinition(Polygon3DDefinition.toPoint3DDefinitionList(polygonPoints), true), materialDefinition);
   }

   public VisualDefinition addPolygon(ConvexPolygon2DReadOnly convexPolygon2d, MaterialDefinition materialDefinition)
   {
      List<Point3D> polygonPoints = new ArrayList<>();
      int numPoints = convexPolygon2d.getNumberOfVertices();

      for (int i = 0; i < numPoints; i++)
      {
         Point2DReadOnly planarPoint = convexPolygon2d.getVertex(i);
         polygonPoints.add(new Point3D(planarPoint.getX(), planarPoint.getY(), 0.0));
      }

      return addPolygon(polygonPoints, materialDefinition);
   }

   public VisualDefinition addPolygon(ConvexPolygon2DReadOnly convexPolygon2d)
   {
      return addPolygon(convexPolygon2d, defaultMaterial);
   }

   public void addPolygons(RigidBodyTransformReadOnly transform, List<? extends ConvexPolygon2DReadOnly> convexPolygon2D)
   {
      addPolygons(transform, convexPolygon2D, defaultMaterial);
   }

   public void addPolygons(RigidBodyTransformReadOnly transform, List<? extends ConvexPolygon2DReadOnly> convexPolygon2D, MaterialDefinition materialDefinition)
   {
      appendTransform(transform);

      for (int i = 0; i < convexPolygon2D.size(); i++)
      {
         ConvexPolygon2DReadOnly convexPolygon = convexPolygon2D.get(i);
         addGeometryDefinition(new Polygon2DDefinition(Polygon2DDefinition.toPoint2DDefinitionList(convexPolygon.getPolygonVerticesView()),
                                                       !convexPolygon.isClockwiseOrdered()),
                               materialDefinition);
      }

      RigidBodyTransform transformLocal = new RigidBodyTransform(transform);
      transformLocal.invert();
      appendTransform(transformLocal);
   }

   public VisualDefinition addPolygon(Point3DReadOnly[] polygonPoint)
   {
      return addPolygon(polygonPoint, defaultMaterial);
   }

   public VisualDefinition addPolygon(Point3DReadOnly[] polygonPoints, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new Polygon3DDefinition(Polygon3DDefinition.toPoint3DDefinitionList(polygonPoints), true), materialDefinition);
   }

   public VisualDefinition addPolygon(MaterialDefinition materialDefinition, Point3DReadOnly... polygonPoints)
   {
      return addPolygon(polygonPoints, materialDefinition);
   }

   public VisualDefinition addExtrudedPolygon(ConvexPolygon2DReadOnly convexPolygon2d, double height)
   {
      return addExtrudedPolygon(convexPolygon2d, height, defaultMaterial);
   }

   public VisualDefinition addExtrudedPolygon(ConvexPolygon2DReadOnly convexPolygon2d, double height, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new ExtrudedPolygon2DDefinition(ExtrudedPolygon2DDefinition.toPoint2DDefinitionList(convexPolygon2d.getPolygonVerticesView()),
                                                                   true,
                                                                   height),
                                   materialDefinition);
   }

   public VisualDefinition addExtrudedPolygon(List<? extends Point2DReadOnly> polygonPoints, double height)
   {
      return addExtrudedPolygon(polygonPoints, height, defaultMaterial);
   }

   public VisualDefinition addExtrudedPolygon(List<? extends Point2DReadOnly> polygonPoints, double height, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new ExtrudedPolygon2DDefinition(ExtrudedPolygon2DDefinition.toPoint2DDefinitionList(polygonPoints), true, height),
                                   materialDefinition);
   }

   public VisualDefinition addExtrusion(BufferedImage bufferedImageToExtrude, double thickness, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new ExtrusionDefinition(bufferedImageToExtrude, thickness), materialDefinition);
   }

   public VisualDefinition addText(String text, double thickness, MaterialDefinition materialDefinition)
   {
      return addGeometryDefinition(new ExtrusionDefinition(text, thickness), materialDefinition);
   }
}
