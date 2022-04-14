package us.ihmc.scs2.definition.visual;

import java.net.URL;
import java.net.URLClassLoader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
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
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.geometry.ArcTorus3DDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrudedPolygon2DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.HemiEllipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition.SubMeshDefinition;
import us.ihmc.scs2.definition.geometry.Polygon2DDefinition;
import us.ihmc.scs2.definition.geometry.Polygon3DDefinition;
import us.ihmc.scs2.definition.geometry.PyramidBox3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.Torus3DDefinition;
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
   private final AffineTransform previousTransform = new AffineTransform();
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
    * Saves the current transform.
    */
   public void saveCurrentTransform()
   {
      previousTransform.set(currentTransform);
   }

   /**
    * Resets the current transform to its value when it was last saved via
    * {@link #saveCurrentTransform()}.
    */
   public void resetCurrentTransform()
   {
      currentTransform.set(previousTransform);
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
    * @param geometry the geometry of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addGeometryDefinition(GeometryDefinition geometry)
   {
      return addGeometryDefinition(geometry, defaultMaterial);
   }

   /**
    * Creates and adds a new visual definition for the given geometry.
    * <p>
    * The pose of the new visual is initialized to the current value of {@link #getCurrentTransform()}.
    * </p>
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * 
    * @param geometry     the geometry of the new visual.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addGeometryDefinition(GeometryDefinition geometry, ColorDefinition diffuseColor)
   {
      return addGeometryDefinition(geometry, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds a new visual definition for the given geometry.
    * <p>
    * The pose of the new visual is initialized to the current value of {@link #getCurrentTransform()}.
    * </p>
    * 
    * @param geometry the geometry of the new visual.
    * @param material the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addGeometryDefinition(GeometryDefinition geometry, MaterialDefinition material)
   {
      VisualDefinition visual = new VisualDefinition(new AffineTransform(currentTransform), geometry, material);
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
    * @param fileURL  the URL pointing to the model file.
    * @param material the material expected to be used to override the model's material.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(URL fileURL, MaterialDefinition material)
   {
      if (fileURL == null)
      {
         LogTools.error("The given fileURL is null.");
         return null;
      }

      return addModelFile(fileURL.getFile(), material);
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
    * @param fileName the path to the model file.
    * @param material the material expected to be used to override the model's material.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(String fileName, MaterialDefinition material)
   {
      if (fileName == null || fileName.equals(""))
      {
         LogTools.error("Error importing model file, filename is null or empty");
         return null;
      }

      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition();
      modelFileGeometryDefinition.setFileName(fileName);
      return addGeometryDefinition(modelFileGeometryDefinition, material);
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
    * @param material            the material expected to be used to override the model's material.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(String fileName, List<String> resourceDirectories, ClassLoader resourceClassLoader, MaterialDefinition material)
   {
      return addModelFile(fileName, null, false, resourceDirectories, resourceClassLoader, material);
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
    * @param material            the material expected to be used to override the model's material.
    * @return the new visual.
    */
   public VisualDefinition addModelFile(String fileName,
                                        String submesh,
                                        boolean centerSubmesh,
                                        List<String> resourceDirectories,
                                        ClassLoader resourceClassLoader,
                                        MaterialDefinition material)
   {
      ModelFileGeometryDefinition modelFileGeometryDefinition = new ModelFileGeometryDefinition(fileName);
      modelFileGeometryDefinition.setResourceDirectories(resourceDirectories);
      if (submesh != null)
         modelFileGeometryDefinition.setSubmeshes(Collections.singletonList(new SubMeshDefinition(submesh, centerSubmesh)));
      modelFileGeometryDefinition.setResourceClassLoader(resourceClassLoader);
      return addGeometryDefinition(modelFileGeometryDefinition, material);
   }

   /**
    * Creates and adds a new visual that represents the given shape.
    * 
    * @param shape the 3D shape to creates a visual definition for. Not modified.
    * @return the new visual or {@code null} if the shape is not supported.
    */
   public VisualDefinition addShape(Shape3DReadOnly shape)
   {
      return addShape(shape, defaultMaterial);
   }

   /**
    * Creates and adds a new visual that represents the given shape.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * 
    * @param shape        the 3D shape to creates a visual definition for. Not modified.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual or {@code null} if the shape is not supported.
    * @see ColorDefinitions
    */
   public VisualDefinition addShape(Shape3DReadOnly shape, ColorDefinition diffuseColor)
   {
      return addShape(shape, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds a new visual that represents the given shape.
    * 
    * @param shape    the 3D shape to creates a visual definition for. Not modified.
    * @param material the material of the new visual.
    * @return the new visual or {@code null} if the shape is not supported.
    */
   public VisualDefinition addShape(Shape3DReadOnly shape, MaterialDefinition material)
   {
      saveCurrentTransform();
      VisualDefinition visual;

      if (shape instanceof Box3DReadOnly)
      {
         Box3DReadOnly box = (Box3DReadOnly) shape;
         appendTransform(box.getPose());
         visual = addBox(box.getSizeX(), box.getSizeY(), box.getSizeZ(), true, material);
      }
      else if (shape instanceof Capsule3DReadOnly)
      {
         Capsule3DReadOnly capsule = (Capsule3DReadOnly) shape;
         appendTranslation(capsule.getPosition());
         appendRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(capsule.getAxis()));
         visual = addCapsule(capsule.getLength(), capsule.getRadius(), material);
      }
      else if (shape instanceof ConvexPolytope3DReadOnly)
      {
         ConvexPolytope3DReadOnly convexPolytope = (ConvexPolytope3DReadOnly) shape;
         visual = addGeometryDefinition(new ConvexPolytope3DDefinition(new ConvexPolytope3D(convexPolytope)), material);
      }
      else if (shape instanceof Cylinder3DReadOnly)
      {
         Cylinder3DReadOnly cylinder = (Cylinder3DReadOnly) shape;
         appendTranslation(cylinder.getPosition());
         appendRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(cylinder.getAxis()));
         visual = addCylinder(cylinder.getLength(), cylinder.getRadius(), material);
      }
      else if (shape instanceof Ellipsoid3DReadOnly)
      {
         Ellipsoid3DReadOnly ellipsoid = (Ellipsoid3DReadOnly) shape;
         appendTransform(ellipsoid.getPose());
         visual = addEllipsoid(ellipsoid.getRadiusX(), ellipsoid.getRadiusY(), ellipsoid.getRadiusZ(), material);
      }
      else if (shape instanceof PointShape3DReadOnly)
      {
         PointShape3DReadOnly pointShape = (PointShape3DReadOnly) shape;
         appendTranslation(pointShape);
         visual = addSphere(0.005, material); // Arbitrary radius
      }
      else if (shape instanceof Ramp3DReadOnly)
      {
         Ramp3DReadOnly ramp = (Ramp3DReadOnly) shape;
         appendTransform(ramp.getPose());
         appendTranslation(-0.5 * ramp.getSizeX(), 0.0, 0.0);
         visual = addRamp(ramp.getSizeX(), ramp.getSizeY(), ramp.getSizeZ(), material);
      }
      else if (shape instanceof Sphere3DReadOnly)
      {
         Sphere3DReadOnly sphere = (Sphere3DReadOnly) shape;
         appendTranslation(sphere.getPosition());
         visual = addSphere(sphere.getRadius(), material);
      }
      else if (shape instanceof Torus3DReadOnly)
      {
         Torus3DReadOnly torus = (Torus3DReadOnly) shape;
         appendTranslation(torus.getPosition());
         appendRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(torus.getAxis()));
         visual = addTorus(torus.getRadius(), torus.getTubeRadius(), material);
      }
      else
      {
         LogTools.error("Unsupported shape: {}", shape);
         return null;
      }

      resetCurrentTransform();
      return visual;
   }

   /**
    * Creates and adds visual for a 3D coordinate system.
    * <p>
    * Expected result for {@code addCoordinateSystem(0.25, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/CoordinateSystem.png"
    * height=250px/>
    * </p>
    * 
    * @param length the length of each arrow.
    */
   public void addCoordinateSystem(double length)
   {
      addCoordinateSystem(length, defaultMaterial);
   }

   /**
    * Creates and adds visual for a 3D coordinate system.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addCoordinateSystem(0.25, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/CoordinateSystem.png"
    * height=250px/>
    * </p>
    * 
    * @param length                the length of each arrow.
    * @param arrowHeadDiffuseColor the diffuse color to use for the head of the arrows.
    * @see ColorDefinitions
    */
   public void addCoordinateSystem(double length, ColorDefinition arrowHeadDiffuseColor)
   {
      addCoordinateSystem(length, new MaterialDefinition(arrowHeadDiffuseColor));
   }

   /**
    * Creates and adds visual for a 3D coordinate system.
    * <p>
    * Expected result for {@code addCoordinateSystem(0.25, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/CoordinateSystem.png"
    * height=250px/>
    * </p>
    * 
    * @param length            the length of each arrow.
    * @param arrowHeadMaterial the material to use for the head of the arrows.
    */
   public void addCoordinateSystem(double length, MaterialDefinition arrowHeadMaterial)
   {
      addCoordinateSystem(length,
                          new MaterialDefinition(ColorDefinitions.Red()),
                          new MaterialDefinition(ColorDefinitions.White()),
                          new MaterialDefinition(ColorDefinitions.Blue()),
                          arrowHeadMaterial);
   }

   /**
    * Creates and adds visual for a 3D coordinate system.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addCoordinateSystem(0.25, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/CoordinateSystem.png"
    * height=250px/>
    * </p>
    * 
    * @param length                the length of each arrow.
    * @param xAxisDiffuseColor     the diffuse color for the x-axis.
    * @param yAxisDiffuseColor     the diffuse color for the y-axis.
    * @param zAxisDiffuseColor     the diffuse color for the z-axis.
    * @param arrowHeadDiffuseColor the diffuse color to use for the head of the arrows.
    * @see ColorDefinitions
    */
   public void addCoordinateSystem(double length,
                                   ColorDefinition xAxisDiffuseColor,
                                   ColorDefinition yAxisDiffuseColor,
                                   ColorDefinition zAxisDiffuseColor,
                                   ColorDefinition arrowHeadDiffuseColor)
   {
      addCoordinateSystem(length,
                          new MaterialDefinition(xAxisDiffuseColor),
                          new MaterialDefinition(yAxisDiffuseColor),
                          new MaterialDefinition(zAxisDiffuseColor),
                          new MaterialDefinition(arrowHeadDiffuseColor));
   }

   /**
    * Creates and adds visual for a 3D coordinate system.
    * <p>
    * Expected result for {@code addCoordinateSystem(0.25, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/CoordinateSystem.png"
    * height=250px/>
    * </p>
    * 
    * @param length            the length of each arrow.
    * @param xAxisMaterial     the material for the x-axis.
    * @param yAxisMaterial     the material for the y-axis.
    * @param zAxisMaterial     the material for the z-axis.
    * @param arrowHeadMaterial the material to use for the head of the arrows.
    */
   public void addCoordinateSystem(double length,
                                   MaterialDefinition xAxisMaterial,
                                   MaterialDefinition yAxisMaterial,
                                   MaterialDefinition zAxisMaterial,
                                   MaterialDefinition arrowHeadMaterial)
   {
      addArrow(Axis3D.X, length, xAxisMaterial, arrowHeadMaterial);
      addArrow(Axis3D.Y, length, yAxisMaterial, arrowHeadMaterial);
      addArrow(Axis3D.Z, length, zAxisMaterial, arrowHeadMaterial);
   }

   /**
    * Creates and adds visual for an arrow.
    * <p>
    * Expected result for
    * {@code addArrow(Axis3D.Z, 0.3, ColorDefinitions.Tomato(), ColorDefinitions.BlueViolet())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Arrow.png"
    * height=250px/>
    * </p>
    * 
    * @param axis   the axis along which the arrow is to be drawn.
    * @param length the total length of the arrow
    */
   public void addArrow(Axis3D axis, double length)
   {
      addArrow(axis, length, defaultMaterial, defaultMaterial);
   }

   /**
    * Creates and adds visual for an arrow.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for
    * {@code addArrow(Axis3D.Z, 0.3, ColorDefinitions.Tomato(), ColorDefinitions.BlueViolet())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Arrow.png"
    * height=250px/>
    * </p>
    * 
    * @param axis             the axis along which the arrow is to be drawn.
    * @param length           the total length of the arrow
    * @param bodyDiffuseColor the diffuse color for the body.
    * @param headDiffuseColor the diffuse color for the head.
    * @see ColorDefinitions
    */
   public void addArrow(Axis3D axis, double length, ColorDefinition bodyDiffuseColor, ColorDefinition headDiffuseColor)
   {
      addArrow(axis, length, new MaterialDefinition(bodyDiffuseColor), new MaterialDefinition(headDiffuseColor));
   }

   /**
    * Creates and adds visual for an arrow.
    * <p>
    * Expected result for
    * {@code addArrow(Axis3D.Z, 0.3, ColorDefinitions.Tomato(), ColorDefinitions.BlueViolet())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Arrow.png"
    * height=250px/>
    * </p>
    * 
    * @param axis         the axis along which the arrow is to be drawn.
    * @param length       the total length of the arrow
    * @param bodyMaterial the material for the body.
    * @param headMaterial the material for the head.
    */
   public void addArrow(Axis3D axis, double length, MaterialDefinition bodyMaterial, MaterialDefinition headMaterial)
   {
      double coneHeight = 0.1 * length;
      double cylinderLength = length - coneHeight;
      double radius = 0.02 * length;
      double coneRadius = 2.0 * radius;

      saveCurrentTransform();

      switch (axis)
      {
         case X:
            appendRotation(Math.PI / 2.0, Axis3D.Y);
            break;
         case Y:
            appendRotation(-Math.PI / 2.0, Axis3D.X);
            break;
         case Z:
            break;
         default:
            LogTools.error("Unexpected axis value: {}", axis);
            break;
      }

      appendTranslation(0.0, 0.0, 0.5 * cylinderLength);
      addCylinder(cylinderLength, radius, bodyMaterial);
      appendTranslation(0.0, 0.0, 0.5 * cylinderLength);
      addCone(coneHeight, coneRadius, headMaterial);

      resetCurrentTransform();
   }

   /**
    * Creates and adds the visual for a 3D box.
    * <p>
    * Expected result for {@code addBox(0.1, 0.2, 0.3, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Box.png"
    * height=250px/>
    * </p>
    * 
    * @param sizeX the size along the x-axis.
    * @param sizeY the size along the y-axis.
    * @param sizeZ the size along the z-axis.
    * @return the new visual.
    */
   public VisualDefinition addBox(double sizeX, double sizeY, double sizeZ)
   {
      return addBox(sizeX, sizeY, sizeZ, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D box.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addBox(0.1, 0.2, 0.3, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Box.png"
    * height=250px/>
    * </p>
    * 
    * @param sizeX        the size along the x-axis.
    * @param sizeY        the size along the y-axis.
    * @param sizeZ        the size along the z-axis.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addBox(double sizeX, double sizeY, double sizeZ, ColorDefinition diffuseColor)
   {
      return addBox(sizeX, sizeY, sizeZ, true, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D box.
    * <p>
    * Expected result for {@code addBox(0.1, 0.2, 0.3, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Box.png"
    * height=250px/>
    * </p>
    * 
    * @param sizeX    the size along the x-axis.
    * @param sizeY    the size along the y-axis.
    * @param sizeZ    the size along the z-axis.
    * @param material the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addBox(double sizeX, double sizeY, double sizeZ, MaterialDefinition material)
   {
      return addBox(sizeX, sizeY, sizeZ, true, material);
   }

   /**
    * Creates and adds the visual for a 3D box.
    * <p>
    * Expected result for {@code addBox(0.1, 0.2, 0.3, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Box.png"
    * height=250px/>
    * </p>
    * 
    * @param sizeX               the size along the x-axis.
    * @param sizeY               the size along the y-axis.
    * @param sizeZ               the size along the z-axis.
    * @param centeredInTheCenter whether the box's origin should its center ({@code true}) or its z-
    *                            face center ({@code false}).
    * @param material            the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addBox(double sizeX, double sizeY, double sizeZ, boolean centeredInTheCenter, MaterialDefinition material)
   {
      return addGeometryDefinition(new Box3DDefinition(sizeX, sizeY, sizeZ, centeredInTheCenter), material);
   }

   /**
    * Creates and adds the visual for a 3D ramp.
    * <p>
    * Expected result for {@code addRamp(0.3, 0.2, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Ramp.png"
    * height=250px/>
    * </p>
    * 
    * @param sizeX the size along the x-axis.
    * @param sizeY the size along the y-axis.
    * @param sizeZ the size along the z-axis.
    * @return the new visual.
    */
   public VisualDefinition addRamp(double sizeX, double sizeY, double sizeZ)
   {
      return addRamp(sizeX, sizeY, sizeZ, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D ramp.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addRamp(0.3, 0.2, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Ramp.png"
    * height=250px/>
    * </p>
    * 
    * @param sizeX        the size along the x-axis.
    * @param sizeY        the size along the y-axis.
    * @param sizeZ        the size along the z-axis.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addRamp(double sizeX, double sizeY, double sizeZ, ColorDefinition diffuseColor)
   {
      return addRamp(sizeX, sizeY, sizeZ, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D ramp.
    * <p>
    * Expected result for {@code addRamp(0.3, 0.2, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Ramp.png"
    * height=250px/>
    * </p>
    * 
    * @param sizeX    the size along the x-axis.
    * @param sizeY    the size along the y-axis.
    * @param sizeZ    the size along the z-axis.
    * @param material the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addRamp(double sizeX, double sizeY, double sizeZ, MaterialDefinition material)
   {
      return addGeometryDefinition(new Ramp3DDefinition(sizeX, sizeY, sizeZ), material);
   }

   /**
    * Creates and adds the visual for a 3D sphere.
    * <p>
    * Expected result for {@code addSphere(0.15, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Sphere.png"
    * height=250px/>
    * </p>
    * 
    * @param radius the sphere radius.
    * @return the new visual.
    */
   public VisualDefinition addSphere(double radius)
   {
      return addSphere(radius, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D sphere.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addSphere(0.15, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Sphere.png"
    * height=250px/>
    * </p>
    * 
    * @param radius       the sphere radius.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addSphere(double radius, ColorDefinition diffuseColor)
   {
      return addSphere(radius, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D sphere.
    * <p>
    * Expected result for {@code addSphere(0.15, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Sphere.png"
    * height=250px/>
    * </p>
    * 
    * @param radius   the sphere radius.
    * @param material the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addSphere(double radius, MaterialDefinition material)
   {
      return addGeometryDefinition(new Sphere3DDefinition(radius), material);
   }

   /**
    * Creates and adds the visual for a 3D capsule.
    * <p>
    * Expected result for {@code addCapsule(0.2, 0.05, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Capsule.png"
    * height=250px/>
    * </p>
    * 
    * @param length the length of the cylindrical part of the capsule.
    * @param radius the capsule radius.
    * @return the new visual.
    */
   public VisualDefinition addCapsule(double length, double radius)
   {
      return addCapsule(length, radius, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D capsule.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addCapsule(0.2, 0.05, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Capsule.png"
    * height=250px/>
    * </p>
    * 
    * @param length       the length of the cylindrical part of the capsule.
    * @param radius       the capsule radius.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addCapsule(double length, double radius, ColorDefinition diffuseColor)
   {
      return addCapsule(length, radius, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D capsule.
    * <p>
    * Expected result for {@code addCapsule(0.2, 0.05, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Capsule.png"
    * height=250px/>
    * </p>
    * 
    * @param length   the length of the cylindrical part of the capsule.
    * @param radius   the capsule radius.
    * @param material the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addCapsule(double length, double radius, MaterialDefinition material)
   {
      return addGeometryDefinition(new Capsule3DDefinition(length, radius), material);
   }

   /**
    * Creates and adds the visual for a 3D ellipsoid.
    * <p>
    * Expected result for {@code addEllipsoid(0.025, 0.2, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Ellipsoid.png"
    * height=250px/>
    * </p>
    * 
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @return the new visual.
    */
   public VisualDefinition addEllipsoid(double radiusX, double radiusY, double radiusZ)
   {
      return addEllipsoid(radiusX, radiusY, radiusZ, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D ellipsoid.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addEllipsoid(0.025, 0.2, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Ellipsoid.png"
    * height=250px/>
    * </p>
    * 
    * @param radiusX      radius of the ellipsoid along the x-axis.
    * @param radiusY      radius of the ellipsoid along the y-axis.
    * @param radiusZ      radius of the ellipsoid along the z-axis.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addEllipsoid(double radiusX, double radiusY, double radiusZ, ColorDefinition diffuseColor)
   {
      return addEllipsoid(radiusX, radiusY, radiusZ, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D ellipsoid.
    * <p>
    * Expected result for {@code addEllipsoid(0.025, 0.2, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Ellipsoid.png"
    * height=250px/>
    * </p>
    * 
    * @param radiusX  radius of the ellipsoid along the x-axis.
    * @param radiusY  radius of the ellipsoid along the y-axis.
    * @param radiusZ  radius of the ellipsoid along the z-axis.
    * @param material the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addEllipsoid(double radiusX, double radiusY, double radiusZ, MaterialDefinition material)
   {
      return addGeometryDefinition(new Ellipsoid3DDefinition(radiusX, radiusY, radiusZ), material);
   }

   /**
    * Creates and adds the visual for a 3D cylinder.
    * <p>
    * Expected result for {@code addCylinder(0.2, 0.05, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Cylinder.png"
    * height=250px/>
    * </p>
    * 
    * @param length the cylinder length.
    * @param radius the cylinder radius.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addCylinder(double length, double radius)
   {
      return addCylinder(length, radius, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D cylinder.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addCylinder(0.2, 0.05, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Cylinder.png"
    * height=250px/>
    * </p>
    * 
    * @param length       the cylinder length.
    * @param radius       the cylinder radius.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addCylinder(double length, double radius, ColorDefinition diffuseColor)
   {
      return addCylinder(length, radius, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D cylinder.
    * <p>
    * Expected result for {@code addCylinder(0.2, 0.05, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Cylinder.png"
    * height=250px/>
    * </p>
    * 
    * @param length   the cylinder length.
    * @param radius   the cylinder radius.
    * @param material the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addCylinder(double length, double radius, MaterialDefinition material)
   {
      return addGeometryDefinition(new Cylinder3DDefinition(length, radius, false), material);
   }

   /**
    * Creates and adds the visual for a 3D cone.
    * <p>
    * Expected result for {@code addCone(0.2, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Cone.png"
    * height=250px/>
    * </p>
    * 
    * @param height the cone length.
    * @param radius the cone radius.
    * @return the new visual.
    */
   public VisualDefinition addCone(double height, double radius)
   {
      return addCone(height, radius, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D cone.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addCone(0.2, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Cone.png"
    * height=250px/>
    * </p>
    * 
    * @param height       the cone length.
    * @param radius       the cone radius.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addCone(double height, double radius, ColorDefinition diffuseColor)
   {
      return addCone(height, radius, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D cone.
    * <p>
    * Expected result for {@code addCone(0.2, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Cone.png"
    * height=250px/>
    * </p>
    * 
    * @param height   the cone length.
    * @param radius   the cone radius.
    * @param material the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addCone(double height, double radius, MaterialDefinition material)
   {
      return addGeometryDefinition(new Cone3DDefinition(height, radius), material);
   }

   /**
    * Creates and adds the visual for a 3D truncated cone.
    * <p>
    * Expected result for {@code addTruncatedCone(0.2, 0.1, 0.04, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/TruncatedCone.png"
    * height=250px/>
    * </p>
    * 
    * @param height     the height of the truncated cone.
    * @param topRadius  the radius of the top face.
    * @param baseRadius the radius of the bottom face.
    * @return the new visual.
    */
   public VisualDefinition addTruncatedCone(double height, double baseRadius, double topRadius)
   {
      return addTruncatedCone(height, baseRadius, topRadius, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D truncated cone.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addTruncatedCone(0.2, 0.1, 0.04, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/TruncatedCone.png"
    * height=250px/>
    * </p>
    * 
    * @param height       the height of the truncated cone.
    * @param topRadius    the radius of the top face.
    * @param baseRadius   the radius of the bottom face.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addTruncatedCone(double height, double baseRadius, double topRadius, ColorDefinition diffuseColor)
   {
      return addTruncatedCone(height, baseRadius, topRadius, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D truncated cone.
    * <p>
    * Expected result for {@code addTruncatedCone(0.2, 0.1, 0.04, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/TruncatedCone.png"
    * height=250px/>
    * </p>
    * 
    * @param height     the height of the truncated cone.
    * @param topRadius  the radius of the top face.
    * @param baseRadius the radius of the bottom face.
    * @param material   the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addTruncatedCone(double height, double baseRadius, double topRadius, MaterialDefinition material)
   {
      return addTruncatedCone(height, baseRadius, baseRadius, topRadius, topRadius, material);
   }

   /**
    * Creates and adds the visual for a 3D truncated cone.
    * <p>
    * Expected result for
    * {@code addTruncatedCone(0.175, 0.12, 0.075, 0.03, 0.06, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/GenericTruncatedCone.png"
    * height=250px/>
    * </p>
    * 
    * @param height      the height of the truncated cone.
    * @param topRadiusX  the radius of the top face along the x-axis.
    * @param topRadiusY  the radius of the top face along the y-axis.
    * @param baseRadiusX the radius of the bottom face along the x-axis.
    * @param baseRadiusY the radius of the bottom face along the y-axis.
    * @return the new visual.
    */
   public VisualDefinition addTruncatedCone(double height, double baseRadiusX, double baseRadiusY, double topRadiusX, double topRadiusY)
   {
      return addTruncatedCone(height, baseRadiusX, baseRadiusY, topRadiusX, topRadiusY, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D truncated cone.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for
    * {@code addTruncatedCone(0.175, 0.12, 0.075, 0.03, 0.06, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/GenericTruncatedCone.png"
    * height=250px/>
    * </p>
    * 
    * @param height       the height of the truncated cone.
    * @param topRadiusX   the radius of the top face along the x-axis.
    * @param topRadiusY   the radius of the top face along the y-axis.
    * @param baseRadiusX  the radius of the bottom face along the x-axis.
    * @param baseRadiusY  the radius of the bottom face along the y-axis.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addTruncatedCone(double height,
                                            double baseRadiusX,
                                            double baseRadiusY,
                                            double topRadiusX,
                                            double topRadiusY,
                                            ColorDefinition diffuseColor)
   {
      return addTruncatedCone(height, baseRadiusX, baseRadiusY, topRadiusX, topRadiusY, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D truncated cone.
    * <p>
    * Expected result for
    * {@code addTruncatedCone(0.175, 0.12, 0.075, 0.03, 0.06, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/GenericTruncatedCone.png"
    * height=250px/>
    * </p>
    * 
    * @param height      the height of the truncated cone.
    * @param topRadiusX  the radius of the top face along the x-axis.
    * @param topRadiusY  the radius of the top face along the y-axis.
    * @param baseRadiusX the radius of the bottom face along the x-axis.
    * @param baseRadiusY the radius of the bottom face along the y-axis.
    * @param material    the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addTruncatedCone(double height,
                                            double baseRadiusX,
                                            double baseRadiusY,
                                            double topRadiusX,
                                            double topRadiusY,
                                            MaterialDefinition material)
   {
      return addGeometryDefinition(new TruncatedCone3DDefinition(height, topRadiusX, topRadiusY, baseRadiusX, baseRadiusY), material);
   }

   /**
    * Creates and adds the visual for a 3D hemi-ellipsoid.
    * <p>
    * Expected result for {@code addHemiEllipsoid(0.15, 0.05, 0.225, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/HemiEllipsoid.png"
    * height=250px/>
    * </p>
    * 
    * @param radiusX radius of the hemi-ellipsoid along the x-axis.
    * @param radiusY radius of the hemi-ellipsoid along the y-axis.
    * @param radiusZ radius of the hemi-ellipsoid along the z-axis.
    * @return the new visual.
    */
   public VisualDefinition addHemiEllipsoid(double radiusX, double radiusY, double radiusZ)
   {
      return addHemiEllipsoid(radiusX, radiusY, radiusZ, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D hemi-ellipsoid.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addHemiEllipsoid(0.15, 0.05, 0.225, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/HemiEllipsoid.png"
    * height=250px/>
    * </p>
    * 
    * @param radiusX      radius of the hemi-ellipsoid along the x-axis.
    * @param radiusY      radius of the hemi-ellipsoid along the y-axis.
    * @param radiusZ      radius of the hemi-ellipsoid along the z-axis.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addHemiEllipsoid(double radiusX, double radiusY, double radiusZ, ColorDefinition diffuseColor)
   {
      return addHemiEllipsoid(radiusX, radiusY, radiusZ, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D hemi-ellipsoid.
    * <p>
    * Expected result for {@code addHemiEllipsoid(0.15, 0.05, 0.225, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/HemiEllipsoid.png"
    * height=250px/>
    * </p>
    * 
    * @param radiusX  radius of the hemi-ellipsoid along the x-axis.
    * @param radiusY  radius of the hemi-ellipsoid along the y-axis.
    * @param radiusZ  radius of the hemi-ellipsoid along the z-axis.
    * @param material the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addHemiEllipsoid(double radiusX, double radiusY, double radiusZ, MaterialDefinition material)
   {
      return addGeometryDefinition(new HemiEllipsoid3DDefinition(radiusX, radiusY, radiusZ), material);
   }

   /**
    * Creates and adds the visual for a 3D torus.
    * <p>
    * Expected result for {@code addTorus(0.2, 0.025, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Torus.png"
    * height=250px/>
    * </p>
    * 
    * @param majorRadius the radius from the torus centroid to the tube center.
    * @param minorRadius the radius of the tube.
    * @return the new visual.
    */
   public VisualDefinition addTorus(double majorRadius, double minorRadius)
   {
      return addTorus(majorRadius, minorRadius, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D torus.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addTorus(0.2, 0.025, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Torus.png"
    * height=250px/>
    * </p>
    * 
    * @param majorRadius  the radius from the torus centroid to the tube center.
    * @param minorRadius  the radius of the tube.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addTorus(double majorRadius, double minorRadius, ColorDefinition diffuseColor)
   {
      return addTorus(majorRadius, minorRadius, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D torus.
    * <p>
    * Expected result for {@code addTorus(0.2, 0.025, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Torus.png"
    * height=250px/>
    * </p>
    * 
    * @param majorRadius the radius from the torus centroid to the tube center.
    * @param minorRadius the radius of the tube.
    * @param material    the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addTorus(double majorRadius, double minorRadius, MaterialDefinition material)
   {
      return addGeometryDefinition(new Torus3DDefinition(majorRadius, minorRadius), material);
   }

   /**
    * Creates and adds the visual for a 3D arc-torus.
    * <p>
    * Expected result for
    * {@code addArcTorus(0.25 * Math.PI, 1.75 * Math.PI, 0.2, 0.025, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/ArcTorus.png"
    * height=250px/>
    * </p>
    * 
    * @param startAngle  the angle at which the torus starts. The angle is in radians, it is expressed
    *                    with respect to the x-axis, and a positive angle corresponds to a
    *                    counter-clockwise rotation.
    * @param endAngle    the angle at which the torus ends. If {@code startAngle == endAngle} the torus
    *                    will be closed. The angle is in radians, it is expressed with respect to the
    *                    x-axis, and a positive angle corresponds to a counter-clockwise rotation.
    * @param majorRadius the radius from the torus centroid to the tube center.
    * @param minorRadius the radius of the tube.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius)
   {
      return addArcTorus(startAngle, endAngle, majorRadius, minorRadius, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D arc-torus.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for
    * {@code addArcTorus(0.25 * Math.PI, 1.75 * Math.PI, 0.2, 0.025, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/ArcTorus.png"
    * height=250px/>
    * </p>
    * 
    * @param startAngle   the angle at which the torus starts. The angle is in radians, it is expressed
    *                     with respect to the x-axis, and a positive angle corresponds to a
    *                     counter-clockwise rotation.
    * @param endAngle     the angle at which the torus ends. If {@code startAngle == endAngle} the
    *                     torus will be closed. The angle is in radians, it is expressed with respect
    *                     to the x-axis, and a positive angle corresponds to a counter-clockwise
    *                     rotation.
    * @param majorRadius  the radius from the torus centroid to the tube center.
    * @param minorRadius  the radius of the tube.
    * @param diffuseColor the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, ColorDefinition diffuseColor)
   {
      return addArcTorus(startAngle, endAngle, majorRadius, minorRadius, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D arc-torus.
    * <p>
    * Expected result for
    * {@code addArcTorus(0.25 * Math.PI, 1.75 * Math.PI, 0.2, 0.025, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/ArcTorus.png"
    * height=250px/>
    * </p>
    * 
    * @param startAngle  the angle at which the torus starts. The angle is in radians, it is expressed
    *                    with respect to the x-axis, and a positive angle corresponds to a
    *                    counter-clockwise rotation.
    * @param endAngle    the angle at which the torus ends. If {@code startAngle == endAngle} the torus
    *                    will be closed. The angle is in radians, it is expressed with respect to the
    *                    x-axis, and a positive angle corresponds to a counter-clockwise rotation.
    * @param majorRadius the radius from the torus centroid to the tube center.
    * @param minorRadius the radius of the tube.
    * @param material    the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, MaterialDefinition material)
   {
      return addGeometryDefinition(new ArcTorus3DDefinition(startAngle, endAngle, majorRadius, minorRadius), material);
   }

   /**
    * Creates and adds the visual for a 3D pyramid-box.
    * <p>
    * Expected result for {@code addPyramidBox(0.15, 0.075, 0.15, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/PyramidBox.png"
    * height=250px/>
    * </p>
    * 
    * @param boxSizeX      the size of the box along the x-axis.
    * @param boxSizeY      the size of the box along the y-axis.
    * @param boxSizeZ      the size of the box along the z-axis.
    * @param pyramidHeight the height for each pyramid.
    * @return the new visual.
    */
   public VisualDefinition addPyramidBox(double boxSizeX, double boxSizeY, double boxSizeZ, double pyramidHeight)
   {
      return addPyramidBox(boxSizeX, boxSizeY, boxSizeZ, pyramidHeight, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a 3D pyramid-box.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for {@code addPyramidBox(0.15, 0.075, 0.15, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/PyramidBox.png"
    * height=250px/>
    * </p>
    * 
    * @param boxSizeX      the size of the box along the x-axis.
    * @param boxSizeY      the size of the box along the y-axis.
    * @param boxSizeZ      the size of the box along the z-axis.
    * @param pyramidHeight the height for each pyramid.
    * @param diffuseColor  the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addPyramidBox(double boxSizeX, double boxSizeY, double boxSizeZ, double pyramidHeight, ColorDefinition diffuseColor)
   {
      return addPyramidBox(boxSizeX, boxSizeY, boxSizeZ, pyramidHeight, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a 3D pyramid-box.
    * <p>
    * Expected result for {@code addPyramidBox(0.15, 0.075, 0.15, 0.1, ColorDefinitions.Cyan())}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/PyramidBox.png"
    * height=250px/>
    * </p>
    * 
    * @param boxSizeX      the size of the box along the x-axis.
    * @param boxSizeY      the size of the box along the y-axis.
    * @param boxSizeZ      the size of the box along the z-axis.
    * @param pyramidHeight the height for each pyramid.
    * @param material      the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addPyramidBox(double boxSizeX, double boxSizeY, double boxSizeZ, double pyramidHeight, MaterialDefinition material)
   {
      return addGeometryDefinition(new PyramidBox3DDefinition(boxSizeX, boxSizeY, boxSizeZ, pyramidHeight), material);
   }

   /**
    * Creates and adds the visual for a polygon 2D.
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param convexPolygon2D the convex polygon to create a visual for.
    * @return the new visual.
    */
   public VisualDefinition addPolygon2D(ConvexPolygon2DReadOnly convexPolygon2D)
   {
      return addPolygon2D(convexPolygon2D, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a polygon 2D.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param convexPolygon2D the convex polygon to create a visual for.
    * @param diffuseColor    the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addPolygon2D(ConvexPolygon2DReadOnly convexPolygon2D, ColorDefinition diffuseColor)
   {
      return addPolygon2D(convexPolygon2D, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a polygon 2D.
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param convexPolygon2D the convex polygon to create a visual for.
    * @param material        the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addPolygon2D(ConvexPolygon2DReadOnly convexPolygon2D, MaterialDefinition material)
   {
      return addPolygon2D(convexPolygon2D.getPolygonVerticesView(), !convexPolygon2D.isClockwiseOrdered(), material);
   }

   /**
    * Creates and adds the visual for a polygon 2D.
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @return the new visual.
    */
   public VisualDefinition addPolygon2D(List<? extends Point2DReadOnly> polygonPoints, boolean counterClockwiseOrdered)
   {
      return addPolygon2D(polygonPoints, counterClockwiseOrdered, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a polygon 2D.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param diffuseColor            the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addPolygon2D(List<? extends Point2DReadOnly> polygonPoints, boolean counterClockwiseOrdered, ColorDefinition diffuseColor)
   {
      return addPolygon2D(polygonPoints, counterClockwiseOrdered, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a polygon 2D.
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param material                the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addPolygon2D(List<? extends Point2DReadOnly> polygonPoints, boolean counterClockwiseOrdered, MaterialDefinition material)
   {
      return addGeometryDefinition(new Polygon2DDefinition(Polygon2DDefinition.toPoint2DDefinitionList(polygonPoints), counterClockwiseOrdered), material);
   }

   /**
    * Creates and adds the visual for a polygon 2D.
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param material                the material of the new visual.
    * @param polygonPoints           the polygon's vertices.
    * @return the new visual.
    */
   public VisualDefinition addPolygon2D(MaterialDefinition material, boolean counterClockwiseOrdered, Point2DReadOnly... polygonPoints)
   {
      return addPolygon2D(Arrays.asList(polygonPoints), counterClockwiseOrdered, material);
   }

   /**
    * Creates and adds the visual for a polygon 2D.
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @return the new visual.
    */
   public VisualDefinition addPolygon2D(Point2DReadOnly[] polygonPoints, boolean counterClockwiseOrdered)
   {
      return addPolygon2D(Arrays.asList(polygonPoints), counterClockwiseOrdered, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a polygon 2D.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param diffuseColor            the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addPolygon2D(Point2DReadOnly[] polygonPoints, boolean counterClockwiseOrdered, ColorDefinition diffuseColor)
   {
      return addPolygon2D(Arrays.asList(polygonPoints), counterClockwiseOrdered, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a polygon 2D.
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param material                the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addPolygon2D(Point2DReadOnly[] polygonPoints, boolean counterClockwiseOrdered, MaterialDefinition material)
   {
      return addPolygon2D(Arrays.asList(polygonPoints), counterClockwiseOrdered, material);
   }

   /**
    * Creates and adds the visual for a list of 2D polygons.
    * 
    * @param convexPolygon2Ds the list of polygons to create the visuals for.
    * @see #addPolygon2D(ConvexPolygon2DReadOnly)
    */
   public void addPolygon2Ds(List<? extends ConvexPolygon2DReadOnly> convexPolygon2Ds)
   {
      addPolygon2Ds(convexPolygon2Ds, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a list of 2D polygons.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * 
    * @param convexPolygon2Ds the list of polygons to create the visuals for.
    * @param diffuseColor     the diffuse color of the new visual.
    * @see #addPolygon2D(ConvexPolygon2DReadOnly, ColorDefinition)
    * @see ColorDefinitions
    */
   public void addPolygon2Ds(List<? extends ConvexPolygon2DReadOnly> convexPolygon2Ds, ColorDefinition diffuseColor)
   {
      addPolygon2Ds(convexPolygon2Ds, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a list of 2D polygons.
    * 
    * @param convexPolygon2Ds the list of polygons to create the visuals for.
    * @param material         the material of the new visuals.
    * @see #addPolygon2D(ConvexPolygon2DReadOnly, MaterialDefinition)
    */
   public void addPolygon2Ds(List<? extends ConvexPolygon2DReadOnly> convexPolygon2Ds, MaterialDefinition material)
   {
      for (int i = 0; i < convexPolygon2Ds.size(); i++)
      {
         addPolygon2D(convexPolygon2Ds.get(i), material);
      }
   }

   /**
    * Creates and adds the visual for a polygon 3D.
    * <p>
    * Expected result for a polygon 3D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon3D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @return the new visual.
    */
   public VisualDefinition addPolygon3D(List<? extends Point3DReadOnly> polygonPoints, boolean counterClockwiseOrdered)
   {
      return addPolygon3D(polygonPoints, counterClockwiseOrdered, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a polygon 3D.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for a polygon 3D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon3D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param diffuseColor            the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addPolygon3D(List<? extends Point3DReadOnly> polygonPoints, boolean counterClockwiseOrdered, ColorDefinition diffuseColor)
   {
      return addPolygon3D(polygonPoints, counterClockwiseOrdered, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a polygon 3D.
    * <p>
    * Expected result for a polygon 3D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon3D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param material                the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addPolygon3D(List<? extends Point3DReadOnly> polygonPoints, boolean counterClockwiseOrdered, MaterialDefinition material)
   {
      return addGeometryDefinition(new Polygon3DDefinition(Polygon3DDefinition.toPoint3DDefinitionList(polygonPoints), counterClockwiseOrdered), material);
   }

   /**
    * Creates and adds the visual for a polygon 3D.
    * <p>
    * Expected result for a polygon 3D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon3D.png"
    * height=250px/>
    * </p>
    * 
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param material                the material of the new visual.
    * @param polygonPoints           the polygon's vertices.
    * @return the new visual.
    */
   public VisualDefinition addPolygon3D(MaterialDefinition material, boolean counterClockwiseOrdered, Point3DReadOnly... polygonPoints)
   {
      return addPolygon3D(Arrays.asList(polygonPoints), counterClockwiseOrdered, material);
   }

   /**
    * Creates and adds the visual for a polygon 3D.
    * <p>
    * Expected result for a polygon 3D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon3D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @return the new visual.
    */
   public VisualDefinition addPolygon3D(Point3DReadOnly[] polygonPoints, boolean counterClockwiseOrdered)
   {
      return addPolygon3D(Arrays.asList(polygonPoints), counterClockwiseOrdered, defaultMaterial);
   }

   /**
    * Creates and adds the visual for a polygon 3D.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for a polygon 3D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon3D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param diffuseColor            the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addPolygon3D(Point3DReadOnly[] polygonPoints, boolean counterClockwiseOrdered, ColorDefinition diffuseColor)
   {
      return addPolygon3D(Arrays.asList(polygonPoints), counterClockwiseOrdered, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for a polygon 3D.
    * <p>
    * Expected result for a polygon 3D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon3D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon's vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param material                the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addPolygon3D(Point3DReadOnly[] polygonPoints, boolean counterClockwiseOrdered, MaterialDefinition material)
   {
      return addPolygon3D(Arrays.asList(polygonPoints), counterClockwiseOrdered, material);
   }

   /**
    * Creates and adds the visual for an extruded polygon.
    * <p>
    * Expected result for an extruded polygon:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/ExtrudedPolygon.png"
    * height=250px/>
    * </p>
    * 
    * @param convexPolygon2D the polygon to create the visual for.
    * @param extrusionHeight the thickness of the extrusion.
    * @return the new visual.
    */
   public VisualDefinition addExtrudedPolygon(ConvexPolygon2DReadOnly convexPolygon2D, double extrusionHeight)
   {
      return addExtrudedPolygon(convexPolygon2D, extrusionHeight, defaultMaterial);
   }

   /**
    * Creates and adds the visual for an extruded polygon.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for an extruded polygon:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/ExtrudedPolygon.png"
    * height=250px/>
    * </p>
    * 
    * @param convexPolygon2D the polygon to create the visual for.
    * @param extrusionHeight the thickness of the extrusion.
    * @param diffuseColor    the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addExtrudedPolygon(ConvexPolygon2DReadOnly convexPolygon2D, double extrusionHeight, ColorDefinition diffuseColor)
   {
      return addExtrudedPolygon(convexPolygon2D, extrusionHeight, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for an extruded polygon.
    * <p>
    * Expected result for an extruded polygon:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/ExtrudedPolygon.png"
    * height=250px/>
    * </p>
    * 
    * @param convexPolygon2D the polygon to create the visual for.
    * @param extrusionHeight the thickness of the extrusion.
    * @param material        the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addExtrudedPolygon(ConvexPolygon2DReadOnly convexPolygon2D, double extrusionHeight, MaterialDefinition material)
   {
      return addExtrudedPolygon(convexPolygon2D.getPolygonVerticesView(), !convexPolygon2D.isClockwiseOrdered(), extrusionHeight, material);
   }

   /**
    * Creates and adds the visual for an extruded polygon.
    * <p>
    * Expected result for an extruded polygon:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/ExtrudedPolygon.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param extrusionHeight         the thickness of the extrusion.
    * @return the new visual.
    */
   public VisualDefinition addExtrudedPolygon(List<? extends Point2DReadOnly> polygonPoints, boolean counterClockwiseOrdered, double extrusionHeight)
   {
      return addExtrudedPolygon(polygonPoints, counterClockwiseOrdered, extrusionHeight, defaultMaterial);
   }

   /**
    * Creates and adds the visual for an extruded polygon.
    * <p>
    * See {@link ColorDefinitions} for generic colors and color parsers.
    * </p>
    * <p>
    * Expected result for an extruded polygon:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/ExtrudedPolygon.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param extrusionHeight         the thickness of the extrusion.
    * @param diffuseColor            the diffuse color of the new visual.
    * @return the new visual.
    * @see ColorDefinitions
    */
   public VisualDefinition addExtrudedPolygon(List<? extends Point2DReadOnly> polygonPoints,
                                              boolean counterClockwiseOrdered,
                                              double extrusionHeight,
                                              ColorDefinition diffuseColor)
   {
      return addExtrudedPolygon(polygonPoints, counterClockwiseOrdered, extrusionHeight, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates and adds the visual for an extruded polygon.
    * <p>
    * Expected result for an extruded polygon:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/ExtrudedPolygon.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPoints           the polygon vertices.
    * @param counterClockwiseOrdered the winding of the vertices.
    * @param extrusionHeight         the thickness of the extrusion.
    * @param material                the material of the new visual.
    * @return the new visual.
    */
   public VisualDefinition addExtrudedPolygon(List<? extends Point2DReadOnly> polygonPoints,
                                              boolean counterClockwiseOrdered,
                                              double extrusionHeight,
                                              MaterialDefinition material)
   {
      return addGeometryDefinition(new ExtrudedPolygon2DDefinition(ExtrudedPolygon2DDefinition.toPoint2DDefinitionList(polygonPoints),
                                                                   counterClockwiseOrdered,
                                                                   extrusionHeight),
                                   material);
   }
}
