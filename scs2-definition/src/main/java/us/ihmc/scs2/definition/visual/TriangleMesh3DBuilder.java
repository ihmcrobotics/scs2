package us.ihmc.scs2.definition.visual;

import java.util.List;
import java.util.stream.Collectors;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.Tetrahedron3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;

/**
 * This class provides a simple way of combining several shapes/meshes into a single mesh. It can be
 * used as a one-time tool or can be recycled by using the method {@link #clear()} every time a new
 * mesh is to be created.
 * 
 * @author Sylvain Bertrand
 */
public class TriangleMesh3DBuilder
{
   /**
    * Internal data structure representing the mesh being built.
    */
   private final MutableTriangleMesh3DData triangleMesh = new MutableTriangleMesh3DData();

   /**
    * Creates an empty builder.
    */
   public TriangleMesh3DBuilder()
   {
      clear();
   }

   /**
    * Combines the triangle mesh contained in {@code other} with the triangle mesh contained in this
    * builder.
    * 
    * @param other the other builder holding on the triangle mesh to combine. Not Modified.
    */
   public void addTriangleMesh3D(TriangleMesh3DBuilder other)
   {
      triangleMesh.add(other.triangleMesh, true);
   }

   /**
    * Translates then combines the triangle mesh contained in {@code other} with the triangle mesh
    * contained in this builder.
    * 
    * @param other  the other builder holding on the triangle mesh to combine. Not Modified.
    * @param offset used to translate the other's triangle mesh. Can be {@code null}. Not modified.
    */
   public void addTriangleMesh3D(TriangleMesh3DBuilder other, Tuple3DReadOnly offset)
   {
      this.triangleMesh.add(other.triangleMesh, offset, null, true);
   }

   /**
    * Rotates, translates, then combines the triangle mesh contained in {@code other} with the triangle
    * mesh contained in this builder.
    * 
    * @param other       the other builder holding on the triangle mesh to combine. Not Modified.
    * @param offset      used to translate the other's triangle mesh. Can be {@code null}. Not
    *                    modified.
    * @param orientation used to rotate the other's triangle mesh. Can be {@code null}. Not modified.
    */
   public void addTriangleMesh3D(TriangleMesh3DBuilder other, Tuple3DReadOnly offset, Orientation3DReadOnly orientation)
   {
      this.triangleMesh.add(other.triangleMesh, offset, orientation, true);
   }

   /**
    * Combines the given mesh with the triangle mesh contained in this builder.
    * 
    * @param triangleMesh the triangle mesh to combine. Not Modified.
    */
   public void addTriangleMesh3D(TriangleMesh3DDefinition triangleMesh)
   {
      this.triangleMesh.add(triangleMesh, true);
   }

   /**
    * Translates then combines the given triangle mesh with the triangle mesh contained in this
    * builder.
    * 
    * @param triangleMesh the triangle mesh to combine. Not Modified.
    * @param offset       used to translate the given triangle mesh. Can be {@code null}. Not modified.
    */
   public void addTriangleMesh3D(TriangleMesh3DDefinition triangleMesh, Tuple3DReadOnly offset)
   {
      this.triangleMesh.add(triangleMesh, offset, null, true);
   }

   /**
    * Rotates, translates, then combines the given triangle mesh with the triangle mesh contained in
    * this builder.
    * 
    * @param triangleMesh the triangle mesh to combine. Not Modified.
    * @param offset       used to translate the given triangle mesh. Can be {@code null}. Not modified.
    * @param orientation  used to rotate the given triangle mesh. Can be {@code null}. Not modified.
    */
   public void addTriangleMesh3D(TriangleMesh3DDefinition triangleMesh, Tuple3DReadOnly offset, Orientation3DReadOnly orientation)
   {
      this.triangleMesh.add(triangleMesh, offset, orientation, true);
   }

   /**
    * Rotates, translates, then combines the given geometry to the triangle mesh contained in this
    * builder.
    * 
    * @param geometry    the geometry to combine. Not Modified.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    */
   public void addGeometry(GeometryDefinition geometry, Tuple3DReadOnly offset, Orientation3DReadOnly orientation)
   {
      this.triangleMesh.add(TriangleMesh3DFactories.TriangleMesh(geometry), offset, orientation, true);
   }

   /**
    * Add a box centered at (0, 0, 0) to this builder.
    * <p>
    * Expected result for {@code addBox(0.1, 0.2, 0.3)}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Box.png"
    * height=250px/>
    * </p>
    * 
    * @param sizeX box size along the x-axis.
    * @param sizeY box size along the y-axis.
    * @param sizeZ box size along the z-axis.
    */
   public void addBox(double sizeX, double sizeY, double sizeZ)
   {
      addBox(sizeX, sizeY, sizeZ, null, null);
   }

   /**
    * Add a box to this builder.
    * 
    * @param sizeX  box size along the x-axis.
    * @param sizeY  box size along the y-axis.
    * @param sizeZ  box size along the z-axis.
    * @param offset coordinate of the geometry. Can be {@code null}. Not modified.
    */
   public void addBox(double sizeX, double sizeY, double sizeZ, Tuple3DReadOnly offset)
   {
      addBox(sizeX, sizeY, sizeZ, offset, null);
   }

   /**
    * Add a box to this builder.
    * 
    * @param sizeX       box size along the x-axis.
    * @param sizeY       box size along the y-axis.
    * @param sizeZ       box size along the z-axis.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    */
   public void addBox(double sizeX, double sizeY, double sizeZ, Tuple3DReadOnly offset, Orientation3DReadOnly orientation)
   {
      addGeometry(new Box3DDefinition(sizeX, sizeY, sizeZ), offset, orientation);
   }

   /**
    * Add a capsule centered at (0, 0, 0) to this builder.
    * <p>
    * Expected result for {@code addCapsule(0.2, 0.05)}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Capsule.png"
    * height=250px/>
    * </p>
    * 
    * @param length the capsule's length or height. Distance separating the center of the two half
    *               spheres.
    * @param radius the capsule's radius.
    */
   public void addCapsule(double length, double radius)
   {
      addCapsule(length, radius, null, null);
   }

   /**
    * Add a capsule to this builder.
    * 
    * @param length the capsule's length or height. Distance separating the center of the two half
    *               spheres.
    * @param radius the capsule's radius.
    * @param offset coordinate of the geometry. Can be {@code null}. Not modified.
    */
   public void addCapsule(double length, double radius, Tuple3DReadOnly offset)
   {
      addCapsule(length, radius, offset, null);
   }

   /**
    * Add a capsule to this builder.
    * 
    * @param length      the capsule's length or height. Distance separating the center of the two half
    *                    spheres.
    * @param radius      the capsule's radius.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    */
   public void addCapsule(double length, double radius, Tuple3DReadOnly offset, Orientation3DReadOnly orientation)
   {
      addGeometry(new Capsule3DDefinition(length, radius), offset, orientation);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with
    * the highest z value.
    * <p>
    * Expected result for {@code addCone(0.2, 0.1)}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Cone.png"
    * height=250px/>
    * </p>
    * 
    * @param height height along z of the cone.
    * @param radius radius of the cone's base.
    */
   public void addCone(double height, double radius)
   {
      addCone(height, radius, null, null);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with
    * the highest z value.
    * 
    * @param height height along z of the cone.
    * @param radius radius of the cone's base.
    * @param offset coordinate of the geometry. Can be {@code null}. Not modified.
    */
   public void addCone(double height, double radius, Tuple3DReadOnly offset)
   {
      addCone(height, radius, offset, null);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with
    * the highest z value in its local coordinate system.
    * 
    * @param height      height along z of the cone.
    * @param radius      radius of the cone's base.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    */
   public void addCone(double height, double radius, Tuple3DReadOnly offset, Orientation3DReadOnly orientation)
   {
      addGeometry(new Cone3DDefinition(height, radius), offset, orientation);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis.
    * <p>
    * Expected result for {@code addCylinder(0.2, 0.05)}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Cylinder.png"
    * height=250px/>
    * </p>
    * 
    * @param height height along z of the cylinder.
    * @param radius the cylinder's radius.
    */
   public void addCylinder(double length, double radius)
   {
      addCylinder(length, radius, null, null);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis.
    * 
    * @param height      height along z of the cylinder.
    * @param radius      the cylinder's radius.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    */
   public void addCylinder(double length, double radius, Tuple3DReadOnly offset)
   {
      addCylinder(length, radius, offset, null);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis in its local coordinate
    * system.
    * 
    * @param height      height along z of the cylinder.
    * @param radius      the cylinder's radius.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    */
   public void addCylinder(double length, double radius, Tuple3DReadOnly offset, Orientation3DReadOnly orientation)
   {
      addGeometry(new Cylinder3DDefinition(radius, length), offset, orientation);
   }

   /**
    * Add a 3D line to this builder.
    * 
    * @param x0        x-coordinate of the line start.
    * @param y0        y-coordinate of the line start.
    * @param z0        z-coordinate of the line start.
    * @param xf        x-coordinate of the line end.
    * @param yf        y-coordinate of the line end.
    * @param zf        z-coordinate of the line end.
    * @param lineWidth width of the line.
    */
   public void addLine(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth)
   {
      addTriangleMesh3D(TriangleMesh3DFactories.Line(x0, y0, z0, xf, yf, zf, lineWidth));
   }

   /**
    * Add a 3D line to this builder.
    * 
    * @param start     start coordinate of the line. Not modified.
    * @param end       end coordinate of the line. Not modified.
    * @param lineWidth width of the line.
    */
   public void addLine(Tuple3DReadOnly start, Tuple3DReadOnly end, double lineWidth)
   {
      addLine(start.getX(), start.getY(), start.getZ(), end.getX(), end.getY(), end.getZ(), lineWidth);
   }

   /**
    * Add a series of connected 3D lines to this builder.
    * 
    * @param transform the transform to apply to the points. Not modified.
    * @param points    coordinates of the line end points. Not modified.
    * @param lineWidth width of the lines.
    * @param close     whether the end of the given array of points should be connected to the
    *                  beginning or not.
    */
   public void addMultiLine(RigidBodyTransformReadOnly transform, List<? extends Point2DReadOnly> points, double lineWidth, boolean close)
   {
      List<Point3D> point3Ds = points.stream().map(Point3D::new).collect(Collectors.toList());
      point3Ds.forEach(transform::transform);
      addMultiLine(point3Ds, lineWidth, close);
   }

   /**
    * Add a series of connected 3D lines to this builder.
    * 
    * @param points    coordinates of the line end points. Not modified.
    * @param lineWidth width of the lines.
    * @param close     whether the end of the given array of points should be connected to the
    *                  beginning or not.
    */
   public void addMultiLine(List<? extends Point3DReadOnly> points, double lineWidth, boolean close)
   {
      if (points.size() < 2)
         return;

      for (int i = 1; i < points.size(); i++)
      {
         Point3DReadOnly start = points.get(i - 1);
         Point3DReadOnly end = points.get(i);
         addLine(start, end, lineWidth);
      }

      if (close)
      {
         Point3DReadOnly start = points.get(points.size() - 1);
         Point3DReadOnly end = points.get(0);
         addLine(start, end, lineWidth);
      }
   }

   /**
    * Add a series of connected 3D lines to this builder.
    * 
    * @param points    coordinates of the line end points. Not modified.
    * @param lineWidth width of the lines.
    * @param close     whether the end of the given array of points should be connected to the
    *                  beginning or not.
    */
   public void addMultiLine(Point3DReadOnly[] points, double lineWidth, boolean close)
   {
      if (points.length < 2)
         return;

      for (int i = 1; i < points.length; i++)
      {
         Point3DReadOnly start = points[i - 1];
         Point3DReadOnly end = points[i];
         addLine(start, end, lineWidth);
      }

      if (close)
      {
         Point3DReadOnly start = points[points.length - 1];
         Point3DReadOnly end = points[0];
         addLine(start, end, lineWidth);
      }
   }

   /**
    * Add a polygon to this builder, given its 2D vertex coordinates and its transform to world. No
    * check is performed on the ordering of the vertices.
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPose the pose of the polygon. Can be {@code null}. Not modified.
    * @param polygon     the polygon's 2D vertices. Not modified.
    */
   public void addPolygonCounterClockwise(RigidBodyTransformReadOnly polygonPose, List<? extends Point2DReadOnly> polygon)
   {
      addTriangleMesh3D(TriangleMesh3DFactories.PolygonCounterClockwise(polygonPose, polygon));
   }

   /**
    * Add a 2D polygon to this builder.
    * <p>
    * Expected result for a polygon 2D:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Polygon2D.png"
    * height=250px/>
    * </p>
    * 
    * @param polygonPose the pose of the polygon. Can be {@code null}. Not modified.
    * @param polygon     the polygon to render.
    */
   public void addPolygon(RigidBodyTransformReadOnly polygonPose, ConvexPolygon2DReadOnly polygon)
   {
      addTriangleMesh3D(TriangleMesh3DFactories.Polygon(polygonPose, polygon));
   }

   /**
    * Add a sphere centered at (0, 0, 0) to this builder.
    * <p>
    * Expected result for {@code addSphere(0.15)}:<br>
    * <img src=
    * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/VisualDefinitionFactoryJavadoc/Sphere.png"
    * height=250px/>
    * </p>
    * 
    * @param radius the sphere radius.
    */
   public void addSphere(double radius)
   {
      addSphere(radius, null);
   }

   /**
    * Add a sphere centered to this builder.
    * 
    * @param radius the sphere radius.
    * @param offset the coordinate of the sphere. Not modified.
    */
   public void addSphere(double radius, Tuple3DReadOnly offset)
   {
      addGeometry(new Sphere3DDefinition(radius), offset, null);
   }

   /**
    * Add an array of spheres to this builder.
    * 
    * @param radius  the radius of the spheres. Not modified.
    * @param offsets the coordinates of each sphere. Not modified.
    */
   public void addSpheres(float radius, Point3DReadOnly[] offsets)
   {
      for (Point3DReadOnly point : offsets)
         addSphere(radius, point);
   }

   /**
    * Add a regular tetrahedron to this builder.
    * 
    * @param edgeLength edge length of the tetrahedron.
    * @param offset     coordinate of the geometry. Can be {@code null}. Not modified.
    */
   public void addTetrahedron(double edgeLength, Tuple3DReadOnly offset)
   {
      addTetrahedron(edgeLength, offset, null);
   }

   /**
    * Add a regular tetrahedron to this builder.
    * 
    * @param edgeLength  edge length of the tetrahedron.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    */
   public void addTetrahedron(double edgeLength, Tuple3DReadOnly offset, Orientation3DReadOnly orientation)
   {
      addGeometry(new Tetrahedron3DDefinition(edgeLength), offset, orientation);
   }

   /**
    * Clears the meshes contained in this builder.
    */
   public void clear()
   {
      triangleMesh.clear();
   }

   /**
    * Generates the triangle mesh containing all the shapes/meshes previously added.
    * 
    * @return the resulting mesh.
    */
   public TriangleMesh3DDefinition generateTriangleMesh3D()
   {
      return triangleMesh.createTriangleMesh3DDefinition();
   }

   /**
    * This class is used instead of {@link TriangleMesh3DDefinition} in {@link TriangleMesh3DBuilder}.
    * It allows to grow a triangle mesh efficiently.
    */
   public static class MutableTriangleMesh3DData
   {
      private final RecyclingArrayList<Point3D32> vertices = new RecyclingArrayList<>(Point3D32.class);
      private final RecyclingArrayList<Point2D32> textures = new RecyclingArrayList<>(Point2D32.class);
      private final RecyclingArrayList<Vector3D32> normals = new RecyclingArrayList<>(Vector3D32.class);
      private final TIntArrayList triangleIndices = new TIntArrayList();

      /**
       * Creates an empty mesh.
       */
      public MutableTriangleMesh3DData()
      {
      }

      /**
       * Clears this triangle mesh. After calling this method, this is an empty mesh.
       */
      public void clear()
      {
         vertices.clear();
         textures.clear();
         normals.clear();
         triangleIndices.reset();
      }

      /**
       * Creates a triangle mesh definition that can be integrated to the main data structure.
       * 
       * @return the immutable mesh data holder.
       * @see TriangleMesh3DDefinition
       */
      public TriangleMesh3DDefinition createTriangleMesh3DDefinition()
      {
         Point3D32[] vertexArray = vertices.toArray(new Point3D32[0]);
         Point2D32[] texturePointArray = textures.toArray(new Point2D32[0]);
         Vector3D32[] vertexNormalArray = normals.toArray(new Vector3D32[0]);
         int[] triangleIndexArray = triangleIndices.toArray();
         return new TriangleMesh3DDefinition(vertexArray, texturePointArray, vertexNormalArray, triangleIndexArray);
      }

      /**
       * Append a triangle mesh to this.
       * 
       * @param triangleMesh          the mesh to append. Not modified.
       * @param updateTriangleIndices whether the indices of the given mesh should be updated when
       *                              appended. Highly recommended, set it to false only if you what you
       *                              are doing.
       */
      public void add(TriangleMesh3DDefinition triangleMesh, boolean updateTriangleIndices)
      {
         add(triangleMesh, null, null, updateTriangleIndices);
      }

      /**
       * Append a triangle mesh to this.
       * 
       * @param triangleMesh          the mesh to append. Not modified.
       * @param positionOffset        used to translate the mesh. Can be {@code null}.
       * @param orientationOffset     used to rotate the mesh. Can be {@code null}.
       * @param updateTriangleIndices whether the indices of the given mesh should be updated when
       *                              appended. Highly recommended, set it to false only if you what you
       *                              are doing.
       */
      public void add(TriangleMesh3DDefinition triangleMesh,
                      Tuple3DReadOnly positionOffset,
                      Orientation3DReadOnly orientationOffset,
                      boolean updateTriangleIndices)
      {
         if (triangleMesh == null)
            return;

         Point3D32[] otherVertices = triangleMesh.getVertices();

         if (otherVertices == null || otherVertices.length < 3)
            return;
         int[] otherTriangleIndices = triangleMesh.getTriangleIndices();
         if (otherTriangleIndices == null || otherTriangleIndices.length < 3)
            return;

         if (updateTriangleIndices)
         {
            int shift = vertices.size();
            for (int otherTriangleIndex : otherTriangleIndices)
               triangleIndices.add(otherTriangleIndex + shift);
         }
         else
         {
            triangleIndices.add(otherTriangleIndices);
         }

         for (Point3D32 otherVertex : otherVertices)
         {
            Point3D32 vertex = vertices.add();
            vertex.set(otherVertex);
            if (orientationOffset != null)
               orientationOffset.transform(vertex);
            if (positionOffset != null)
               vertex.add(positionOffset);
         }

         for (Point2D32 otherTexture : triangleMesh.getTextures())
         {
            textures.add().set(otherTexture);
         }

         for (Vector3D32 otherNormal : triangleMesh.getNormals())
         {
            Vector3D32 normal = normals.add();
            normal.set(otherNormal);
            if (orientationOffset != null)
               orientationOffset.transform(normal);
         }
      }

      /**
       * Append a triangle mesh to this.
       * 
       * @param other                 the triangle mesh to append. Not modified.
       * @param updateTriangleIndices whether the indices of the given mesh should be updated when
       *                              appended. Highly recommended, set it to false only if you what you
       *                              are doing.
       */
      public void add(MutableTriangleMesh3DData other, boolean updateTriangleIndices)
      {
         add(other, null, null, updateTriangleIndices);
      }

      /**
       * Append a triangle mesh to this.
       * 
       * @param other                 the triangle mesh to append. Not modified.
       * @param positionOffset        used to translate the mesh. Can be {@code null}.
       * @param orientationOffset     used to rotate the mesh. Can be {@code null}.
       * @param updateTriangleIndices whether the indices of the given mesh should be updated when
       *                              appended. Highly recommended, set it to false only if you what you
       *                              are doing.
       */
      public void add(MutableTriangleMesh3DData other, Tuple3DReadOnly positionOffset, Orientation3DReadOnly orientationOffset, boolean updateTriangleIndices)
      {
         if (updateTriangleIndices)
         {
            int shift = vertices.size();
            for (int i = 0; i < other.triangleIndices.size(); i++)
               triangleIndices.add(other.triangleIndices.get(i) + shift);
         }
         else
         {
            triangleIndices.addAll(other.triangleIndices);
         }

         for (int i = 0; i < other.vertices.size(); i++)
         {
            Point3D32 vertex = vertices.add();
            vertex.set(other.vertices.get(i));
            if (orientationOffset != null)
               orientationOffset.transform(vertex);
            if (positionOffset != null)
               vertex.add(positionOffset);
         }

         for (int i = 0; i < other.textures.size(); i++)
         {
            textures.add().set(other.textures.get(i));
         }

         for (int i = 0; i < other.normals.size(); i++)
         {
            Vector3D32 normal = normals.add();
            normal.set(other.normals.get(i));
            if (orientationOffset != null)
               orientationOffset.transform(normal);
         }
      }
   }
}
