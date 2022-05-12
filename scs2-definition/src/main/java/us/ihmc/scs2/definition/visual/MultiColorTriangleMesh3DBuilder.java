package us.ihmc.scs2.definition.visual;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

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

// TODO This class needs major cleanup
public class MultiColorTriangleMesh3DBuilder
{
   private final TriangleMesh3DBuilder meshBuilder = new TriangleMesh3DBuilder();
   private TextureDefinitionColorPalette colorPalette;

   /**
    * Creates an empty mesh builder using the default texture color palette.
    */
   public MultiColorTriangleMesh3DBuilder()
   {
      colorPalette = new TextureDefinitionColorPalette2D();
   }

   /**
    * Creates an empty mesh builder given a texture color palette to use.
    * 
    * @param colorPalette the color palette with this mesh builder.
    */
   public MultiColorTriangleMesh3DBuilder(TextureDefinitionColorPalette colorPalette)
   {
      this.colorPalette = colorPalette;
   }

   /**
    * Combines the given mesh with the triangle mesh contained in this builder.
    * 
    * @param triangleMesh the triangle mesh to combine. Not Modified.
    * @param color        the color of the triangle mesh. Not modified.
    */
   public void addTriangleMesh3D(TriangleMesh3DDefinition triangleMesh, ColorDefinition color)
   {
      addTriangleMesh3D(triangleMesh, null, null, color);
   }

   /**
    * Translates then combines the given triangle mesh with the triangle mesh contained in this
    * builder.
    * 
    * @param triangleMesh the triangle mesh to combine. Not Modified.
    * @param offset       used to translate the given triangle mesh. Can be {@code null}. Not modified.
    * @param color        the color of the triangle mesh. Not modified.
    */
   public void addTriangleMesh3D(TriangleMesh3DDefinition triangleMesh, Tuple3DReadOnly offset, ColorDefinition color)
   {
      addTriangleMesh3D(triangleMesh, offset, null, color);
   }

   /**
    * Rotates, translates, then combines the given triangle mesh with the triangle mesh contained in
    * this builder.
    * 
    * @param triangleMesh the triangle mesh to combine. Not Modified.
    * @param offset       used to translate the given triangle mesh. Can be {@code null}. Not modified.
    * @param orientation  used to rotate the given triangle mesh. Can be {@code null}. Not modified.
    * @param color        the color of the triangle mesh. Not modified.
    */
   public void addTriangleMesh3D(TriangleMesh3DDefinition triangleMesh, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, ColorDefinition color)
   {
      meshBuilder.addTriangleMesh3D(setColor(triangleMesh, color), offset, orientation);
   }

   private TriangleMesh3DDefinition setColor(TriangleMesh3DDefinition input, ColorDefinition color)
   {
      if (input == null)
         return null;
      Point3D32[] vertices = input.getVertices();
      int[] triangleIndices = input.getTriangleIndices();
      Vector3D32[] normals = input.getNormals();
      Point2D32[] textures = new Point2D32[input.getTextures().length];
      Arrays.fill(textures, colorPalette.getTextureLocation(color));
      return new TriangleMesh3DDefinition(vertices, textures, normals, triangleIndices);
   }

   /**
    * Rotates, translates, then combines the given geometry to the triangle mesh contained in this
    * builder.
    * 
    * @param geometry    the geometry to combine. Not Modified.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    * @param color       the color of the geometry. Not modified.
    */
   public void addGeometry(GeometryDefinition geometry, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, ColorDefinition color)
   {
      addTriangleMesh3D(TriangleMesh3DFactories.TriangleMesh(geometry), offset, orientation, color);
   }

   /**
    * Add a box centered at (0, 0, 0) to this builder.
    * 
    * @param sizeX box size along the x-axis.
    * @param sizeY box size along the y-axis.
    * @param sizeZ box size along the z-axis.
    * @param color the color of the geometry. Not modified.
    */
   public void addBox(double sizeX, double sizeY, double sizeZ, ColorDefinition color)
   {
      addBox(sizeX, sizeY, sizeZ, null, null, color);
   }

   /**
    * Add a box to this builder.
    * 
    * @param sizeX  box size along the x-axis.
    * @param sizeY  box size along the y-axis.
    * @param sizeZ  box size along the z-axis.
    * @param offset coordinate of the geometry. Can be {@code null}. Not modified.
    * @param color  the color of the geometry. Not modified.
    */
   public void addBox(double sizeX, double sizeY, double sizeZ, Tuple3DReadOnly offset, ColorDefinition color)
   {
      addBox(sizeX, sizeY, sizeZ, offset, null, color);
   }

   /**
    * Add a box to this builder.
    * 
    * @param sizeX       box size along the x-axis.
    * @param sizeY       box size along the y-axis.
    * @param sizeZ       box size along the z-axis.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    * @param color       the color of the geometry. Not modified.
    */
   public void addBox(double sizeX, double sizeY, double sizeZ, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, ColorDefinition color)
   {
      addGeometry(new Box3DDefinition(sizeX, sizeY, sizeZ), offset, orientation, color);
   }

   /**
    * Add a capsule centered at (0, 0, 0) to this builder.
    * 
    * @param length the capsule's length or height. Distance separating the center of the two half
    *               spheres.
    * @param radius the capsule's radius.
    * @param color  the color of the geometry. Not modified.
    */
   public void addCapsule(double length, double radius, ColorDefinition color)
   {
      addCapsule(length, radius, null, null, color);
   }

   /**
    * Add a capsule to this builder.
    * 
    * @param length the capsule's length or height. Distance separating the center of the two half
    *               spheres.
    * @param radius the capsule's radius.
    * @param offset coordinate of the geometry. Can be {@code null}. Not modified.
    * @param color  the color of the geometry. Not modified.
    */
   public void addCapsule(double length, double radius, Tuple3DReadOnly offset, ColorDefinition color)
   {
      addCapsule(length, radius, offset, null, color);
   }

   /**
    * Add a capsule to this builder.
    * 
    * @param length      the capsule's length or height. Distance separating the center of the two half
    *                    spheres.
    * @param radius      the capsule's radius.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    * @param color       the color of the geometry. Not modified.
    */
   public void addCapsule(double length, double radius, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, ColorDefinition color)
   {
      addGeometry(new Capsule3DDefinition(length, radius), offset, orientation, color);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with
    * the highest z value.
    * 
    * @param height height along z of the cone.
    * @param radius radius of the cone's base.
    * @param color  the color of the geometry. Not modified.
    */
   public void addCone(double height, double radius, ColorDefinition color)
   {
      addCone(height, radius, null, null, color);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with
    * the highest z value.
    * 
    * @param height height along z of the cone.
    * @param radius radius of the cone's base.
    * @param offset coordinate of the geometry. Can be {@code null}. Not modified.
    * @param color  the color of the geometry. Not modified.
    */
   public void addCone(double height, double radius, Tuple3DReadOnly offset, ColorDefinition color)
   {
      addCone(height, radius, offset, null, color);
   }

   /**
    * Add a cone to this builder. Its axis is aligned with the z-axis and its top is the vertex with
    * the highest z value in its local coordinate system.
    * 
    * @param height      height along z of the cone.
    * @param radius      radius of the cone's base.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    * @param color       the color of the geometry. Not modified.
    */
   public void addCone(double height, double radius, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, ColorDefinition color)
   {
      addGeometry(new Cone3DDefinition(height, radius), offset, orientation, color);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis.
    * 
    * @param height height along z of the cylinder.
    * @param radius the cylinder's radius.
    * @param color  the color of the geometry. Not modified.
    */
   public void addCylinder(double length, double radius, ColorDefinition color)
   {
      addCylinder(length, radius, null, null, color);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis.
    * 
    * @param height      height along z of the cylinder.
    * @param radius      the cylinder's radius.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    * @param color       the color of the geometry. Not modified.
    */
   public void addCylinder(double length, double radius, Tuple3DReadOnly offset, ColorDefinition color)
   {
      addCylinder(length, radius, offset, null, color);
   }

   /**
    * Add a cylinder to this builder. Its axis is aligned with the z-axis in its local coordinate
    * system.
    * 
    * @param height      height along z of the cylinder.
    * @param radius      the cylinder's radius.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    * @param color       the color of the geometry. Not modified.
    */
   public void addCylinder(double length, double radius, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, ColorDefinition color)
   {
      addGeometry(new Cylinder3DDefinition(radius, length), offset, orientation, color);
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
    * @param color     the color of the geometry. Not modified.
    */
   public void addLine(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth, ColorDefinition color)
   {
      addTriangleMesh3D(TriangleMesh3DFactories.Line(x0, y0, z0, xf, yf, zf, lineWidth), color);
   }

   /**
    * Add a 3D line to this builder.
    * 
    * @param start     start coordinate of the line. Not modified.
    * @param end       end coordinate of the line. Not modified.
    * @param lineWidth width of the line.
    * @param color     the color of the geometry. Not modified.
    */
   public void addLine(Tuple3DReadOnly start, Tuple3DReadOnly end, double lineWidth, ColorDefinition color)
   {
      addLine(start.getX(), start.getY(), start.getZ(), end.getX(), end.getY(), end.getZ(), lineWidth, color);
   }

   /**
    * Add a series of connected 3D lines to this builder.
    * 
    * @param transform the transform to apply to the points. Not modified.
    * @param points    coordinates of the line end points. Not modified.
    * @param lineWidth width of the lines.
    * @param close     whether the end of the given array of points should be connected to the
    *                  beginning or not.
    * @param color     the color of the geometry. Not modified.
    */
   public void addMultiLine(RigidBodyTransformReadOnly transform,
                            List<? extends Point2DReadOnly> points,
                            double lineWidth,
                            boolean close,
                            ColorDefinition color)
   {
      List<Point3D> point3Ds = points.stream().map(Point3D::new).collect(Collectors.toList());
      point3Ds.forEach(transform::transform);
      addMultiLine(point3Ds, lineWidth, close, color);
   }

   /**
    * Add a series of connected 3D lines to this builder.
    * 
    * @param points    coordinates of the line end points. Not modified.
    * @param lineWidth width of the lines.
    * @param close     whether the end of the given array of points should be connected to the
    *                  beginning or not.
    * @param color     the color of the geometry. Not modified.
    */
   public void addMultiLine(List<? extends Point3DReadOnly> points, double lineWidth, boolean close, ColorDefinition color)
   {
      if (points.size() < 2)
         return;

      for (int i = 1; i < points.size(); i++)
      {
         Point3DReadOnly start = points.get(i - 1);
         Point3DReadOnly end = points.get(i);
         addLine(start, end, lineWidth, color);
      }

      if (close)
      {
         Point3DReadOnly start = points.get(points.size() - 1);
         Point3DReadOnly end = points.get(0);
         addLine(start, end, lineWidth, color);
      }
   }

   /**
    * Add a series of connected 3D lines to this builder.
    * 
    * @param points    coordinates of the line end points. Not modified.
    * @param lineWidth width of the lines.
    * @param close     whether the end of the given array of points should be connected to the
    *                  beginning or not.
    * @param color     the color of the geometry. Not modified.
    */
   public void addMultiLine(Point3DReadOnly[] points, double lineWidth, boolean close, ColorDefinition color)
   {
      if (points.length < 2)
         return;

      for (int i = 1; i < points.length; i++)
      {
         Point3DReadOnly start = points[i - 1];
         Point3DReadOnly end = points[i];
         addLine(start, end, lineWidth, color);
      }

      if (close)
      {
         Point3DReadOnly start = points[points.length - 1];
         Point3DReadOnly end = points[0];
         addLine(start, end, lineWidth, color);
      }
   }

   /**
    * Add a polygon to this builder, given its 2D vertex coordinates and its transform to world. No
    * check is performed on the ordering of the vertices.
    * 
    * @param polygonPose the pose of the polygon. Can be {@code null}. Not modified.
    * @param polygon     the polygon's 2D vertices. Not modified.
    * @param color       the color of the geometry. Not modified.
    */
   public void addPolygonCounterClockwise(RigidBodyTransformReadOnly polygonPose, List<? extends Point2DReadOnly> polygon, ColorDefinition color)
   {
      addTriangleMesh3D(TriangleMesh3DFactories.PolygonCounterClockwise(polygonPose, polygon), color);
   }

   /**
    * Add a 2D polygon to this builder.
    * 
    * @param polygonPose the pose of the polygon. Can be {@code null}. Not modified.
    * @param polygon     the polygon to render.
    * @param color       the color of the geometry. Not modified.
    */
   public void addPolygon(RigidBodyTransformReadOnly polygonPose, ConvexPolygon2DReadOnly polygon, ColorDefinition color)
   {
      addTriangleMesh3D(TriangleMesh3DFactories.Polygon(polygonPose, polygon), color);
   }

   /**
    * Add a sphere centered at (0, 0, 0) to this builder.
    * 
    * @param radius the sphere radius.
    * @param color  the color of the geometry. Not modified.
    */
   public void addSphere(double radius, ColorDefinition color)
   {
      addSphere(radius, null, color);
   }

   /**
    * Add a sphere centered to this builder.
    * 
    * @param radius the sphere radius.
    * @param offset the coordinate of the sphere. Not modified.
    * @param color  the color of the geometry. Not modified.
    */
   public void addSphere(double radius, Tuple3DReadOnly offset, ColorDefinition color)
   {
      addGeometry(new Sphere3DDefinition(radius), offset, null, color);
   }

   /**
    * Add an array of spheres to this builder.
    * 
    * @param radius  the radius of the spheres. Not modified.
    * @param offsets the coordinates of each sphere. Not modified.
    * @param color   the color of the geometry. Not modified.
    */
   public void addSpheres(float radius, Point3DReadOnly[] offsets, ColorDefinition color)
   {
      for (Point3DReadOnly point : offsets)
         addSphere(radius, point, color);
   }

   /**
    * Add a regular tetrahedron to this builder.
    * 
    * @param edgeLength edge length of the tetrahedron.
    * @param offset     coordinate of the geometry. Can be {@code null}. Not modified.
    * @param color      the color of the geometry. Not modified.
    */
   public void addTetrahedron(double edgeLength, Tuple3DReadOnly offset, ColorDefinition color)
   {
      addTetrahedron(edgeLength, offset, null, color);
   }

   /**
    * Add a regular tetrahedron to this builder.
    * 
    * @param edgeLength  edge length of the tetrahedron.
    * @param offset      coordinate of the geometry. Can be {@code null}. Not modified.
    * @param orientation the orientation of the geometry. Can be {@code null}. Not modified.
    * @param color       the color of the geometry. Not modified.
    */
   public void addTetrahedron(double edgeLength, Tuple3DReadOnly offset, Orientation3DReadOnly orientation, ColorDefinition color)
   {
      addGeometry(new Tetrahedron3DDefinition(edgeLength), offset, orientation, color);
   }

   /**
    * Change the color palette used by this mesh builder.
    * 
    * @param newColorPalette color palette to use in this mesh builder.
    */
   public void changeColorPalette(TextureDefinitionColorPalette2D newColorPalette)
   {
      colorPalette = newColorPalette;
   }

   /**
    * Clears the meshes contained in this builder.
    */
   public void clear()
   {
      meshBuilder.clear();
   }

   public TextureDefinition generateTexture()
   {
      return colorPalette.getTextureDefinition();
   }

   public MaterialDefinition generateMaterial()
   {
      return new MaterialDefinition(generateTexture());
   }

   public TriangleMesh3DDefinition generateTriangleMesh3D()
   {
      return meshBuilder.generateTriangleMesh3D();
   }

   public VisualDefinition generateVisual()
   {
      return new VisualDefinition(generateTriangleMesh3D(), generateMaterial());
   }
}
