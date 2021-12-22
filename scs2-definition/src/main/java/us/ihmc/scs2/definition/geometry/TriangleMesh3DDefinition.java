package us.ihmc.scs2.definition.geometry;

import java.util.Arrays;
import java.util.stream.Stream;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;

public class TriangleMesh3DDefinition extends GeometryDefinition
{
   private Point3D32[] vertices;
   private Point2D32[] textures;
   private Vector3D32[] normals;
   private int[] triangleIndices;

   public TriangleMesh3DDefinition()
   {
      setName("TriangleMesh");
   }

   public TriangleMesh3DDefinition(Point3D32[] vertices, Point2D32[] textures, Vector3D32[] normals, int[] triangleIndices)
   {
      this();
      setVertices(vertices);
      setTextures(textures);
      setNormals(normals);
      setTriangleIndices(triangleIndices);
   }

   public TriangleMesh3DDefinition(String name, Point3D32[] vertices, Point2D32[] textures, Vector3D32[] normals, int[] triangleIndices)
   {
      setName(name);
      setVertices(vertices);
      setTextures(textures);
      setNormals(normals);
      setTriangleIndices(triangleIndices);
   }

   public TriangleMesh3DDefinition(TriangleMesh3DDefinition other)
   {
      setName(other.getName());
      vertices = other.vertices == null ? null : Stream.of(other.vertices).map(Point3D32::new).toArray(Point3D32[]::new);
      textures = other.textures == null ? null : Stream.of(other.textures).map(Point2D32::new).toArray(Point2D32[]::new);
      normals = other.normals == null ? null : Stream.of(other.normals).map(Vector3D32::new).toArray(Vector3D32[]::new);
      triangleIndices = other.triangleIndices == null ? null : Arrays.copyOf(other.triangleIndices, other.triangleIndices.length);
   }

   @XmlElement
   public void setVertices(Point3D32[] vertices)
   {
      this.vertices = vertices;
   }

   @XmlElement
   public void setTextures(Point2D32[] textures)
   {
      this.textures = textures;
   }

   @XmlElement
   public void setNormals(Vector3D32[] normals)
   {
      this.normals = normals;
   }

   @XmlElement
   public void setTriangleIndices(int[] triangleIndices)
   {
      this.triangleIndices = triangleIndices;
   }

   public Point3D32[] getVertices()
   {
      return vertices;
   }

   public Point2D32[] getTextures()
   {
      return textures;
   }

   public Vector3D32[] getNormals()
   {
      return normals;
   }

   public int[] getTriangleIndices()
   {
      return triangleIndices;
   }

   @Override
   public TriangleMesh3DDefinition copy()
   {
      return new TriangleMesh3DDefinition(this);
   }
}
