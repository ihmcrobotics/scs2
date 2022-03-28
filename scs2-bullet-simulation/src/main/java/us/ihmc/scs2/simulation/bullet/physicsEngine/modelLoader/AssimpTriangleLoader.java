package us.ihmc.scs2.simulation.bullet.physicsEngine.modelLoader;

import org.lwjgl.assimp.AIMesh;
import org.lwjgl.assimp.AIVector3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;

/**
 * Loads the vertices as triangles, in triplets of 3, with duplicates.
 */
public class AssimpTriangleLoader
{
   public static final int MAX_VERTICES = 65538;
   private final AIMesh assimpMesh;
   private boolean hasNormals;
   private boolean hasTangents;
   private boolean hasBitangents;
   private boolean hasColors;
   private boolean hasTextureCoordinates;
   private int numberOfVertices;
   private ArrayList<Point3D32> vertices = new ArrayList<>();

   public AssimpTriangleLoader(AIMesh assimpMesh)
   {
      this.assimpMesh = assimpMesh;
   }

   public ArrayList<Point3D32> loadVerticesAsList()
   {
      // vertices and faces guaranteed to present; else need to check null
      numberOfVertices = assimpMesh.mNumVertices();
      LogTools.debug("Number of vertices: {}", numberOfVertices);
      if (numberOfVertices > MAX_VERTICES)
         LogTools.error("Mesh contains too many vertices! {}/{}", numberOfVertices, MAX_VERTICES);

      hasNormals = assimpMesh.mNormals() != null;
      LogTools.debug("Has normals: {}", hasNormals);

      hasTangents = assimpMesh.mTangents() != null;
      LogTools.debug("Has tangents: {}", hasTangents);

      hasBitangents = assimpMesh.mTangents() != null;
      LogTools.debug("Has bitangents: {}", hasBitangents);

      // TODO: There actually can be up to Assimp.AI_MAX_NUMBER_OF_COLOR_SETS (4) colors per vertex
      hasColors = assimpMesh.mColors(0) != null;
      LogTools.debug("Has colors: {}", hasColors);

      // TODO: There actually can be up to Assimp.AI_MAX_NUMBER_OF_TEXTURECOORDS (4) texture coodinates per vertex
      hasTextureCoordinates = assimpMesh.mTextureCoords(0) != null;
      LogTools.debug("Has texture coordinates: {}", hasTextureCoordinates);

      AIVector3D.Buffer assimpVerticesVector3DS = assimpMesh.mVertices();

      for (int j = 0; j < numberOfVertices; j++)
      {
         float x = assimpVerticesVector3DS.get(j).x();
         float y = assimpVerticesVector3DS.get(j).y();
         float z = assimpVerticesVector3DS.get(j).z();
         vertices.add(new Point3D32(x, y, z));

         if (hasNormals)
         {
            float normalX = assimpMesh.mNormals().get(j).x();
            float normalY = assimpMesh.mNormals().get(j).y();
            float normalZ = assimpMesh.mNormals().get(j).z();
         }
         if (hasTangents)
         {
            float tangentX = assimpMesh.mTangents().get(j).x();
            float tangentY = assimpMesh.mTangents().get(j).y();
            float tangentZ = assimpMesh.mTangents().get(j).z();
         }
         if (hasBitangents)
         {
            float bitangentX = assimpMesh.mBitangents().get(j).x();
            float bitangentY = assimpMesh.mBitangents().get(j).y();
            float bitangentZ = assimpMesh.mBitangents().get(j).z();
         }
         if (hasColors)
         {
            float r = assimpMesh.mColors(0).get(j).r();
            float g = assimpMesh.mColors(0).get(j).g();
            float b = assimpMesh.mColors(0).get(j).b();
            float a = assimpMesh.mColors(0).get(j).a();
         }
         if (hasTextureCoordinates)
         {
            // Assume 2D texture here
            float textureX = assimpMesh.mTextureCoords(0).get(j).x();
            float textureY = assimpMesh.mTextureCoords(0).get(j).y();
         }
      }

      return vertices;
   }

   public FloatBuffer loadVerticesAsFloatBuffer()
   {
      loadVerticesAsList();

      int vertexSize = 3; // We are just loading triangle vertex positions
      ByteBuffer directVertexByteBuffer = ByteBuffer.allocateDirect(numberOfVertices * vertexSize * Float.BYTES);
      directVertexByteBuffer.order(ByteOrder.nativeOrder());
      FloatBuffer vertexDirectBuffer = directVertexByteBuffer.asFloatBuffer();
      for (Point3D32 vertex : vertices)
      {
         vertexDirectBuffer.put(vertex.getX32());
         vertexDirectBuffer.put(vertex.getY32());
         vertexDirectBuffer.put(vertex.getZ32());
      }

      return vertexDirectBuffer;
   }
}
