package us.ihmc.scs2.simulation.bullet.physicsEngine.modelLoader;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.function.Consumer;

import org.lwjgl.PointerBuffer;
import org.lwjgl.assimp.AIMesh;
import org.lwjgl.assimp.AIPropertyStore;
import org.lwjgl.assimp.AIScene;
import org.lwjgl.assimp.Assimp;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;

public class AssimpLoader
{
   public static void iterateMeshes(String modelFilePath, Consumer<AIMesh> assimpMeshConsumer)
   {
      AssimpResourceImporter assimpResourceImporter = new AssimpResourceImporter();

      AIPropertyStore assimpPropertyStore = Assimp.aiCreatePropertyStore();

      int postProcessingSteps = 0; // none

      // We read UVs flipped from assimp default.
      postProcessingSteps += Assimp.aiProcess_FlipUVs;

      // Force loading triangle primitives.
      postProcessingSteps += Assimp.aiProcess_Triangulate;

      // Split the meshes into parts with no more than 65536 vertices so it can fit in a short.
      // TODO: Necessary for Bullet?
      Assimp.aiSetImportPropertyInteger(assimpPropertyStore, Assimp.AI_CONFIG_PP_SLM_VERTEX_LIMIT, 1 << 16);
      postProcessingSteps += Assimp.aiProcess_SplitLargeMeshes;

      AIScene assimpScene = assimpResourceImporter.importScene(modelFilePath, postProcessingSteps, assimpPropertyStore);
      PointerBuffer meshesPointerBuffer = assimpScene.mMeshes();

      int numberOfMeshes = assimpScene.mNumMeshes();
      for (int i = 0; i < numberOfMeshes; i++)
      {
         AIMesh assimpMesh = new AIMesh(MemoryUtil.memByteBuffer(meshesPointerBuffer.get(i), AIMesh.SIZEOF));
         assimpMeshConsumer.accept(assimpMesh);
      }
   }

   public static List<FloatBuffer> loadTriangleVertexPositionsAsFloatBuffer(String modelFilePath)
   {
      List<FloatBuffer> vertexBuffers = new ArrayList<>();
      iterateMeshes(modelFilePath, assimpMesh ->
      {
         AssimpTriangleLoader assimpTriangleLoader = new AssimpTriangleLoader(assimpMesh);
         FloatBuffer vetricesFloatBuffer = assimpTriangleLoader.loadVerticesAsFloatBuffer();
         vertexBuffers.add(vetricesFloatBuffer);
      });

      return vertexBuffers;
   }

   public static List<List<Point3D32>> loadTriangleVertexPositionsAsList(String modelFilePath)
   {
      List<List<Point3D32>> vertexLists = new ArrayList<>();
      iterateMeshes(modelFilePath, assimpMesh ->
      {
         AssimpTriangleLoader assimpTriangleLoader = new AssimpTriangleLoader(assimpMesh);
         List<Point3D32> vertexList = assimpTriangleLoader.loadVerticesAsList();
         vertexLists.add(vertexList);
      });

      return vertexLists;
   }

   public static List<HashSet<Vector3D32>> loadUniqueVertexPositions(String modelFilePath)
   {
      List<HashSet<Vector3D32>> vertexSets = new ArrayList<>();
      iterateMeshes(modelFilePath, assimpMesh ->
      {
         AssimpVertexLoader assimpVertexLoader = new AssimpVertexLoader(assimpMesh);
         vertexSets.add(assimpVertexLoader.loadVertices());
      });

      return vertexSets;
   }
}
