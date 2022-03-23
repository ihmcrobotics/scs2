package us.ihmc.scs2.simulation.bullet.physicsEngine.modelLoader;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

import org.lwjgl.PointerBuffer;
import org.lwjgl.assimp.AIMesh;
import org.lwjgl.assimp.AIPropertyStore;
import org.lwjgl.assimp.AIScene;
import org.lwjgl.assimp.AIVector3D;
import org.lwjgl.assimp.Assimp;
import org.lwjgl.system.MemoryUtil;

import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;

public class AssimpLoader
{

   public static List<FloatBuffer> loadVertices(String modelFilePath)
   {
      AssimpResourceImporter assimpResourceImporter = new AssimpResourceImporter();

      AIPropertyStore assimpPropertyStore = Assimp.aiCreatePropertyStore();

      int postProcessingSteps = 0; // none

      /** libGDX reads UVs flipped from assimp default */
      postProcessingSteps += Assimp.aiProcess_FlipUVs;

      /** libGDX needs triangles */
      postProcessingSteps += Assimp.aiProcess_Triangulate;

      /**
       * libGDX has limits in MeshBuilder. Not sure if there is a triangle limit.
       */
      Assimp.aiSetImportPropertyInteger(assimpPropertyStore, Assimp.AI_CONFIG_PP_SLM_VERTEX_LIMIT, MeshBuilder.MAX_VERTICES);
      postProcessingSteps += Assimp.aiProcess_SplitLargeMeshes;

      AIScene assimpScene = assimpResourceImporter.importScene(modelFilePath, postProcessingSteps, assimpPropertyStore);
      List<FloatBuffer> vertexBuffers = new ArrayList<>();
      PointerBuffer meshesPointerBuffer = assimpScene.mMeshes();

      for (int i = 0; i < assimpScene.mNumMeshes(); i++)
      {
         AIMesh assimpMesh = new AIMesh(MemoryUtil.memByteBuffer(meshesPointerBuffer.get(i), AIMesh.SIZEOF));
         AIVector3D.Buffer assimpVerticesVector3DS = assimpMesh.mVertices();
         int vertexSize = 3;
         FloatBuffer vertexHeapBuffer = FloatBuffer.allocate(assimpMesh.mNumVertices() * vertexSize);

         for (int j = 0; j < assimpMesh.mNumVertices(); j++)
         {
            float x = assimpVerticesVector3DS.get(j).x();
            float y = assimpVerticesVector3DS.get(j).y();
            float z = assimpVerticesVector3DS.get(j).z();
            vertexHeapBuffer.put(x);
            vertexHeapBuffer.put(y);
            vertexHeapBuffer.put(z);
         }
      }

      return vertexBuffers;
   }
}
