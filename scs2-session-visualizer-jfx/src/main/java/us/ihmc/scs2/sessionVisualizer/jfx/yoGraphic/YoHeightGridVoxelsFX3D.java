package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.Node;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.session.log.heightMap.HeightMapData;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * The height map viewer wired up by the log and remote session controllers: renders a packed elevation grid
 * ({@link HeightMapData}) by drawing each cell as its own discrete voxel centered on the cell's sample point, instead
 * of stitching the samples into one continuous surface the way {@link YoHeightGridFX3D} does. Without the interpolated "walls" between neighboring cells, height discontinuities,
 * outliers and missing cells read as they actually are in the data. Cells whose elevation is not finite are skipped.
 * <p>
 * Voxels are cubes whose side is the smaller of the two cell sizes, shrunk by {@link #VOXEL_FILL_RATIO} so adjacent
 * voxels stay visually distinguishable. Coloring uses {@link YoHeightGridFX3D}'s elevation colormap.
 * <p>
 * All voxels go into a single {@link TriangleMesh} built directly (rather than via a
 * {@code TriangleMesh3DDefinition}): {@link VertexFormat#POINT_NORMAL_TEXCOORD} indexes points, normals and texture
 * coordinates independently, so each voxel only needs its 8 corners and 1 colormap sample, and the 6 face normals are
 * shared by every voxel. Faces wind counter-clockwise seen from outside, so back faces are culled.
 * <p>
 * Same double-buffered pattern as {@link YoHeightGridFX3D}: {@link #setData(HeightMapData)} is called from outside,
 * {@link #computeBackground()} rebuilds the mesh when the data changed, and {@link #render()} swaps it in. The rebuild
 * is skipped while the graphic is hidden and picked up once it is shown again.
 */
public class YoHeightGridVoxelsFX3D extends YoGraphicFX3D
{
   private static final String ELEVATION_FIELD_NAME = "elevation";
   /** Fraction of the cell size each voxel occupies, leaving a gap between neighbors. */
   private static final double VOXEL_FILL_RATIO = 0.9;

   /**
    * Corner {@code i} of a voxel is at {@code -/+} half size along X, Y, Z for bit 0, 1, 2 of {@code i}. Each row is
    * one face as a counter-clockwise (seen from outside) quad, in the same order as the normals built in
    * {@link #buildVoxelMesh}: +X, -X, +Y, -Y, +Z, -Z.
    */
   private static final int[][] FACE_CORNERS = {{1, 3, 7, 5}, {0, 4, 6, 2}, {2, 6, 7, 3}, {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 2, 3, 1}};

   private final MeshView meshView = new MeshView();

   private volatile HeightMapData newData;
   private HeightMapData oldData;
   private Mesh newMesh;
   private boolean clearMesh = false;

   public YoHeightGridVoxelsFX3D()
   {
      meshView.setCullFace(CullFace.BACK);
      meshView.idProperty().bind(nameProperty());
      meshView.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);
      meshView.setMaterial(YoHeightGridFX3D.createColormapMaterial());
   }

   /** Called from outside (log-viewer / live-feed wiring) whenever the height map data changes. Cheap, FX-thread-safe. */
   public void setData(HeightMapData data)
   {
      newData = data;
   }

   @Override
   public void computeBackground()
   {
      HeightMapData data = newData;

      if (data == null)
      {
         if (oldData != null)
         {
            clearMesh = true;
            oldData = null;
         }
         return;
      }
      if (data == oldData || !isVisible())
         return;

      oldData = data;

      Integer elevationOffset = data.findFieldOffset(ELEVATION_FIELD_NAME);
      if (elevationOffset == null || data.getRowCount() == 0 || data.getColumnCount() == 0)
      {
         clearMesh = true;
         return;
      }

      TriangleMesh mesh = buildVoxelMesh(data, elevationOffset);
      if (mesh == null)
         clearMesh = true;
      else
         newMesh = mesh;
   }

   private static TriangleMesh buildVoxelMesh(HeightMapData data, int elevationOffset)
   {
      int rowCount = data.getRowCount();
      int columnCount = data.getColumnCount();
      ByteBuffer cellData = ByteBuffer.wrap(data.getData()).order(ByteOrder.LITTLE_ENDIAN);

      Quaternion orientation = new Quaternion(data.getOrientationX(), data.getOrientationY(), data.getOrientationZ(), data.getOrientationW());
      Point3D position = new Point3D(data.getPositionX(), data.getPositionY(), data.getPositionZ());
      RigidBodyTransform gridToWorld = new RigidBodyTransform(orientation, position);

      // Same convention as YoHeightGridFX3D: X/Y follow the grid's pose, Z is the cell's absolute world elevation.
      double halfSize = 0.5 * VOXEL_FILL_RATIO * Math.min(data.getCellSizeX(), data.getCellSizeY());
      Vector3D halfX = new Vector3D(halfSize, 0.0, 0.0);
      Vector3D halfY = new Vector3D(0.0, halfSize, 0.0);
      orientation.transform(halfX);
      orientation.transform(halfY);
      halfX.setZ(0.0);
      halfY.setZ(0.0);

      float[] points = new float[rowCount * columnCount * 8 * 3];
      float[] texCoords = new float[rowCount * columnCount * 2];
      int voxelCount = 0;
      Point3D center = new Point3D();

      for (int row = 0; row < rowCount; row++)
      {
         for (int col = 0; col < columnCount; col++)
         {
            int cellOffset = row * data.getRowStride() + col * data.getCellStride() + elevationOffset;
            if (cellOffset + Float.BYTES > cellData.capacity())
               continue;
            float elevation = cellData.getFloat(cellOffset);
            if (!Float.isFinite(elevation))
               continue;

            center.set(col * data.getCellSizeX(), row * data.getCellSizeY(), 0.0);
            gridToWorld.transform(center);
            center.setZ(elevation);

            int p = voxelCount * 8 * 3;
            for (int corner = 0; corner < 8; corner++)
            {
               double signX = (corner & 1) == 0 ? -1.0 : 1.0;
               double signY = (corner & 2) == 0 ? -1.0 : 1.0;
               double signZ = (corner & 4) == 0 ? -1.0 : 1.0;
               points[p++] = (float) (center.getX() + signX * halfX.getX() + signY * halfY.getX());
               points[p++] = (float) (center.getY() + signX * halfX.getY() + signY * halfY.getY());
               points[p++] = (float) (center.getZ() + signZ * halfSize);
            }

            texCoords[2 * voxelCount] = YoHeightGridFX3D.toColormapU(elevation);
            texCoords[2 * voxelCount + 1] = 0.5f;
            voxelCount++;
         }
      }

      if (voxelCount == 0)
         return null;

      Vector3D[] faceNormals = {new Vector3D(halfX), new Vector3D(halfX), new Vector3D(halfY), new Vector3D(halfY), new Vector3D(0.0, 0.0, 1.0),
                                new Vector3D(0.0, 0.0, -1.0)};
      faceNormals[1].negate();
      faceNormals[3].negate();
      float[] normals = new float[faceNormals.length * 3];
      for (int i = 0; i < faceNormals.length; i++)
      {
         faceNormals[i].normalize();
         normals[3 * i] = (float) faceNormals[i].getX();
         normals[3 * i + 1] = (float) faceNormals[i].getY();
         normals[3 * i + 2] = (float) faceNormals[i].getZ();
      }

      // Per triangle: 3 x (point, normal, texCoord) indices; 2 triangles per face, 6 faces per voxel.
      int[] faces = new int[voxelCount * FACE_CORNERS.length * 2 * 3 * 3];
      int f = 0;
      for (int voxel = 0; voxel < voxelCount; voxel++)
      {
         int firstPoint = voxel * 8;
         for (int face = 0; face < FACE_CORNERS.length; face++)
         {
            int[] quad = FACE_CORNERS[face];
            int[] triangleCorners = {quad[0], quad[1], quad[2], quad[0], quad[2], quad[3]};
            for (int corner : triangleCorners)
            {
               faces[f++] = firstPoint + corner;
               faces[f++] = face;
               faces[f++] = voxel;
            }
         }
      }

      TriangleMesh mesh = new TriangleMesh(VertexFormat.POINT_NORMAL_TEXCOORD);
      mesh.getPoints().setAll(points, 0, voxelCount * 8 * 3);
      mesh.getTexCoords().setAll(texCoords, 0, voxelCount * 2);
      mesh.getNormals().setAll(normals);
      mesh.getFaces().setAll(faces);
      return mesh;
   }

   @Override
   public void render()
   {
      if (clearMesh)
      {
         clearMesh = false;
         meshView.setMesh(null);
      }

      if (newMesh != null)
      {
         meshView.setMesh(newMesh);
         newMesh = null;
      }
   }

   @Override
   public Node getNode()
   {
      return meshView;
   }

   @Override
   public void clear()
   {
      newData = null;
      oldData = null;
   }

   @Override
   public YoHeightGridVoxelsFX3D clone()
   {
      return new YoHeightGridVoxelsFX3D();
   }
}
