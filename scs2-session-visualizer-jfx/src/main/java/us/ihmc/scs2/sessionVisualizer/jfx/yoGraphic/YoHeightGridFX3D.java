package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import javafx.scene.Node;
import javafx.scene.image.Image;
import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.session.log.heightScan.HeightScanData;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXTriangleMesh3DDefinitionInterpreter;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Renders a {@code heightScan.mcap}-sourced grid ({@link HeightScanData}) as a per-vertex-colored terrain mesh,
 * mirroring how Foxglove renders a {@code foxglove.Grid} panel. There is no existing SCS2 graphic for a
 * grid/heightmap surface, so this builds the mesh from lower-level primitives:
 * <ul>
 * <li>Elevation is mapped to a color via {@link #getRedGreenBlue(double)}, a magenta-blue-green-yellow-orange
 * cyclic gradient keyed off absolute world height (not the current grid's min/max) so coloring reads as stable
 * topographic contour bands and doesn't shift between frames the way a per-frame-normalized range would.
 * <li>Per-vertex coloring uses a hand-built {@link TriangleMesh3DDefinition} with a distinct texture UV per vertex,
 * sampling into a dedicated single-row gradient strip ({@link #buildColormapImage(int)}) rather than
 * {@code TextureDefinitionColorAdaptivePalette} (built for many unrelated discrete colors, not one smooth 1D ramp -
 * it packs colors into a square image top-down, which for a ramp this size leaves most of the image blank and
 * makes sequential gradient samples wrap across unrelated texture rows; JavaFX auto-generates mipmaps for any
 * material diffuse map with no public way to disable it, so at minified mip levels that blank space and row-wrap
 * bled into the gradient as visible artifacts - streaks of one color bleeding into an unrelated one). A strip where
 * every texel is real, sequential gradient data has nothing but gradient-adjacent colors to blend with at any mip
 * level, so minification just smooths the ramp instead of corrupting it.
 * </ul>
 * Normals are approximated as a single uniform "up" vector (the grid's local +Z, rotated by its pose) rather than
 * computed per-vertex from neighboring cell heights - simple and sufficient given color is what communicates
 * elevation here; {@link CullFace#NONE} is used so an unexpected winding order doesn't hide the mesh.
 * <p>
 * Follows {@code YoPolygonExtrudedFX3D}'s double-buffered pattern: {@link #setData(HeightScanData)} is called from
 * outside (see the log-viewer wiring that owns a {@code HeightScanMcapScrubber}), {@link #computeBackground()}
 * (background thread, ~100ms cadence) rebuilds the mesh only when the data actually changed, and {@link #render()}
 * (FX thread, every frame) just swaps the prebuilt mesh in. The material/colormap never changes, so it's built once.
 */
public class YoHeightGridFX3D extends YoGraphicFX3D
{
   private static final String ELEVATION_FIELD_NAME = "elevation";

   /** Width, in meters, of one color segment of {@link #getRedGreenBlue(double)}'s cycle (5 segments per cycle). */
   private static final double GRADIENT_SIZE = 0.2;
   private static final double GRADIENT_LENGTH = 5.0 * GRADIENT_SIZE;
   /** Number of texels in the gradient strip - GPU bilinear sampling smooths between them, so this just needs to
    *  be fine enough that adjacent texels are visually indistinguishable. */
   private static final int COLORMAP_SAMPLES = 256;

   private final MeshView meshView = new MeshView();

   private volatile HeightScanData newData;
   private HeightScanData oldData;
   private Mesh newMesh;
   private boolean clearMesh = false;

   public YoHeightGridFX3D()
   {
      meshView.setCullFace(CullFace.NONE);
      meshView.idProperty().bind(nameProperty());
      meshView.getProperties().put(YO_GRAPHICFX_ITEM_KEY, this);

      PhongMaterial material = new PhongMaterial();
      material.setDiffuseMap(buildColormapImage(COLORMAP_SAMPLES));
      // Default PhongMaterial specular reflectivity shows up as bright highlight streaks on the steep "wall"
      // triangles between big height jumps - not wanted for a flat, matte, color-coded data surface like this one.
      material.setSpecularColor(Color.TRANSPARENT);
      material.setSpecularPower(0.0);
      meshView.setMaterial(material);
   }

   /** Called from outside (log-viewer wiring) whenever the scrubbed height scan data changes. Cheap, FX-thread-safe. */
   public void setData(HeightScanData data)
   {
      newData = data;
   }

   @Override
   public void computeBackground()
   {
      HeightScanData data = newData;

      if (data == null)
      {
         if (oldData != null)
         {
            clearMesh = true;
            oldData = null;
         }
         return;
      }
      if (data == oldData)
         return;

      oldData = data;

      Integer elevationOffset = data.findFieldOffset(ELEVATION_FIELD_NAME);
      int rowCount = data.getRowCount();
      int columnCount = data.getColumnCount();

      if (elevationOffset == null || rowCount < 2 || columnCount < 2)
      {
         clearMesh = true;
         return;
      }

      TriangleMesh3DDefinition definition = buildGridMeshDefinition(data, elevationOffset, rowCount, columnCount);
      newMesh = JavaFXTriangleMesh3DDefinitionInterpreter.interpretDefinition(definition, false);
   }

   private TriangleMesh3DDefinition buildGridMeshDefinition(HeightScanData data, int elevationOffset, int rowCount, int columnCount)
   {
      int vertexCount = rowCount * columnCount;
      Point3D32[] vertices = new Point3D32[vertexCount];
      Point2D32[] textures = new Point2D32[vertexCount];
      Vector3D32[] normals = new Vector3D32[vertexCount];

      Quaternion orientation = new Quaternion(data.getOrientationX(), data.getOrientationY(), data.getOrientationZ(), data.getOrientationW());
      Point3D position = new Point3D(data.getPositionX(), data.getPositionY(), data.getPositionZ());
      RigidBodyTransform gridToWorld = new RigidBodyTransform(orientation, position);

      Vector3D upNormal = new Vector3D(0.0, 0.0, 1.0);
      orientation.transform(upNormal);
      Vector3D32 upNormal32 = new Vector3D32(upNormal);

      ByteBuffer cellData = ByteBuffer.wrap(data.getData()).order(ByteOrder.LITTLE_ENDIAN);

      for (int row = 0; row < rowCount; row++)
      {
         for (int col = 0; col < columnCount; col++)
         {
            int vertexIndex = row * columnCount + col;
            int cellOffset = row * data.getRowStride() + col * data.getCellStride() + elevationOffset;
            float elevation = cellOffset + Float.BYTES <= cellData.capacity() ? cellData.getFloat(cellOffset) : 0.0f;

            // elevation is the cell's absolute world Z height (see HeightScanTerm.populateObservations()), not an
            // offset relative to the grid's pose - only X/Y go through the pose transform (the grid corner's
            // position and yaw), Z is used as-is.
            Point3D localPoint = new Point3D(col * data.getCellSizeX(), row * data.getCellSizeY(), 0.0);
            gridToWorld.transform(localPoint);
            localPoint.setZ(elevation);
            vertices[vertexIndex] = new Point3D32(localPoint);
            normals[vertexIndex] = upNormal32;

            double alpha = elevation % GRADIENT_LENGTH;
            if (alpha < 0.0)
               alpha += GRADIENT_LENGTH;
            textures[vertexIndex] = new Point2D32((float) (alpha / GRADIENT_LENGTH), 0.5f);
         }
      }

      int[] triangleIndices = new int[(rowCount - 1) * (columnCount - 1) * 6];
      int t = 0;
      for (int row = 0; row < rowCount - 1; row++)
      {
         for (int col = 0; col < columnCount - 1; col++)
         {
            int topLeft = row * columnCount + col;
            int topRight = topLeft + 1;
            int bottomLeft = topLeft + columnCount;
            int bottomRight = bottomLeft + 1;

            triangleIndices[t++] = topLeft;
            triangleIndices[t++] = bottomLeft;
            triangleIndices[t++] = topRight;

            triangleIndices[t++] = topRight;
            triangleIndices[t++] = bottomLeft;
            triangleIndices[t++] = bottomRight;
         }
      }

      return new TriangleMesh3DDefinition(vertices, textures, normals, triangleIndices);
   }

   /** A {@code samples x 1} image holding one full cycle of {@link #getRedGreenBlue(double)} - every texel is real,
    *  sequential gradient data, so there is no blank space or row-wrapping for mip-level blending to corrupt. */
   private static Image buildColormapImage(int samples)
   {
      WritableImage image = new WritableImage(samples, 1);
      PixelWriter pixelWriter = image.getPixelWriter();
      for (int i = 0; i < samples; i++)
      {
         double height = GRADIENT_LENGTH * i / samples;
         double[] rgb = getRedGreenBlue(height);
         pixelWriter.setColor(i, 0, new Color(rgb[0], rgb[1], rgb[2], 1.0));
      }
      return image;
   }

   /**
    * Returns the red, green, and blue components of a color based on a given height: a cyclic
    * magenta-blue-green-yellow-orange-magenta gradient repeating every {@link #GRADIENT_LENGTH} meters.
    */
   private static double[] getRedGreenBlue(double height)
   {
      double r, g, b;
      double magentaR = 1.0, magentaG = 0.0, magentaB = 1.0;
      double orangeR = 1.0, orangeG = 200.0 / 255.0, orangeB = 0.0;
      double yellowR = 1.0, yellowG = 1.0, yellowB = 0.0;
      double blueR = 0.0, blueG = 0.0, blueB = 1.0;
      double greenR = 0.0, greenG = 1.0, greenB = 0.0;

      double alpha = height % GRADIENT_LENGTH;
      if (alpha < 0)
         alpha = GRADIENT_LENGTH + alpha;

      if (alpha <= GRADIENT_SIZE * 1)
      {
         r = InterpolationTools.linearInterpolate(magentaR, blueR, alpha / GRADIENT_SIZE);
         g = InterpolationTools.linearInterpolate(magentaG, blueG, alpha / GRADIENT_SIZE);
         b = InterpolationTools.linearInterpolate(magentaB, blueB, alpha / GRADIENT_SIZE);
      }
      else if (alpha <= GRADIENT_SIZE * 2)
      {
         r = InterpolationTools.linearInterpolate(blueR, greenR, (alpha - GRADIENT_SIZE * 1) / GRADIENT_SIZE);
         g = InterpolationTools.linearInterpolate(blueG, greenG, (alpha - GRADIENT_SIZE * 1) / GRADIENT_SIZE);
         b = InterpolationTools.linearInterpolate(blueB, greenB, (alpha - GRADIENT_SIZE * 1) / GRADIENT_SIZE);
      }
      else if (alpha <= GRADIENT_SIZE * 3)
      {
         r = InterpolationTools.linearInterpolate(greenR, yellowR, (alpha - GRADIENT_SIZE * 2) / GRADIENT_SIZE);
         g = InterpolationTools.linearInterpolate(greenG, yellowG, (alpha - GRADIENT_SIZE * 2) / GRADIENT_SIZE);
         b = InterpolationTools.linearInterpolate(greenB, yellowB, (alpha - GRADIENT_SIZE * 2) / GRADIENT_SIZE);
      }
      else if (alpha <= GRADIENT_SIZE * 4)
      {
         r = InterpolationTools.linearInterpolate(yellowR, orangeR, (alpha - GRADIENT_SIZE * 3) / GRADIENT_SIZE);
         g = InterpolationTools.linearInterpolate(yellowG, orangeG, (alpha - GRADIENT_SIZE * 3) / GRADIENT_SIZE);
         b = InterpolationTools.linearInterpolate(yellowB, orangeB, (alpha - GRADIENT_SIZE * 3) / GRADIENT_SIZE);
      }
      else
      {
         r = InterpolationTools.linearInterpolate(orangeR, magentaR, (alpha - GRADIENT_SIZE * 4) / GRADIENT_SIZE);
         g = InterpolationTools.linearInterpolate(orangeG, magentaG, (alpha - GRADIENT_SIZE * 4) / GRADIENT_SIZE);
         b = InterpolationTools.linearInterpolate(orangeB, magentaB, (alpha - GRADIENT_SIZE * 4) / GRADIENT_SIZE);
      }

      return new double[] {r, g, b};
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
   public YoHeightGridFX3D clone()
   {
      return new YoHeightGridFX3D();
   }
}
