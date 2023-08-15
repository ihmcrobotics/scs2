package us.ihmc.scs2.sessionVisualizer.jfx.camera;

import org.junit.jupiter.api.Test;

import javafx.scene.PerspectiveCamera;
import javafx.scene.transform.Transform;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.PerspectiveCameraController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TranslateSCS2;

public class CameraControlTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testCameraControls()
   {
      PerspectiveCamera camera = new PerspectiveCamera();
      camera = new PerspectiveCamera(true);
      camera.setNearClip(0.05);
      camera.setFarClip(2.0e5);
      PerspectiveCameraController controller = new PerspectiveCameraController(null, null, camera, Axis3D.Z, Axis3D.X);

      Point3D expectedFocalPoint = new Point3D();
      Point3D expectedCameraPosition = new Point3D(-1, 0, 0);
      controller.setFocalPoint(expectedFocalPoint, true);
      controller.setCameraPosition(expectedCameraPosition, false);

      Vector3D expectedCameraLookDirection = new Vector3D(1, 0, 0);
      Vector3D expectedCameraRightDirection = new Vector3D(0, -1, 0);

      EuclidCoreTestTools.assertEquals(expectedFocalPoint, controller.getFocalPointTranslate(), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraPosition, cameraPosition(camera), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraLookDirection, cameraLookDirection(camera), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraRightDirection, cameraRightDirection(camera), EPSILON);

      controller.setCameraPosition(Double.NaN, Double.NaN, Double.NaN, false);
      EuclidCoreTestTools.assertEquals(expectedFocalPoint, controller.getFocalPointTranslate(), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraPosition, cameraPosition(camera), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraLookDirection, cameraLookDirection(camera), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraRightDirection, cameraRightDirection(camera), EPSILON);

      expectedFocalPoint.add(0, 0, 1.0);
      controller.setFocalPoint(expectedFocalPoint, false);

      EuclidCoreTestTools.assertEquals(expectedFocalPoint, controller.getFocalPointTranslate(), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraPosition, cameraPosition(camera), EPSILON);
      expectedCameraLookDirection.sub(expectedFocalPoint, expectedCameraPosition);
      expectedCameraLookDirection.normalize();
      EuclidCoreTestTools.assertEquals(expectedCameraLookDirection, cameraLookDirection(camera), EPSILON);

      expectedFocalPoint.add(0, 0, 1);
      expectedCameraRightDirection = cameraRightDirection(camera);
      expectedCameraPosition.add(0, 0, 1);
      controller.setCameraPosition(expectedCameraPosition, true);

      EuclidCoreTestTools.assertEquals(expectedFocalPoint, controller.getFocalPointTranslate(), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraPosition, cameraPosition(camera), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraLookDirection, cameraLookDirection(camera), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraRightDirection, cameraRightDirection(camera), EPSILON);

      expectedFocalPoint = new Point3D();
      expectedCameraPosition = new Point3D(-1, 0, 0);
      controller.setFocalPoint(expectedFocalPoint, true);
      controller.setCameraPosition(expectedCameraPosition, false);

      expectedCameraLookDirection = new Vector3D(1, 0, 0);
      expectedCameraRightDirection = new Vector3D(0, -1, 0);

      EuclidCoreTestTools.assertEquals(expectedFocalPoint, controller.getFocalPointTranslate(), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraPosition, cameraPosition(camera), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraLookDirection, cameraLookDirection(camera), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraRightDirection, cameraRightDirection(camera), EPSILON);

      expectedCameraPosition = new Point3D(-2, 0, 0);
      controller.setCameraOrbit(2, Double.NaN, Double.NaN, Double.NaN, false);
      EuclidCoreTestTools.assertEquals(expectedFocalPoint, controller.getFocalPointTranslate(), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraPosition, cameraPosition(camera), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraLookDirection, cameraLookDirection(camera), EPSILON);
      EuclidCoreTestTools.assertEquals(expectedCameraRightDirection, cameraRightDirection(camera), EPSILON);
   }

   public TranslateSCS2 cameraPosition(PerspectiveCamera camera)
   {
      return new TranslateSCS2(camera.getLocalToSceneTransform());
   }

   public static Vector3D cameraLookDirection(PerspectiveCamera camera)
   {
      Transform cameraTransform = camera.getLocalToSceneTransform();
      return new Vector3D(cameraTransform.getMxz(), cameraTransform.getMyz(), cameraTransform.getMzz());
   }

   public static Vector3D cameraRightDirection(PerspectiveCamera camera)
   {
      Transform cameraTransform = camera.getLocalToSceneTransform();
      return new Vector3D(cameraTransform.getMxx(), cameraTransform.getMyx(), cameraTransform.getMzx());
   }

}
