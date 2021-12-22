package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.IOException;
import java.io.InputStream;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.value.ChangeListener;
import javafx.geometry.Point3D;
import javafx.scene.Camera;
import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;

public class Skybox extends Group
{
   private final Translate translate = new Translate();
   private final DoubleProperty size = new SimpleDoubleProperty(this, "size", 100000.0);

   private final Affine backAffine = new Affine();
   private final Affine frontAffine = new Affine();
   private final Affine topAffine = new Affine();
   private final Affine bottomAffine = new Affine();
   private final Affine leftAffine = new Affine();
   private final Affine rightAffine = new Affine();

   private final ImageView topView = new ImageView();
   private final ImageView bottomView = new ImageView();
   private final ImageView leftView = new ImageView();
   private final ImageView rightView = new ImageView();
   private final ImageView backView = new ImageView();
   private final ImageView frontView = new ImageView();
   private final ImageView[] views = new ImageView[] {topView, leftView, backView, rightView, frontView, bottomView};

   private Camera camera;
   private ChangeListener<Transform> cameraMotionListener = (observable, oldValue, newValue) ->
   {
      translate.setX(newValue.getTx());
      translate.setY(newValue.getTy());
      translate.setZ(newValue.getTz());
   };;

   public Skybox()
   {
      for (ImageView view : views)
      {
         view.setSmooth(true);
         view.setPreserveRatio(true);
         view.fitWidthProperty().bind(size);
         view.fitHeightProperty().bind(size);
      }

      updateTransforms(size.get());

      backView.getTransforms().addAll(backAffine);
      frontView.getTransforms().addAll(frontAffine);
      topView.getTransforms().addAll(topAffine);
      bottomView.getTransforms().addAll(bottomAffine);
      leftView.getTransforms().addAll(leftAffine);
      rightView.getTransforms().addAll(rightAffine);

      getTransforms().add(translate);
      getChildren().addAll(views);
      setMouseTransparent(true);
   }

   private void updateTransforms(double size)
   {
      backAffine.setToIdentity();
      backAffine.appendTranslation(-0.5 * size, -0.5 * size, 0.5 * size);
      backAffine.appendRotation(+90.0, Point3D.ZERO, Rotate.Z_AXIS);
      backAffine.appendRotation(-90.0, Point3D.ZERO, Rotate.X_AXIS);

      frontAffine.setToIdentity();
      frontAffine.appendTranslation(0.5 * size, 0.5 * size, 0.5 * size);
      frontAffine.appendRotation(-90.0, Point3D.ZERO, Rotate.Z_AXIS);
      frontAffine.appendRotation(-90.0, Point3D.ZERO, Rotate.X_AXIS);

      topAffine.setToIdentity();
      topAffine.appendTranslation(-0.5 * size, 0.5 * size, 0.5 * size);
      topAffine.appendRotation(-90.0, Point3D.ZERO, Rotate.Z_AXIS);

      bottomAffine.setToIdentity();
      bottomAffine.appendTranslation(0.5 * size, 0.5 * size, -0.5 * size);
      bottomAffine.appendRotation(180.0, Point3D.ZERO, Rotate.X_AXIS);
      bottomAffine.appendRotation(+90.0, Point3D.ZERO, Rotate.Z_AXIS);

      leftAffine.setToIdentity();
      leftAffine.appendTranslation(-0.5 * size, 0.5 * size, 0.5 * size);
      leftAffine.appendRotation(-90.0, Point3D.ZERO, Rotate.X_AXIS);

      rightAffine.setToIdentity();
      rightAffine.appendTranslation(0.5 * size, -0.5 * size, 0.5 * size);
      rightAffine.appendRotation(180.0, Point3D.ZERO, Rotate.Z_AXIS);
      rightAffine.appendRotation(-90.0, Point3D.ZERO, Rotate.X_AXIS);
   }

   public void setupSkybox(Image topImage, Image bottomImage, Image leftImage, Image rightImage, Image frontImage, Image backImage)
   {
      backView.setImage(backImage);
      frontView.setImage(frontImage);
      topView.setImage(topImage);
      bottomView.setImage(bottomImage);
      leftView.setImage(leftImage);
      rightView.setImage(rightImage);
   }

   public void setupSkybox(String directoryPath, String fileExtension)
   {
      Image topImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Up." + fileExtension));
      Image bottomImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Down." + fileExtension));
      Image leftImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Left." + fileExtension));
      Image rightImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Right." + fileExtension));
      Image frontImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Front." + fileExtension));
      Image backImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Back." + fileExtension));
      setupSkybox(topImage, bottomImage, leftImage, rightImage, frontImage, backImage);
   }

   private static Image loadImage(InputStream is)
   {
      Image image = new Image(is);
      try
      {
         is.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      return image;
   }

   public void setupCloudyCrown()
   {
      Image topImage = SessionVisualizerIOTools.SKYBOX_TOP_IMAGE;
      Image bottomImage = SessionVisualizerIOTools.SKYBOX_BOTTOM_IMAGE;
      Image leftImage = SessionVisualizerIOTools.SKYBOX_LEFT_IMAGE;
      Image rightImage = SessionVisualizerIOTools.SKYBOX_RIGHT_IMAGE;
      Image frontImage = SessionVisualizerIOTools.SKYBOX_FRONT_IMAGE;
      Image backImage = SessionVisualizerIOTools.SKYBOX_BACK_IMAGE;
      setupSkybox(topImage, bottomImage, leftImage, rightImage, frontImage, backImage);
   }

   public void setupCamera(Camera camera)
   {
      if (this.camera != null)
         this.camera.localToSceneTransformProperty().removeListener(cameraMotionListener);
      this.camera = camera;
      if (camera != null)
         camera.localToSceneTransformProperty().addListener(cameraMotionListener);
   }

   public void setSize(double size)
   {
      this.size.set(size);
   }

   public double getSize()
   {
      return size.get();
   }

   public DoubleProperty sizeProperty()
   {
      return size;
   }

   public void dispose()
   {
      setupCamera(null);

      for (ImageView view : views)
      {
         view.setImage(null);
      }
   }
}
