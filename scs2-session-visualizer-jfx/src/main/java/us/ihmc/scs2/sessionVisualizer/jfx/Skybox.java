package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.value.ChangeListener;
import javafx.geometry.Point3D;
import javafx.geometry.Rectangle2D;
import javafx.scene.Camera;
import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import us.ihmc.log.LogTools;

public class Skybox extends Group
{
   public enum SkyboxTheme
   {
      CLOUDY_CROWN_MIDDAY, SCS1, CUSTOM
   };

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

   public void setupSkybox(Image skybox)
   {
      if (skybox.getWidth() / 4 != skybox.getHeight() / 3)
         throw new IllegalArgumentException("Unexpected size ratio");

      int sideSize = (int) (skybox.getWidth() - skybox.getHeight());

      int topX = sideSize;
      int bottomX = sideSize;
      int leftX = 0;
      int frontX = 3 * sideSize;
      int rightX = 2 * sideSize;
      int backX = 1 * sideSize;

      int topY = 0;
      int bottomY = 2 * sideSize;
      int leftY = sideSize;
      int rightY = sideSize;
      int frontY = sideSize;
      int backY = sideSize;
      backView.setImage(skybox);
      frontView.setImage(skybox);
      topView.setImage(skybox);
      bottomView.setImage(skybox);
      leftView.setImage(skybox);
      rightView.setImage(skybox);

      // FIXME Need to play with the viewports to get rid of the poor connection between side panels.
      topView.setViewport(new Rectangle2D(topX, topY, sideSize, sideSize));
      bottomView.setViewport(new Rectangle2D(bottomX, bottomY, sideSize, sideSize));
      leftView.setViewport(new Rectangle2D(leftX, leftY, sideSize, sideSize));
      backView.setViewport(new Rectangle2D(frontX, frontY, sideSize, sideSize));
      rightView.setViewport(new Rectangle2D(rightX, rightY, sideSize, sideSize));
      frontView.setViewport(new Rectangle2D(backX, backY, sideSize, sideSize));

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

   public boolean loadSkyboxFlexible(File file)
   {
      if (!file.exists())
      {
         LogTools.warn("File: " + file.getAbsolutePath() + " does not exist.");
         return false;
      }

      if (file.isDirectory())
      {
         File[] imageFiles = new File[6];
         for (String extension : new String[] {".png", ".jpg", ".bmp"})
         {
            if (findSkyboxImageFiles(file, extension, imageFiles, false))
               return loadSkyboxFromImageFiles(imageFiles);
         }

         LogTools.warn("Could not find skybox images in (%s).".formatted(file.getAbsolutePath()));
         return false;
      }
      else
      {
         return loadSkyboxFromSingleImageFile(file);
      }
   }

   public boolean loadSkyboxFromSingleImageFile(File file)
   {
      if (!file.exists())
      {
         LogTools.warn("File: " + file.getAbsolutePath() + " does not exist.");
         return false;
      }

      if (file.isDirectory())
      {
         LogTools.warn("Given file: " + file.getAbsolutePath() + " is a directory.");
         return false;
      }

      Image image = loadImage(file);
      if (image == null)
         return false;

      setupSkybox(image);
      return true;
   }

   public boolean loadSkyboxFromDirectory(File directory, String fileExtension)
   {
      if (!directory.exists())
      {
         LogTools.warn("Directory: " + directory.getAbsolutePath() + " does not exist.");
         return false;
      }

      if (!directory.isDirectory())
      {
         LogTools.warn("Given file: " + directory.getAbsolutePath() + " is not a directory.");
         return false;
      }

      File[] imageFiles = new File[6];
      if (!findSkyboxImageFiles(directory, fileExtension, imageFiles, true))
         return false;

      return loadSkyboxFromImageFiles(imageFiles);
   }

   public boolean loadSkyboxFromImageFiles(File[] imageFiles)
   {
      File topFile = imageFiles[0];
      File bottomFile = imageFiles[1];
      File leftFile = imageFiles[2];
      File rightFile = imageFiles[3];
      File frontFile = imageFiles[4];
      File backFile = imageFiles[5];
      return loadSkyboxFromImageFiles(topFile, bottomFile, leftFile, rightFile, frontFile, backFile);
   }

   public boolean loadSkyboxFromImageFiles(File topFile, File bottomFile, File leftFile, File rightFile, File frontFile, File backFile)
   {
      Image topImage = loadImage(topFile);
      Image bottomImage = loadImage(bottomFile);
      Image leftImage = loadImage(leftFile);
      Image rightImage = loadImage(rightFile);
      Image frontImage = loadImage(frontFile);
      Image backImage = loadImage(backFile);

      if (topImage == null || bottomImage == null || leftImage == null || rightImage == null || frontImage == null || backImage == null)
         return false;

      setupSkybox(topImage, bottomImage, leftImage, rightImage, frontImage, backImage);
      return true;
   }

   public boolean findSkyboxImageFiles(File directory, String fileExtension, File[] imageFiles, boolean printWarnings)
   {
      String topName = "Up." + fileExtension;
      String bottomName = "Down." + fileExtension;
      String leftName = "Left." + fileExtension;
      String rightName = "Right." + fileExtension;
      String frontName = "Front." + fileExtension;
      String backName = "Back." + fileExtension;
      String[] filenames = {topName, bottomName, leftName, rightName, frontName, backName};
      for (int i = 0; i < filenames.length; i++)
      {
         String filename = filenames[i];
         File[] result = directory.listFiles(name -> name.getName().equalsIgnoreCase(filename));
         if (result.length == 0)
         {
            if (printWarnings)
               LogTools.warn("Could not find the file for (%s) image of the skybox.".formatted(filename));
            return false;
         }
         imageFiles[i] = result[0];
      }
      return true;
   }

   public boolean loadSkyboxFromResource(String directoryPath, String fileExtension)
   {
      Image topImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Up." + fileExtension));
      Image bottomImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Down." + fileExtension));
      Image leftImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Left." + fileExtension));
      Image rightImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Right." + fileExtension));
      Image frontImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Front." + fileExtension));
      Image backImage = loadImage(SessionVisualizerIOTools.getSkyboxResource(directoryPath + "/Back." + fileExtension));

      if (topImage == null || bottomImage == null || leftImage == null || rightImage == null || frontImage == null || backImage == null)
         return false;

      setupSkybox(topImage, bottomImage, leftImage, rightImage, frontImage, backImage);
      return true;
   }

   private static Image loadImage(File file)
   {
      if (file == null)
         return null;
      try
      {
         return loadImage(new FileInputStream(file));
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
      return null;
   }

   private static Image loadImage(InputStream is)
   {
      if (is == null)
         return null;

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

   public void setupSCS1Skybox()
   {
      Image topImage = SessionVisualizerIOTools.SCS1_SKYBOX_TOP_IMAGE;
      Image bottomImage = SessionVisualizerIOTools.SCS1_SKYBOX_BOTTOM_IMAGE;
      Image leftImage = SessionVisualizerIOTools.SCS1_SKYBOX_LEFT_IMAGE;
      Image rightImage = SessionVisualizerIOTools.SCS1_SKYBOX_RIGHT_IMAGE;
      Image frontImage = SessionVisualizerIOTools.SCS1_SKYBOX_FRONT_IMAGE;
      Image backImage = SessionVisualizerIOTools.SCS1_SKYBOX_BACK_IMAGE;
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
