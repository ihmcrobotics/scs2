package us.ihmc.scs2.simulation.bullet.physicsEngine;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import org.bytedeco.bullet.LinearMath.btIDebugDraw;
import org.bytedeco.bullet.LinearMath.btVector3;
import org.bytedeco.javacpp.BytePointer;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MultiColorTriangleMesh3DBuilder;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class BulletDebugDrawingNode extends Group
{
   private int debugMode = btIDebugDraw.DBG_DrawWireframe; // TODO: Provide options to user
   private final btIDebugDraw btDebugDraw;
   private final BulletMultiBodyDynamicsWorld bulletMultiBodyDynamicsWorld;
   private final MultiColorTriangleMesh3DBuilder meshHelper = new MultiColorTriangleMesh3DBuilder();
   private PrivateAnimationTimer animationTimer;
   private int lineDraws;
   private final int maxLineDrawsPerModel = 100;
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private final YoBoolean updateDebugDrawings = new YoBoolean("updateDebugDrawings", yoRegistry);
   private final YoBoolean showDebugDrawings = new YoBoolean("showDebugDrawings", yoRegistry);
   private final Color phongColor = Color.LIGHTGRAY;
   private final Point3D fromEuclid = new Point3D();
   private final Point3D toEuclid = new Point3D();
   private final Point3D pointOnEuclid = new Point3D();

   public BulletDebugDrawingNode(BulletMultiBodyDynamicsWorld bulletMultiBodyDynamicsWorld)
   {
      this.bulletMultiBodyDynamicsWorld = bulletMultiBodyDynamicsWorld;

      updateDebugDrawings.set(true);
      showDebugDrawings.set(true);

      btDebugDraw = new btIDebugDraw()
      {
         @Override
         public void drawLine(btVector3 from, btVector3 to, btVector3 color)
         {
            if (lineDraws >= maxLineDrawsPerModel)
            {
               lineDraws = 0;
               nextModel();
            }

            BulletTools.toEuclid(from, fromEuclid);
            BulletTools.toEuclid(to, toEuclid);
            ColorDefinition colorJavaFX = new ColorDefinition(color.getX(), color.getY(), color.getZ(), 1.0);

            meshHelper.addLine(fromEuclid, toEuclid, 0.002, colorJavaFX);

            ++lineDraws;
         }

         @Override
         public void drawContactPoint(btVector3 pointOnB, btVector3 normalOnB, double distance, int lifeTime, btVector3 color)
         {
            ColorDefinition colorJavaFX = new ColorDefinition(color.getX(), color.getY(), color.getZ(), 1.0);
            BulletTools.toEuclid(pointOnB, pointOnEuclid);
            meshHelper.addSphere(0.005, pointOnEuclid, colorJavaFX);
            BulletTools.toEuclid(normalOnB, pointOnEuclid);
            meshHelper.addSphere(0.005, pointOnEuclid, colorJavaFX);
         }

         @Override
         public void drawTriangle(btVector3 v0, btVector3 v1, btVector3 v2, btVector3 color, double alpha)
         {

         }

         @Override
         public void reportErrorWarning(BytePointer warningString)
         {
            LogTools.error("Bullet: {}", warningString.getString().trim());
         }

         @Override
         public void draw3dText(btVector3 location, BytePointer textString)
         {

         }

         @Override
         public void setDebugMode(int debugMode)
         {
            BulletDebugDrawingNode.this.debugMode = debugMode;
         }

         @Override
         public int getDebugMode()
         {
            return debugMode;
         }
      };
      bulletMultiBodyDynamicsWorld.setBtDebugDrawer(btDebugDraw);
   }

   public void initializeWithJavaFX()
   {
      animationTimer = new PrivateAnimationTimer(this::update);
      animationTimer.start();
   }

   public void update(long now)
   {
      if (!showDebugDrawings.getBooleanValue())
      {
         getChildren().clear();
         return;
      }

      if (updateDebugDrawings.getBooleanValue())
      {
         getChildren().clear();
         lineDraws = 0;
         bulletMultiBodyDynamicsWorld.debugDrawWorld();
         nextModel();
      }
   }

   private void nextModel()
   {
      MeshView meshView = new MeshView(JavaFXVisualTools.toTriangleMesh(meshHelper.generateTriangleMesh3D()));
      meshView.setMaterial(new PhongMaterial(phongColor));
      getChildren().add(meshView);
      meshHelper.clear();
   }

   public YoRegistry getYoRegistry()
   {
      return yoRegistry;
   }
}
