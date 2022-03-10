package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;
import com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class BulletDebugDrawingNode extends Group
{
   private final com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw btIDebugDraw;
   private int debugMode = com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw.DebugDrawModes.DBG_DrawWireframe; // TODO: Provide options in combo box
   private final btMultiBodyDynamicsWorld multiBodyDynamicsWorld;
   private final JavaFXMultiColorMeshBuilder meshHelper = new JavaFXMultiColorMeshBuilder();
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

   public BulletDebugDrawingNode(btMultiBodyDynamicsWorld multiBodyDynamicsWorld)
   {
      this.multiBodyDynamicsWorld = multiBodyDynamicsWorld;

      updateDebugDrawings.set(true);
      showDebugDrawings.set(true);

      btIDebugDraw = new btIDebugDraw()
      {
         @Override
         public void drawLine(Vector3 from, Vector3 to, Vector3 color)
         {
            if (lineDraws >= maxLineDrawsPerModel)
            {
               lineDraws = 0;
               nextModel();
            }

            BulletTools.toEuclid(from, fromEuclid);
            BulletTools.toEuclid(to, toEuclid);
            Color colorJavaFX = new Color(color.x, color.y, color.z, 1.0);

            meshHelper.addLine(fromEuclid, toEuclid, 0.002, colorJavaFX);

            ++lineDraws;
         }

         @Override
         public void drawContactPoint(Vector3 pointOnB, Vector3 normalOnB, float distance, int lifeTime, Vector3 color)
         {
            Color colorJavaFX = new Color(color.x, color.y, color.z, 1.0);
            BulletTools.toEuclid(pointOnB, pointOnEuclid);
            meshHelper.addSphere(0.005, pointOnEuclid, colorJavaFX);
            BulletTools.toEuclid(normalOnB, pointOnEuclid);
            meshHelper.addSphere(0.005, pointOnEuclid, colorJavaFX);
         }

         @Override
         public void drawTriangle(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 color, float alpha)
         {

         }

         @Override
         public void reportErrorWarning(String warningString)
         {
            LogTools.error("Bullet: {}", warningString);
         }

         @Override
         public void draw3dText(Vector3 location, String textString)
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
      multiBodyDynamicsWorld.setDebugDrawer(btIDebugDraw);
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
         multiBodyDynamicsWorld.debugDrawWorld();
         nextModel();
      }
   }

   private void nextModel()
   {
      MeshView meshView = new MeshView(meshHelper.generateMesh());
      meshView.setMaterial(new PhongMaterial(phongColor));
      getChildren().add(meshView);
      meshHelper.clear();
   }

   public YoRegistry getYoRegistry()
   {
      return yoRegistry;
   }
}
