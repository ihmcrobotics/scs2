package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;
import com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw;
import javafx.scene.Group;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class BulletDebugDrawingNode extends Group
{
   private final com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw btIDebugDraw;
   private int debugMode = com.badlogic.gdx.physics.bullet.linearmath.btIDebugDraw.DebugDrawModes.DBG_DrawWireframe; // TODO: Provide options in combo box
   private final btMultiBodyDynamicsWorld multiBodyDynamicsWorld;
   private final Timer autoDisableTimer = new Timer();
   private PrivateAnimationTimer animationTimer;
   private int lineDraws;
   private final int maxLineDrawsPerModel = 100;
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private final YoBoolean updateDebugDrawings = new YoBoolean("updateDebugDrawings", yoRegistry);
   private final YoBoolean showDebugDrawings = new YoBoolean("showDebugDrawings", yoRegistry);

   public BulletDebugDrawingNode(btMultiBodyDynamicsWorld multiBodyDynamicsWorld)
   {
      this.multiBodyDynamicsWorld = multiBodyDynamicsWorld;

      btIDebugDraw = new btIDebugDraw()
      {
         @Override
         public void drawLine(Vector3 from, Vector3 to, Vector3 color)
         {
            if (lineDraws >= maxLineDrawsPerModel)
            {
               lineDraws = 0;
//               currentModel.end();
               nextModel();
            }

//            currentModel.addLine(from, to, color);

            ++lineDraws;
         }

         @Override
         public void drawContactPoint(Vector3 PointOnB, Vector3 normalOnB, float distance, int lifeTime, Vector3 color)
         {

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
      if (autoDisableTimer.isExpired(3.0))
      {
         updateDebugDrawings.set(false);
      }

      if (updateDebugDrawings.getBooleanValue())
      {
//         models.clear();
         lineDraws = 0;
         nextModel();
         multiBodyDynamicsWorld.debugDrawWorld();
//         currentModel.end();
      }
   }

   private void nextModel()
   {
//      currentModel = models.add();
//      currentModel.begin();
   }

   public YoRegistry getYoRegistry()
   {
      return yoRegistry;
   }
}
