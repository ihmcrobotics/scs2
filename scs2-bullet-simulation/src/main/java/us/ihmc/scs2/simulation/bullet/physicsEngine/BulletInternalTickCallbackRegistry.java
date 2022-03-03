package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.physics.bullet.dynamics.InternalTickCallback;
import com.badlogic.gdx.physics.bullet.dynamics.btDynamicsWorld;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;

import java.util.ArrayList;

public class BulletInternalTickCallbackRegistry
{
   private final ArrayList<Runnable> postTickRunnables = new ArrayList<>();

   public BulletInternalTickCallbackRegistry(btMultiBodyDynamicsWorld multiBodyDynamicsWorld)
   {
      // Note: Apparently you can't have both pre and post tick callbacks, so we'll just do with post
      new InternalTickCallback(multiBodyDynamicsWorld, false)
      {
         @Override
         public void onInternalTick(btDynamicsWorld dynamicsWorld, float timeStep)
         {
            super.onInternalTick(dynamicsWorld, timeStep);
            for (Runnable postTickRunnable : postTickRunnables)
            {
               postTickRunnable.run();
            }
         }
      };
   }

   public ArrayList<Runnable> getPostTickRunnables()
   {
      return postTickRunnables;
   }
}
