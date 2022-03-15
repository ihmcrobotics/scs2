package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.physics.bullet.collision.btPersistentManifold;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.sensors.SimWrenchSensor;

public class BulletWrenchSensorCalculator
{
   private final SimRevoluteJoint simRevoluteJoint;
   private final SimWrenchSensor wrenchSensor;

   public BulletWrenchSensorCalculator(SimRevoluteJoint simRevoluteJoint, SimWrenchSensor wrenchSensor)
   {
      this.simRevoluteJoint = simRevoluteJoint;
      this.wrenchSensor = wrenchSensor;
   }

   public void handleContact(btPersistentManifold contactManifold)
   {
      // contact on the link after joint is B

      int numContacts = contactManifold.getNumContacts();
      if (numContacts == 0)
      {
         wrenchSensor.getWrench().setToZero();
      }
      else
      {
         wrenchSensor.getWrench().setToZero();
         wrenchSensor.getWrench().setLinearPartZ(20.0);
      }
   }
}
