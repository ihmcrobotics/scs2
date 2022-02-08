package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointBasics;
import us.ihmc.mecano.tools.JointStateType;

public interface SphericalJointStateReadOnly extends JointStateReadOnly
{
   QuaternionReadOnly getOrientation();

   Vector3DReadOnly getAngularVelocity();

   Vector3DReadOnly getAngularAcceleration();

   Vector3DReadOnly getTorque();

   @Override
   default int getConfigurationSize()
   {
      return 4;
   }

   @Override
   default int getDegreesOfFreedom()
   {
      return 3;
   }

   @Override
   default boolean hasOutputFor(JointStateType query)
   {
      switch (query)
      {
         case CONFIGURATION:
            return !getOrientation().containsNaN();
         case VELOCITY:
            return !getAngularVelocity().containsNaN();
         case ACCELERATION:
            return !getAngularAcceleration().containsNaN();
         case EFFORT:
            return !getTorque().containsNaN();
         default:
            throw new IllegalStateException("Should not get here.");
      }
   }

   @Override
   default void getConfiguration(JointBasics jointToUpdate)
   {
      SphericalJointBasics sphericalJoint = (SphericalJointBasics) jointToUpdate;
      sphericalJoint.getJointOrientation().set(getOrientation());
   }

   @Override
   default int getConfiguration(int startRow, DMatrix configurationToPack)
   {
      getOrientation().get(startRow, configurationToPack);
      return startRow + getConfigurationSize();
   }

   @Override
   default void getVelocity(JointBasics jointToUpdate)
   {
      SphericalJointBasics sphericalJoint = (SphericalJointBasics) jointToUpdate;
      sphericalJoint.getJointAngularVelocity().set(getAngularVelocity());
   }

   @Override
   default int getVelocity(int startRow, DMatrix velocityToPack)
   {
      getAngularVelocity().get(startRow, velocityToPack);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   default void getAcceleration(JointBasics jointToUpdate)
   {
      SphericalJointBasics sphericalJoint = (SphericalJointBasics) jointToUpdate;
      sphericalJoint.getJointAngularAcceleration().set(getAngularAcceleration());
   }

   @Override
   default int getAcceleration(int startRow, DMatrix accelerationToPack)
   {
      getAngularAcceleration().get(startRow, accelerationToPack);
      return startRow + getDegreesOfFreedom();
   }

   @Override
   default void getEffort(JointBasics jointToUpdate)
   {
      SphericalJointBasics sphericalJoint = (SphericalJointBasics) jointToUpdate;
      sphericalJoint.getJointTorque().set(getTorque());
   }

   @Override
   default int getEffort(int startRow, DMatrix effortToPack)
   {
      getTorque().get(startRow, effortToPack);
      return startRow + getDegreesOfFreedom();
   }
}
