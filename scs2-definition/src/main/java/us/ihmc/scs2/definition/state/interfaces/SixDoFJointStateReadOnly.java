package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;

public interface SixDoFJointStateReadOnly extends JointStateReadOnly
{
   /**
    * The 3D orientation of the joint local frame.
    * 
    * @return the joint orientation.
    */
   QuaternionReadOnly getOrientation();

   /**
    * The 3D position of the joint local frame.
    * 
    * @return the joint position.
    */
   Point3DReadOnly getPosition();

   /**
    * The 3D angular velocity expressed in the local frame of the joint, i.e. the coordinate system
    * defined by {@link #getOrientation()}.
    * 
    * @return the joint angular velocity.
    */
   Vector3DReadOnly getAngularVelocity();

   /**
    * The 3D linear velocity expressed in the local frame of the joint, i.e. the coordinate system
    * defined by {@link #getOrientation()}.
    * 
    * @return the joint linear velocity.
    */
   Vector3DReadOnly getLinearVelocity();

   /**
    * The 3D angular acceleration expressed in the local frame of the joint, i.e. the coordinate system
    * defined by {@link #getOrientation()}.
    * 
    * @return the joint angular acceleration.
    */
   Vector3DReadOnly getAngularAcceleration();

   /**
    * The 3D linear acceleration expressed in the local frame of the joint, i.e. the coordinate system
    * defined by {@link #getOrientation()}.
    * 
    * @return the joint linear acceleration.
    */
   Vector3DReadOnly getLinearAcceleration();

   /**
    * The 3D torque expressed in the local frame of the joint, i.e. the coordinate system defined by
    * {@link #getOrientation()}.
    * 
    * @return the joint torque.
    */
   Vector3DReadOnly getTorque();

   /**
    * The 3D force expressed in the local frame of the joint, i.e. the coordinate system defined by
    * {@link #getOrientation()}.
    * 
    * @return the joint force.
    */
   Vector3DReadOnly getForce();

   @Override
   default int getConfigurationSize()
   {
      return 7;
   }

   @Override
   default int getDegreesOfFreedom()
   {
      return 6;
   }

   @Override
   default boolean hasOutputFor(JointStateType query)
   {
      switch (query)
      {
         case CONFIGURATION:
            return !getOrientation().containsNaN() && !getPosition().containsNaN();
         case VELOCITY:
            return !getAngularVelocity().containsNaN() && !getLinearVelocity().containsNaN();
         case ACCELERATION:
            return !getAngularAcceleration().containsNaN() && !getLinearAcceleration().containsNaN();
         case EFFORT:
            return !getTorque().containsNaN() && !getForce().containsNaN();
         default:
            throw new IllegalStateException("Should not get here.");
      }
   }

   @Override
   default int getConfiguration(int startRow, DMatrix configurationToPack)
   {
      getOrientation().get(startRow, configurationToPack);
      startRow += 4;
      getPosition().get(startRow, configurationToPack);
      return startRow + 3;
   }

   @Override
   default int getVelocity(int startRow, DMatrix velocityToPack)
   {
      getAngularVelocity().get(startRow, velocityToPack);
      startRow += 3;
      getLinearVelocity().get(startRow, velocityToPack);
      return startRow + 3;
   }

   @Override
   default int getAcceleration(int startRow, DMatrix accelerationToPack)
   {
      getAngularAcceleration().get(startRow, accelerationToPack);
      startRow += 3;
      getLinearAcceleration().get(startRow, accelerationToPack);
      return startRow + 3;
   }

   @Override
   default int getEffort(int startRow, DMatrix effortToPack)
   {
      getTorque().get(startRow, effortToPack);
      startRow += 3;
      getForce().get(startRow, effortToPack);
      return startRow + 3;
   }

   @Override
   default void getConfiguration(JointBasics jointToUpdate)
   {
      ((SixDoFJointBasics) jointToUpdate).setJointConfiguration(getOrientation(), getPosition());
   }

   @Override
   default void getVelocity(JointBasics jointToUpdate)
   {
      ((SixDoFJointBasics) jointToUpdate).getJointTwist().set(getAngularVelocity(), getLinearVelocity());
   }

   @Override
   default void getAcceleration(JointBasics jointToUpdate)
   {
      ((SixDoFJointBasics) jointToUpdate).getJointAcceleration().set(getAngularAcceleration(), getLinearAcceleration());
   }

   @Override
   default void getEffort(JointBasics jointToUpdate)
   {
      ((SixDoFJointBasics) jointToUpdate).getJointWrench().set(getTorque(), getForce());
   }
}
