package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.tools.JointStateType;

public interface JointStateReadOnly
{
   int getConfigurationSize();

   int getDegreesOfFreedom();

   boolean hasOutputFor(JointStateType query);

   void getConfiguration(JointBasics jointToUpdate);

   int getConfiguration(int startRow, DMatrix configurationToPack);

   void getVelocity(JointBasics jointToUpdate);

   int getVelocity(int startRow, DMatrix velocityToPack);

   void getAcceleration(JointBasics jointToUpdate);

   int getAcceleration(int startRow, DMatrix accelerationToPack);

   void getEffort(JointBasics jointToUpdate);

   int getEffort(int startRow, DMatrix effortToPack);

   default void getAllStates(JointBasics jointToUpdate)
   {
      if (hasOutputFor(JointStateType.CONFIGURATION))
         getConfiguration(jointToUpdate);
      if (hasOutputFor(JointStateType.VELOCITY))
         getVelocity(jointToUpdate);
      if (hasOutputFor(JointStateType.ACCELERATION))
         getAcceleration(jointToUpdate);
      if (hasOutputFor(JointStateType.EFFORT))
         getEffort(jointToUpdate);
   }

   default void checkConfigurationSize(JointReadOnly joint)
   {
      checkConfigurationSize(joint.getConfigurationMatrixSize());
   }

   default void checkConfigurationSize(DMatrix configurationMatrix)
   {
      checkConfigurationSize(configurationMatrix.getNumRows());
   }

   default void checkConfigurationSize(int query)
   {
      if (query != getConfigurationSize())
         throw new IllegalArgumentException("Bad configuration size: expected = " + getConfigurationSize() + ", was = " + query);
   }

   default void checkDegreesOfFreedom(JointReadOnly joint)
   {
      checkDegreesOfFreedom(joint.getDegreesOfFreedom());
   }

   default void checkDegreesOfFreedom(DMatrix degreesOfFreedomMatrix)
   {
      checkDegreesOfFreedom(degreesOfFreedomMatrix.getNumRows());
   }

   default void checkDegreesOfFreedom(int query)
   {
      if (query != getDegreesOfFreedom())
         throw new IllegalArgumentException("Bad number of DoFs: expected = " + getDegreesOfFreedom() + ", was = " + query);
   }
}