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
      if (joint.getConfigurationMatrixSize() != getConfigurationSize())
         throw new IllegalArgumentException("Bad configuration size: expected = " + getConfigurationSize() + ", was = " + joint.getConfigurationMatrixSize());
   }

   default void checkConfigurationSize(int startRow, DMatrix configurationMatrix)
   {
      if (configurationMatrix.getNumRows() - startRow < getConfigurationSize())
         throw new IllegalArgumentException("Bad configuration size: expected >= " + getConfigurationSize() + ", was = " + configurationMatrix.getNumRows());
   }

   default void checkDegreesOfFreedom(JointReadOnly joint)
   {
      if (joint.getDegreesOfFreedom() != getDegreesOfFreedom())
         throw new IllegalArgumentException("Bad number of DoFs: expected = " + getDegreesOfFreedom() + ", was = " + joint.getDegreesOfFreedom());
   }

   default void checkDegreesOfFreedom(int startRow, DMatrix degreesOfFreedomMatrix)
   {
      if (degreesOfFreedomMatrix.getNumRows() - startRow < getDegreesOfFreedom())
         throw new IllegalArgumentException("Bad number of DoFs: expected >= " + getDegreesOfFreedom() + ", was = " + degreesOfFreedomMatrix.getNumRows());
   }
}