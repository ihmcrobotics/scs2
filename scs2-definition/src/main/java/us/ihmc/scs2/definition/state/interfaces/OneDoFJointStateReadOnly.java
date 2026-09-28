package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;

public interface OneDoFJointStateReadOnly extends JointStateReadOnly
{
   double getConfiguration();

   double getVelocity();

   double getAcceleration();

   double getEffort();

   // The low-level command: stiffness, damping and feedforwardEffort alongside the configuration
   // and velocity, which in a controller's output are setpoints rather than measurements. NaN when
   // none was published. It describes the same intent as getEffort(), which holds the whole torque
   // for an engine that can only apply one, so an engine applies the effort or evaluates the
   // command, never both. Defaulted to NaN so an implementation predating the command still
   // compiles: hasCommand() then reports false and the engine falls back to the effort.
   default double getStiffness()
   {
      return Double.NaN;
   }

   default double getDamping()
   {
      return Double.NaN;
   }

   default double getFeedforwardEffort()
   {
      return Double.NaN;
   }

   /** True when a complete command was published, so an engine may evaluate it instead of the effort. */
   default boolean hasCommand()
   {
      return hasOutputFor(JointStateType.CONFIGURATION) && hasOutputFor(JointStateType.VELOCITY) && !Double.isNaN(getStiffness())
             && !Double.isNaN(getDamping()) && !Double.isNaN(getFeedforwardEffort());
   }

   @Override
   default boolean hasOutputFor(JointStateType query)
   {
      switch (query)
      {
         case CONFIGURATION:
            return !Double.isNaN(getConfiguration());
         case VELOCITY:
            return !Double.isNaN(getVelocity());
         case ACCELERATION:
            return !Double.isNaN(getAcceleration());
         case EFFORT:
            return !Double.isNaN(getEffort());
         default:
            throw new IllegalStateException("Should not get here.");
      }
   }

   @Override
   default int getConfigurationSize()
   {
      return 1;
   }

   @Override
   default int getDegreesOfFreedom()
   {
      return 1;
   }

   @Override
   default int getConfiguration(int startRow, DMatrix configurationToPack)
   {
      configurationToPack.set(startRow, 0, getConfiguration());
      return startRow + 1;
   }

   @Override
   default int getVelocity(int startRow, DMatrix velocityToPack)
   {
      velocityToPack.set(startRow, 0, getVelocity());
      return startRow + 1;
   }

   @Override
   default int getAcceleration(int startRow, DMatrix accelerationToPack)
   {
      accelerationToPack.set(startRow, 0, getAcceleration());
      return startRow + 1;
   }

   @Override
   default int getEffort(int startRow, DMatrix effortToPack)
   {
      effortToPack.set(startRow, 0, getEffort());
      return startRow + 1;
   }

   @Override
   default void getConfiguration(JointBasics jointToUpdate)
   {
      ((OneDoFJointBasics) jointToUpdate).setQ(getConfiguration());
   }

   @Override
   default void getVelocity(JointBasics jointToUpdate)
   {
      ((OneDoFJointBasics) jointToUpdate).setQd(getVelocity());
   }

   @Override
   default void getAcceleration(JointBasics jointToUpdate)
   {
      ((OneDoFJointBasics) jointToUpdate).setQdd(getAcceleration());
   }

   @Override
   default void getEffort(JointBasics jointToUpdate)
   {
      ((OneDoFJointBasics) jointToUpdate).setTau(getEffort());
   }
}
