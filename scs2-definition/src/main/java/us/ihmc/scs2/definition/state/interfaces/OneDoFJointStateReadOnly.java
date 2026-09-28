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

   /**
    * Position feedback gain the controller wants the joint driven with, or NaN when none was
    * published.
    *
    * <p>Together with {@link #getDamping()}, {@link #getFeedforwardEffort()} and the configuration
    * and velocity above -- which in a controller's output are setpoints, not measurements -- this
    * completes the low-level command, so an engine that models actuators can close the impedance
    * loop itself instead of applying a torque computed once per controller tick.
    *
    * <p>The command and {@link #getEffort()} are published together and describe the same intent:
    * {@code effort == feedforwardEffort + stiffness * (configuration - q) + damping * (velocity -
    * qd)} at the state the writer sampled, clamped to the joint's effort limits. An engine either
    * applies the effort or evaluates the command, never both.
    *
    * @see #hasCommand()
    */
   double getStiffness();

   /** Velocity feedback gain the controller wants the joint driven with, or NaN. @see #getStiffness() */
   double getDamping();

   /**
    * The effort the controller asked for before feedback, or NaN when none was published. Only this
    * term is separate from the joint state proper: the setpoints are the configuration and velocity,
    * but the effort has to stay the whole torque for an engine that can only apply one.
    *
    * @see #getStiffness()
    */
   double getFeedforwardEffort();

   /**
    * True when a complete low-level command was published, so an engine may evaluate it in place of
    * {@link #getEffort()}.
    */
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
