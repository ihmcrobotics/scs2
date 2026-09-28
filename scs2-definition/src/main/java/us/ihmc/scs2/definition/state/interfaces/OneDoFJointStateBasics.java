package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.tools.JointStateType;

public interface OneDoFJointStateBasics extends JointStateBasics, OneDoFJointStateReadOnly
{
   void setConfiguration(double q);

   void setVelocity(double qd);

   void setAcceleration(double qdd);

   void setEffort(double tau);

   /** @see OneDoFJointStateReadOnly#getStiffness() */
   void setStiffness(double stiffness);

   /** @see OneDoFJointStateReadOnly#getDamping() */
   void setDamping(double damping);

   /** @see OneDoFJointStateReadOnly#getFeedforwardEffort() */
   void setFeedforwardEffort(double tau_ff);

   /**
    * Publishes the impedance command that produced {@code tau}, alongside the effort itself. The
    * setpoints go in the configuration and velocity, which in a controller's output are what the
    * controller wants rather than what it measured.
    *
    * @see OneDoFJointStateReadOnly#getStiffness()
    */
   default void setEffortAndCommand(double tau, double tau_ff, double q_d, double qd_d, double stiffness, double damping)
   {
      setEffort(tau);
      setFeedforwardEffort(tau_ff);
      setConfiguration(q_d);
      setVelocity(qd_d);
      setStiffness(stiffness);
      setDamping(damping);
   }

   @Override
   default void clear()
   {
      setConfiguration(Double.NaN);
      setVelocity(Double.NaN);
      setAcceleration(Double.NaN);
      setEffort(Double.NaN);
      setStiffness(Double.NaN);
      setDamping(Double.NaN);
      setFeedforwardEffort(Double.NaN);
   }

   default void addConfiguration(double q)
   {
      if (!hasOutputFor(JointStateType.CONFIGURATION))
         setConfiguration(q);
      else
         setConfiguration(getConfiguration() + q);
   }

   default void addVelocity(double qd)
   {
      if (!hasOutputFor(JointStateType.VELOCITY))
         setVelocity(qd);
      else
         setVelocity(getVelocity() + qd);
   }

   default void addAcceleration(double qdd)
   {
      if (!hasOutputFor(JointStateType.ACCELERATION))
         setAcceleration(qdd);
      else
         setAcceleration(getAcceleration() + qdd);
   }

   default void addEffort(double tau)
   {
      if (!hasOutputFor(JointStateType.EFFORT))
         setEffort(tau);
      else
         setEffort(getEffort() + tau);
   }

   default void set(OneDoFJointStateReadOnly other)
   {
      setConfiguration(other.getConfiguration());
      setVelocity(other.getVelocity());
      setAcceleration(other.getAcceleration());
      setEffort(other.getEffort());
      setStiffness(other.getStiffness());
      setDamping(other.getDamping());
      setFeedforwardEffort(other.getFeedforwardEffort());
   }

   @Override
   default void setConfiguration(JointReadOnly joint)
   {
      setConfiguration(((OneDoFJointReadOnly) joint).getQ());
   }

   @Override
   default int setConfiguration(int startRow, DMatrix configuration)
   {
      setConfiguration(configuration.get(startRow++, 0));
      return startRow;
   }

   @Override
   default void setVelocity(JointReadOnly joint)
   {
      setVelocity(((OneDoFJointReadOnly) joint).getQd());
   }

   @Override
   default int setVelocity(int startRow, DMatrix velocity)
   {
      setVelocity(velocity.get(startRow++, 0));
      return startRow;
   }

   @Override
   default void setAcceleration(JointReadOnly joint)
   {
      setAcceleration(((OneDoFJointReadOnly) joint).getQdd());
   }

   @Override
   default int setAcceleration(int startRow, DMatrix acceleration)
   {
      setAcceleration(acceleration.get(startRow++, 0));
      return startRow;
   }

   @Override
   default void setEffort(JointReadOnly joint)
   {
      setEffort(((OneDoFJointReadOnly) joint).getTau());
   }

   @Override
   default int setEffort(int startRow, DMatrix effort)
   {
      setEffort(effort.get(startRow++, 0));
      return startRow;
   }
}
