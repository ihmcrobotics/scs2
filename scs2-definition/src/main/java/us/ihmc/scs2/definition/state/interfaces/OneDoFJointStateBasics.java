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
      clear();
      if (other.hasOutputFor(JointStateType.CONFIGURATION))
         setConfiguration(other.getConfiguration());
      if (other.hasOutputFor(JointStateType.VELOCITY))
         setVelocity(other.getVelocity());
      if (other.hasOutputFor(JointStateType.ACCELERATION))
         setAcceleration(other.getAcceleration());
      if (other.hasOutputFor(JointStateType.EFFORT))
         setEffort(other.getEffort());
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
