package us.ihmc.scs2.definition.state.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;

public interface JointStateBasics extends JointStateReadOnly
{
   void clear();

   void set(JointStateReadOnly other);

   int setConfiguration(int startRow, DMatrix configuration);

   void setConfiguration(JointReadOnly joint);

   int setVelocity(int startRow, DMatrix velocity);

   void setVelocity(JointReadOnly joint);

   int setAcceleration(int startRow, DMatrix acceleration);

   void setAcceleration(JointReadOnly joint);

   int setEffort(int startRow, DMatrix effort);

   void setEffort(JointReadOnly joint);

   JointStateBasics copy();
}