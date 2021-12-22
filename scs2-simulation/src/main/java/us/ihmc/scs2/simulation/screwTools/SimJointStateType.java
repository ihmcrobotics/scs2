package us.ihmc.scs2.simulation.screwTools;

import us.ihmc.mecano.tools.JointStateType;

public enum SimJointStateType
{
   /**
    * Refers to the position and/or orientation of a joint.
    */
   CONFIGURATION,
   /**
    * Refers to the angular and/or linear velocity of a joint.
    */
   VELOCITY,
   VELOCITY_CHANGE,
   /**
    * Refers to the angular and/or linear acceleration of a joint.
    */
   ACCELERATION,
   /**
    * Refers to the moment and/or force of a joint.
    */
   EFFORT;

   public JointStateType toJointStateType()
   {
      switch (this)
      {
         case CONFIGURATION:
            return JointStateType.CONFIGURATION;
         case VELOCITY:
            return JointStateType.VELOCITY;
         case VELOCITY_CHANGE:
            return null;
         case ACCELERATION:
            return JointStateType.ACCELERATION;
         case EFFORT:
            return JointStateType.EFFORT;
         default:
            return null;
      }
   }

   public SimJointStateType fromJointStateType(JointStateType jointStateType)
   {
      switch (jointStateType)
      {
         case CONFIGURATION:
            return SimJointStateType.CONFIGURATION;
         case VELOCITY:
            return SimJointStateType.VELOCITY;
         case ACCELERATION:
            return SimJointStateType.ACCELERATION;
         case EFFORT:
            return SimJointStateType.EFFORT;
         default:
            return null;
      }
   }
}
