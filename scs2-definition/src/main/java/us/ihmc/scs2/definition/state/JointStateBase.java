package us.ihmc.scs2.definition.state;

import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;

public abstract class JointStateBase implements JointStateBasics
{
   @Override
   public abstract JointStateBase copy();
}
