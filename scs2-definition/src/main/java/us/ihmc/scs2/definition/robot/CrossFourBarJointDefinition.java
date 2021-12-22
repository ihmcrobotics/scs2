package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;

public class CrossFourBarJointDefinition extends OneDoFJointDefinition
{
   private String jointNameA;
   private String jointNameB;
   private String jointNameC;
   private String jointNameD;
   private YawPitchRollTransformDefinition transformAToPredecessor = new YawPitchRollTransformDefinition();
   private YawPitchRollTransformDefinition transformBToPredecessor = new YawPitchRollTransformDefinition();
   private YawPitchRollTransformDefinition transformCToB = new YawPitchRollTransformDefinition();
   private YawPitchRollTransformDefinition transformDToA = new YawPitchRollTransformDefinition();

   private RigidBodyDefinition bodyDA = new RigidBodyDefinition();
   private RigidBodyDefinition bodyBC = new RigidBodyDefinition();

   private int actuatedJointIndex;
   private int loopClosureJointIndex;

   public CrossFourBarJointDefinition(String name)
   {
      super(name);
      bodyDA.setName(name + "_DA");
      bodyBC.setName(name + "_BC");
   }

   public CrossFourBarJointDefinition(String name, Vector3DReadOnly axis)
   {
      this(name);
      setAxis(axis);
   }

   public void setJointNameA(String jointNameA)
   {
      this.jointNameA = jointNameA;
   }

   public void setJointNameB(String jointNameB)
   {
      this.jointNameB = jointNameB;
   }

   public void setJointNameC(String jointNameC)
   {
      this.jointNameC = jointNameC;
   }

   public void setJointNameD(String jointNameD)
   {
      this.jointNameD = jointNameD;
   }

   public void setJointNames(String jointNameA, String jointNameB, String jointNameC, String jointNameD)
   {
      this.jointNameA = jointNameA;
      this.jointNameB = jointNameB;
      this.jointNameC = jointNameC;
      this.jointNameD = jointNameD;
   }

   public void setTransformAToPredecessor(YawPitchRollTransformDefinition transformAToPredecessor)
   {
      this.transformAToPredecessor = transformAToPredecessor;
   }

   public void setTransformAToPredecessor(RigidBodyTransformReadOnly transformAToPredecessor)
   {
      this.transformAToPredecessor.set(transformAToPredecessor);
   }

   public void setTransformBToPredecessor(YawPitchRollTransformDefinition transformBToPredecessor)
   {
      this.transformBToPredecessor = transformBToPredecessor;
   }

   public void setTransformBToPredecessor(RigidBodyTransformReadOnly transformBToPredecessor)
   {
      this.transformBToPredecessor.set(transformBToPredecessor);
   }

   public void setTransformCToB(YawPitchRollTransformDefinition transformCToB)
   {
      this.transformCToB = transformCToB;
   }

   public void setTransformCToB(RigidBodyTransformReadOnly transformCToB)
   {
      this.transformCToB.set(transformCToB);
   }

   public void setTransformDToA(YawPitchRollTransformDefinition transformDToA)
   {
      this.transformDToA = transformDToA;
   }

   public void setTransformDToA(RigidBodyTransformReadOnly transformDToA)
   {
      this.transformDToA.set(transformDToA);
   }

   public void setJointTransforms(RigidBodyTransformReadOnly transformAToPredecessor,
                                  RigidBodyTransformReadOnly transformBToPredecessor,
                                  RigidBodyTransformReadOnly transformCToB,
                                  RigidBodyTransformReadOnly transformDToA)
   {
      this.transformAToPredecessor.set(transformAToPredecessor);
      this.transformBToPredecessor.set(transformBToPredecessor);
      this.transformCToB.set(transformCToB);
      this.transformDToA.set(transformDToA);
   }

   public void setBodyDA(RigidBodyDefinition bodyDA)
   {
      this.bodyDA = bodyDA;
   }

   public void setBodyBC(RigidBodyDefinition bodyBC)
   {
      this.bodyBC = bodyBC;
   }

   public void setActuatedJointIndex(int actuatedJointIndex)
   {
      this.actuatedJointIndex = actuatedJointIndex;
   }

   public void setLoopClosureJointIndex(int loopClosureJointIndex)
   {
      this.loopClosureJointIndex = loopClosureJointIndex;
   }

   public String getJointNameA()
   {
      return jointNameA;
   }

   public String getJointNameB()
   {
      return jointNameB;
   }

   public String getJointNameC()
   {
      return jointNameC;
   }

   public String getJointNameD()
   {
      return jointNameD;
   }

   public YawPitchRollTransformDefinition getTransformAToPredecessor()
   {
      return transformAToPredecessor;
   }

   public YawPitchRollTransformDefinition getTransformBToPredecessor()
   {
      return transformBToPredecessor;
   }

   public YawPitchRollTransformDefinition getTransformCToB()
   {
      return transformCToB;
   }

   public YawPitchRollTransformDefinition getTransformDToA()
   {
      return transformDToA;
   }

   public RigidBodyDefinition getBodyDA()
   {
      return bodyDA;
   }

   public RigidBodyDefinition getBodyBC()
   {
      return bodyBC;
   }

   public int getActuatedJointIndex()
   {
      return actuatedJointIndex;
   }

   public int getLoopClosureJointIndex()
   {
      return loopClosureJointIndex;
   }

   @Override
   public CrossFourBarJoint toJoint(RigidBodyBasics predecessor)
   {
      return new CrossFourBarJoint(getName(),
                                   predecessor,
                                   jointNameA,
                                   jointNameB,
                                   jointNameC,
                                   jointNameD,
                                   bodyDA.getName(),
                                   bodyBC.getName(),
                                   transformAToPredecessor,
                                   transformBToPredecessor,
                                   transformDToA,
                                   transformCToB,
                                   bodyDA.getMomentOfInertia(),
                                   bodyBC.getMomentOfInertia(),
                                   bodyDA.getMass(),
                                   bodyBC.getMass(),
                                   bodyDA.getInertiaPose(),
                                   bodyBC.getInertiaPose(),
                                   actuatedJointIndex,
                                   loopClosureJointIndex,
                                   getAxis());
   }

   @Override
   public CrossFourBarJointDefinition copy()
   {
      CrossFourBarJointDefinition clone = new CrossFourBarJointDefinition(getName(), getAxis());
      clone.jointNameA = jointNameA;
      clone.jointNameB = jointNameB;
      clone.jointNameC = jointNameC;
      clone.jointNameD = jointNameD;
      clone.transformAToPredecessor.set(transformAToPredecessor);
      clone.transformBToPredecessor.set(transformBToPredecessor);
      clone.transformCToB.set(transformCToB);
      clone.transformDToA.set(transformDToA);
      clone.bodyDA = bodyDA.copy();
      clone.bodyBC = bodyBC.copy();
      clone.actuatedJointIndex = actuatedJointIndex;
      clone.loopClosureJointIndex = loopClosureJointIndex;
      return clone;
   }
}
