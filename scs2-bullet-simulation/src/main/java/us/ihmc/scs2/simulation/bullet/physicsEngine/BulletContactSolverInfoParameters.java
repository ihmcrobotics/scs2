package us.ihmc.scs2.simulation.bullet.physicsEngine;

public class BulletContactSolverInfoParameters
{
   private double tau;                                               //m_tau
   private double damping;                                           //m_damping - global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
   private double friction;                                          //m_friction
   private double timeStep;                                          //m_timeStep
   private double restitution;                                       //m_restitution
   private int numberOfIterations;                                   //m_numIterations
   private double maxErrorReduction;                                 //m_maxErrorReduction
   private double successiveOverRelaxationTerm;                      //m_sor
   private double errorReductionForNonContactConstraints;            //m_erp
   private double errorReductionForContactConstraints;               //m_erp2
   private double constraintForceMixingForContactsAndNonContacts;    //m_globalCfm
   private double errorReductionForFrictionConstraints;              //m_frictionERP
   private double constraintForceMixingForFrictionConstraints;       //m_frictionCFM
   private int splitImpulse;                                         //m_splitImpulse - 0 = false or 1 = true
   private double splitImpulsePenetrationThreshold;                  //m_splitImpulsePenetrationThreshold
   private double splitImpulseTurnErp;                               //m_splitImpulseTurnErp
   private double linearSlop;                                        //m_linearSlop - Used in btMultiBodyConstraintSolver if (!isFriction), adds linearSlop to distance of manifoldPoint
   private double warmstartingFactor;                                //m_warmstartingFactor
   private int solverMode;                                           //m_solverMode
   private int restingContactRestitutionThreshold;                   //m_restingContactRestitutionThreshold - unused as of 2.81
   private int minimumSolverBatchSize;                               //m_minimumSolverBatchSize - try to combine islands until the amount of constraints reaches this limit
   private double maxGyroscopicForce;                                //m_maxGyroscopeForce it is only used for 'explicit' version of gyroscopic force
   private double singleAxisRollingFrictionThreshold;                //m_singleAxisRollingFrictionThreshold - if the velocity is above this threshold, it will use a single constraint row (axis), otherwise 3 rows.
   private double leastSquaresResidualThreshold;                     //m_leastSquaresResidualThreshold
   private double restitutionVelocityThreshold;                      //m_restitutionVelocityThreshold - if the relative velocity is below this threshold, there is zero restitution
   
   public static int SOLVER_RANDMIZE_ORDER = 1;
   public static int SOLVER_FRICTION_SEPARATE = 2;
   public static int SOLVER_USE_WARMSTARTING = 4;
   public static int SOLVER_USE_2_FRICTION_DIRECTIONS = 16;
   public static int SOLVER_ENABLE_FRICTION_DIRECTION_CACHING = 32;
   public static int SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION = 64;
   public static int SOLVER_CACHE_FRIENDLY = 128;
   public static int SOLVER_SIMD = 256;
   public static int SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS = 512;
   public static int SOLVER_ALLOW_ZERO_LENGTH_FRICTION_DIRECTIONS = 1024;
   public static int SOLVER_DISABLE_IMPLICIT_CONE_FRICTION = 2048;
   public static int SOLVER_USE_ARTICULATED_WARMSTARTING = 4096;

   public static BulletContactSolverInfoParameters defaultBulletContactSolverInfoParameters()
   {
      BulletContactSolverInfoParameters bulletContactSolverInfoParameters = new BulletContactSolverInfoParameters();
      bulletContactSolverInfoParameters.setTau(0.6);
      bulletContactSolverInfoParameters.setDamping(1.0);
      bulletContactSolverInfoParameters.setFriction(0.3);
      bulletContactSolverInfoParameters.setTimeStep(1f / 60f);
      bulletContactSolverInfoParameters.setRestitution(0.0);
      bulletContactSolverInfoParameters.setMaxErrorReduction(20.0);
      bulletContactSolverInfoParameters.setNumberOfIterations(10);
      bulletContactSolverInfoParameters.setErrorReductionForNonContactConstraints(0.2);
      bulletContactSolverInfoParameters.setErrorReductionForContactConstraints(0.2);
      bulletContactSolverInfoParameters.setConstraintForceMixingForContactsAndNonContacts(0);
      bulletContactSolverInfoParameters.setErrorReductionForFrictionConstraints(0.2);
      bulletContactSolverInfoParameters.setConstraintForceMixingForFrictionConstraints(0);
      bulletContactSolverInfoParameters.setSuccessiveOverRelaxationTerm(1);
      bulletContactSolverInfoParameters.setSplitImpulse(1);
      bulletContactSolverInfoParameters.setSplitImpulsePenetrationThreshold(-0.04);
      bulletContactSolverInfoParameters.setSplitImpulseTurnErp(0.1);
      bulletContactSolverInfoParameters.setLinearSlop(0.0);
      bulletContactSolverInfoParameters.setWarmstartingFactor(0.85);
      bulletContactSolverInfoParameters.setSolverMode(SOLVER_USE_WARMSTARTING | SOLVER_SIMD | SOLVER_USE_2_FRICTION_DIRECTIONS);
      bulletContactSolverInfoParameters.setRestingContactRestitutionThreshold(2);
      bulletContactSolverInfoParameters.setMinimumSolverBatchSize(128);
      bulletContactSolverInfoParameters.setMaxGyroscopicForce(100.0);
      bulletContactSolverInfoParameters.setSingleAxisRollingFrictionThreshold(1e30f);
      bulletContactSolverInfoParameters.setLeastSquaresResidualThreshold(0);
      bulletContactSolverInfoParameters.setRestitutionVelocityThreshold(0.2);
      
      return bulletContactSolverInfoParameters;
   }
   
   public BulletContactSolverInfoParameters()
   {
 
   }

   public double getTau()
   {
      return tau;
   }

   public void setTau(double tau)
   {
      this.tau = tau;
   }

   public double getDamping()
   {
      return damping;
   }

   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   public double getFriction()
   {
      return friction;
   }

   public void setFriction(double friction)
   {
      this.friction = friction;
   }

   public double getTimeStep()
   {
      return timeStep;
   }

   public void setTimeStep(double timeStep)
   {
      this.timeStep = timeStep;
   }

   public double getRestitution()
   {
      return restitution;
   }

   public void setRestitution(double restitution)
   {
      this.restitution = restitution;
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }

   public void setNumberOfIterations(int numberOfIterations)
   {
      this.numberOfIterations = numberOfIterations;
   }

   public double getMaxErrorReduction()
   {
      return maxErrorReduction;
   }

   public void setMaxErrorReduction(double maxErrorReduction)
   {
      this.maxErrorReduction = maxErrorReduction;
   }

   public double getSuccessiveOverRelaxationTerm()
   {
      return successiveOverRelaxationTerm;
   }

   public void setSuccessiveOverRelaxationTerm(double successiveOverRelaxationTerm)
   {
      this.successiveOverRelaxationTerm = successiveOverRelaxationTerm;
   }

   public double getErrorReductionForNonContactConstraints()
   {
      return errorReductionForNonContactConstraints;
   }

   public void setErrorReductionForNonContactConstraints(double errorReductionForNonContactConstraints)
   {
      this.errorReductionForNonContactConstraints = errorReductionForNonContactConstraints;
   }

   public double getErrorReductionForContactConstraints()
   {
      return errorReductionForContactConstraints;
   }

   public void setErrorReductionForContactConstraints(double errorRecutionForContactConstraints)
   {
      this.errorReductionForContactConstraints = errorRecutionForContactConstraints;
   }

   public double getConstraintForceMixingForContactsAndNonContacts()
   {
      return constraintForceMixingForContactsAndNonContacts;
   }

   public void setConstraintForceMixingForContactsAndNonContacts(double constraintForceMixingForContactsAndNonContacts)
   {
      this.constraintForceMixingForContactsAndNonContacts = constraintForceMixingForContactsAndNonContacts;
   }

   public double getErrorReductionForFrictionConstraints()
   {
      return errorReductionForFrictionConstraints;
   }

   public void setErrorReductionForFrictionConstraints(double errorReductionForFrictionConstraints)
   {
      this.errorReductionForFrictionConstraints = errorReductionForFrictionConstraints;
   }

   public double getConstraintForceMixingForFrictionConstraints()
   {
      return constraintForceMixingForFrictionConstraints;
   }

   public void setConstraintForceMixingForFrictionConstraints(double constraintForceMixingForFrictionConstraints)
   {
      this.constraintForceMixingForFrictionConstraints = constraintForceMixingForFrictionConstraints;
   }

   public int getSplitImpulse()
   {
      return splitImpulse;
   }

   public void setSplitImpulse(int splitImpulse)
   {
      this.splitImpulse = splitImpulse;
   }

   public double getSplitImpulsePenetrationThreshold()
   {
      return splitImpulsePenetrationThreshold;
   }

   public void setSplitImpulsePenetrationThreshold(double splitImpulsePenetrationThreshold)
   {
      this.splitImpulsePenetrationThreshold = splitImpulsePenetrationThreshold;
   }

   public double getSplitImpulseTurnErp()
   {
      return splitImpulseTurnErp;
   }

   public void setSplitImpulseTurnErp(double splitImpulseTurnErp)
   {
      this.splitImpulseTurnErp = splitImpulseTurnErp;
   }

   public double getLinearSlop()
   {
      return linearSlop;
   }

   public void setLinearSlop(double linearSlop)
   {
      this.linearSlop = linearSlop;
   }

   public double getWarmstartingFactor()
   {
      return warmstartingFactor;
   }

   public void setWarmstartingFactor(double warmstartingFactor)
   {
      this.warmstartingFactor = warmstartingFactor;
   }

   public int getSolverMode()
   {
      return solverMode;
   }

   public void setSolverMode(int solverMode)
   {
      this.solverMode = solverMode;
   }

   public int getRestingContactRestitutionThreshold()
   {
      return restingContactRestitutionThreshold;
   }

   public void setRestingContactRestitutionThreshold(int restingContactRestitutionThreshold)
   {
      this.restingContactRestitutionThreshold = restingContactRestitutionThreshold;
   }

   public int getMinimumSolverBatchSize()
   {
      return minimumSolverBatchSize;
   }

   public void setMinimumSolverBatchSize(int minimumSolverBatchSize)
   {
      this.minimumSolverBatchSize = minimumSolverBatchSize;
   }

   public double getMaxGyroscopicForce()
   {
      return maxGyroscopicForce;
   }

   public void setMaxGyroscopicForce(double maxGyroscopicForce)
   {
      this.maxGyroscopicForce = maxGyroscopicForce;
   }

   public double getSingleAxisRollingFrictionThreshold()
   {
      return singleAxisRollingFrictionThreshold;
   }

   public void setSingleAxisRollingFrictionThreshold(double singleAxisRollingFrictionThreshold)
   {
      this.singleAxisRollingFrictionThreshold = singleAxisRollingFrictionThreshold;
   }

   public double getLeastSquaresResidualThreshold()
   {
      return leastSquaresResidualThreshold;
   }

   public void setLeastSquaresResidualThreshold(double leastSquaresResidualThreshold)
   {
      this.leastSquaresResidualThreshold = leastSquaresResidualThreshold;
   }

   public double getRestitutionVelocityThreshold()
   {
      return restitutionVelocityThreshold;
   }

   public void setRestitutionVelocityThreshold(double restitutionVelocityThreshold)
   {
      this.restitutionVelocityThreshold = restitutionVelocityThreshold;
   }
}
