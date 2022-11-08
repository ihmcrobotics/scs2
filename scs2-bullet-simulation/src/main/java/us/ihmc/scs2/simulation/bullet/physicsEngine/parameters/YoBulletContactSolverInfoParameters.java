package us.ihmc.scs2.simulation.bullet.physicsEngine.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoBulletContactSolverInfoParameters
{
   private boolean updateGlobalContactSolverInfoParameters;
   private final YoDouble tau;
   private final YoDouble damping;
   private final YoDouble friction;
   private final YoDouble timeStep;
   private final YoDouble restitution;
   private final YoInteger numberOfIterations;
   private final YoDouble maxErrorReduction;
   private final YoDouble successiveOverRelaxationTerm;
   private final YoDouble errorReductionForNonContactConstraints;
   private final YoDouble errorReductionForContactConstraints;
   private final YoDouble constraintForceMixingForContactsAndNonContacts;
   private final YoDouble errorReductionForFrictionConstraints;
   private final YoDouble constraintForceMixingForFrictionConstraints;
   private final YoInteger splitImpulse;
   private final YoDouble splitImpulsePenetrationThreshold;
   private final YoDouble splitImpulseTurnErp;
   private final YoDouble linearSlop;
   private final YoDouble warmstartingFactor;
   private final YoInteger solverMode;
   private final YoInteger restingContactRestitutionThreshold;
   private final YoInteger minimumSolverBatchSize;
   private final YoDouble maxGyroscopicForce;
   private final YoDouble singleAxisRollingFrictionThreshold;
   private final YoDouble leastSquaresResidualThreshold;
   private final YoDouble restitutionVelocityThreshold;

   public YoBulletContactSolverInfoParameters(String prefix, YoRegistry registry)
   {
      String bulletContactSolverInfoTau;
      String bulletContactSolverInfoDamping;
      String bulletContactSolverInfoFriction;
      String bulletContactSolverInfoTimeStep;
      String bulletContactSolverInfoRestitution;
      String bulletContactSolverInfoNumberOfIterations;
      String bulletContactSolverInfoMaxErrorReduction;
      String bulletContactSolverInfoSuccessiveOverRelaxationTerm;
      String bulletContactSolverInfoErrorReductionForNonContactConstraints;
      String bulletContactSolverInfoErrorReductionForContactConstraints;
      String bulletContactSolverInfoConstraintForceMixingForContactsAndNonContacts;
      String bulletContactSolverInfoErrorReductionForFrictionConstraints;
      String bulletContactSolverInfoConstraintForceMixingForFrictionConstraints;
      String bulletContactSolverInfoSplitImpulse;
      String bulletContactSolverInfoSplitImpulsePenetrationThreshold;
      String bulletContactSolverInfoSplitImpulseTurnErp;
      String bulletContactSolverInfoLinearSlop;
      String bulletContactSolverInfoWarmstartingFactor;
      String bulletContactSolverInfoSolverMode;
      String bulletContactSolverInfoRestingContactRestitutionThreshold;
      String bulletContactSolverInfoMinimumSolverBatchSize;
      String bulletContactSolverInfoMaxGyroscopicForce;
      String bulletContactSolverInfoSingleAxisRollingFrictionThreshold;
      String bulletContactSolverInfoLeastSquaresResidualThreshold;
      String bulletContactSolverInfoRestitutionVelocityThreshold;
      
      if (prefix == null || prefix.isEmpty())
      {
         bulletContactSolverInfoTau = "Tau";
         bulletContactSolverInfoDamping = "Damping";
         bulletContactSolverInfoFriction = "Friction";
         bulletContactSolverInfoTimeStep = "TimeStep";
         bulletContactSolverInfoRestitution = "Restitution";
         bulletContactSolverInfoNumberOfIterations = "NumberOfIterations";
         bulletContactSolverInfoMaxErrorReduction = "MaxErrorReduction";
         bulletContactSolverInfoSuccessiveOverRelaxationTerm = "SuccessiveOverRelaxationTerm";
         bulletContactSolverInfoErrorReductionForNonContactConstraints = "ErrorReductionForNonContactConstraints";
         bulletContactSolverInfoErrorReductionForContactConstraints = "ErrorReductionForContactConstraints";
         bulletContactSolverInfoConstraintForceMixingForContactsAndNonContacts = "ConstraintForceMixingForContactsAndNonContacts";
         bulletContactSolverInfoErrorReductionForFrictionConstraints = "ErrorReductionForFrictionConstraints";
         bulletContactSolverInfoConstraintForceMixingForFrictionConstraints = "ConstraintForceMixingForFrictionConstraints";
         bulletContactSolverInfoSplitImpulse = "SplitImpulse";
         bulletContactSolverInfoSplitImpulsePenetrationThreshold = "SplitImpulsePenetrationThreshold";
         bulletContactSolverInfoSplitImpulseTurnErp = "SplitImpulseTurnErp";
         bulletContactSolverInfoLinearSlop = "LinearSlop";
         bulletContactSolverInfoWarmstartingFactor = "WarmstartingFactor";
         bulletContactSolverInfoSolverMode = "SolverMode";
         bulletContactSolverInfoRestingContactRestitutionThreshold = "RestingContactRestitutionThreshold";
         bulletContactSolverInfoMinimumSolverBatchSize = "MinimumSolverBatchSize";
         bulletContactSolverInfoMaxGyroscopicForce = "MaxGyroscopicForce";
         bulletContactSolverInfoSingleAxisRollingFrictionThreshold = "SingleAxisRollingFrictionThreshold";
         bulletContactSolverInfoLeastSquaresResidualThreshold = "LeastSquaresResidualThreshold";
         bulletContactSolverInfoRestitutionVelocityThreshold = "RestitutionVelocityThreshold";
      }
      else
      {
         bulletContactSolverInfoTau = prefix + "Tau";
         bulletContactSolverInfoDamping = prefix + "Damping";
         bulletContactSolverInfoFriction = prefix + "Friction";
         bulletContactSolverInfoTimeStep = prefix + "TimeStep";
         bulletContactSolverInfoRestitution = prefix + "Restitution";
         bulletContactSolverInfoNumberOfIterations = prefix + "NumberOfIterations";
         bulletContactSolverInfoMaxErrorReduction = prefix + "MaxErrorReduction";
         bulletContactSolverInfoSuccessiveOverRelaxationTerm = prefix + "SuccessiveOverRelaxationTerm";
         bulletContactSolverInfoErrorReductionForNonContactConstraints = prefix + "ErrorReductionForNonContactConstraints";
         bulletContactSolverInfoErrorReductionForContactConstraints = prefix + "ErrorReductionForContactConstraints";
         bulletContactSolverInfoConstraintForceMixingForContactsAndNonContacts = prefix + "ConstraintForceMixingForContactsAndNonContacts";
         bulletContactSolverInfoErrorReductionForFrictionConstraints = prefix + "ErrorReductionForFrictionConstraints";
         bulletContactSolverInfoConstraintForceMixingForFrictionConstraints = prefix + "ConstraintForceMixingForFrictionConstraints";
         bulletContactSolverInfoSplitImpulse = prefix + "SplitImpulse";
         bulletContactSolverInfoSplitImpulsePenetrationThreshold = prefix + "SplitImpulsePenetrationThreshold";
         bulletContactSolverInfoSplitImpulseTurnErp = prefix + "SplitImpulseTurnErp";
         bulletContactSolverInfoLinearSlop = prefix + "LinearSlop";
         bulletContactSolverInfoWarmstartingFactor = prefix + "WarmstartingFactor";
         bulletContactSolverInfoSolverMode = prefix + "SolverMode";
         bulletContactSolverInfoRestingContactRestitutionThreshold = prefix + "RestingContactRestitutionThreshold";
         bulletContactSolverInfoMinimumSolverBatchSize = prefix + "MinimumSolverBatchSize";
         bulletContactSolverInfoMaxGyroscopicForce = prefix + "MaxGyroscopicForce";
         bulletContactSolverInfoSingleAxisRollingFrictionThreshold = prefix + "SingleAxisRollingFrictionThreshold";
         bulletContactSolverInfoLeastSquaresResidualThreshold = prefix + "LeastSquaresResidualThreshold";
         bulletContactSolverInfoRestitutionVelocityThreshold = prefix + "RestitutionVelocityThreshold";
      }
      
      tau = new YoDouble(bulletContactSolverInfoTau, registry);
      damping = new YoDouble(bulletContactSolverInfoDamping, registry);
      friction = new YoDouble(bulletContactSolverInfoFriction, registry);
      timeStep = new YoDouble(bulletContactSolverInfoTimeStep, registry);
      restitution = new YoDouble(bulletContactSolverInfoRestitution, registry);
      numberOfIterations = new YoInteger(bulletContactSolverInfoNumberOfIterations, registry);
      maxErrorReduction = new YoDouble(bulletContactSolverInfoMaxErrorReduction, registry);
      successiveOverRelaxationTerm = new YoDouble(bulletContactSolverInfoSuccessiveOverRelaxationTerm, registry);
      errorReductionForNonContactConstraints = new YoDouble(bulletContactSolverInfoErrorReductionForNonContactConstraints, registry);
      errorReductionForContactConstraints = new YoDouble(bulletContactSolverInfoErrorReductionForContactConstraints, registry);
      constraintForceMixingForContactsAndNonContacts = new YoDouble(bulletContactSolverInfoConstraintForceMixingForContactsAndNonContacts, registry);
      errorReductionForFrictionConstraints = new YoDouble(bulletContactSolverInfoErrorReductionForFrictionConstraints, registry);
      constraintForceMixingForFrictionConstraints = new YoDouble(bulletContactSolverInfoConstraintForceMixingForFrictionConstraints, registry);
      splitImpulse = new YoInteger(bulletContactSolverInfoSplitImpulse, registry);
      splitImpulsePenetrationThreshold = new YoDouble(bulletContactSolverInfoSplitImpulsePenetrationThreshold, registry);
      splitImpulseTurnErp = new YoDouble(bulletContactSolverInfoSplitImpulseTurnErp, registry);
      linearSlop = new YoDouble(bulletContactSolverInfoLinearSlop, registry);
      warmstartingFactor = new YoDouble(bulletContactSolverInfoWarmstartingFactor, registry);
      solverMode = new YoInteger(bulletContactSolverInfoSolverMode, registry);
      restingContactRestitutionThreshold = new YoInteger(bulletContactSolverInfoRestingContactRestitutionThreshold, registry);
      minimumSolverBatchSize = new YoInteger(bulletContactSolverInfoMinimumSolverBatchSize, registry);
      maxGyroscopicForce = new YoDouble(bulletContactSolverInfoMaxGyroscopicForce, registry);
      singleAxisRollingFrictionThreshold = new YoDouble(bulletContactSolverInfoSingleAxisRollingFrictionThreshold, registry);
      leastSquaresResidualThreshold = new YoDouble(bulletContactSolverInfoLeastSquaresResidualThreshold, registry);
      restitutionVelocityThreshold = new YoDouble(bulletContactSolverInfoRestitutionVelocityThreshold, registry);
      setUpdateGlobalContactSolverInfoParameters(false);
      
      tau.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      damping.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      friction.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      timeStep.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      restitution.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      numberOfIterations.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      maxErrorReduction.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      successiveOverRelaxationTerm.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      errorReductionForNonContactConstraints.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      errorReductionForContactConstraints.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      constraintForceMixingForContactsAndNonContacts.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      errorReductionForFrictionConstraints.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      constraintForceMixingForFrictionConstraints.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      splitImpulse.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      splitImpulsePenetrationThreshold.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      splitImpulseTurnErp.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      linearSlop.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      warmstartingFactor.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      solverMode.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      restingContactRestitutionThreshold.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      minimumSolverBatchSize.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      maxGyroscopicForce.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      singleAxisRollingFrictionThreshold.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      leastSquaresResidualThreshold.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
      restitutionVelocityThreshold.addListener(v -> setUpdateGlobalContactSolverInfoParameters(true));
   }

   public void set(BulletContactSolverInfoParameters parameters)
   {
      setTau(parameters.getTau());
      setDamping(parameters.getDamping());
      setFriction(parameters.getFriction());
      setTimeStep(parameters.getTimeStep());
      setRestitution(parameters.getRestitution());
      setNumberOfIterations(parameters.getNumberOfIterations());
      setMaxErrorReduction(parameters.getMaxErrorReduction());
      setSuccessiveOverRelaxationTerm(parameters.getSuccessiveOverRelaxationTerm());
      setErrorReductionForNonContactConstraints(parameters.getErrorReductionForNonContactConstraints());
      setErrorReductionForContactConstraints(parameters.getErrorReductionForContactConstraints());
      setConstraintForceMixingForContactsAndNonContacts(parameters.getConstraintForceMixingForContactsAndNonContacts());
      setErrorReductionForFrictionConstraints(parameters.getErrorReductionForFrictionConstraints());
      setConstraintForceMixingForFrictionConstraints(parameters.getConstraintForceMixingForFrictionConstraints());
      setSplitImpulse(parameters.getSplitImpulse());
      setSplitImpulsePenetrationThreshold(parameters.getSplitImpulsePenetrationThreshold());
      setSplitImpulseTurnErp(parameters.getSplitImpulseTurnErp());
      setLinearSlop(parameters.getLinearSlop());
      setWarmstartingFactor(parameters.getWarmstartingFactor());
      setSolverMode(parameters.getSolverMode());
      setRestingContactRestitutionThreshold(parameters.getRestingContactRestitutionThreshold());
      setMinimumSolverBatchSize(parameters.getMinimumSolverBatchSize());
      setMaxGyroscopicForce(parameters.getMaxGyroscopicForce());
      setSingleAxisRollingFrictionThreshold(parameters.getSingleAxisRollingFrictionThreshold());
      setLeastSquaresResidualThreshold(parameters.getLeastSquaresResidualThreshold());
      setRestitutionVelocityThreshold(parameters.getRestitutionVelocityThreshold());
   }

   public boolean getUpdateGlobalContactSolverInfoParameters()
   {
      return updateGlobalContactSolverInfoParameters;
   }

   public void setUpdateGlobalContactSolverInfoParameters(boolean updateGlobalContactSolverInfoParameters)
   {
      this.updateGlobalContactSolverInfoParameters = updateGlobalContactSolverInfoParameters;
   }

   public double getTau()
   {
      return tau.getValue();
   }

   public void setTau(double tau)
   {
      this.tau.set(tau);
   }

   public double getDamping()
   {
      return damping.getValue();
   }

   public void setDamping(double damping)
   {
      this.damping.set(damping);
   }

   public double getFriction()
   {
      return friction.getValue();
   }

   public void setFriction(double friction)
   {
      this.friction.set(friction);
   }

   public double getTimeStep()
   {
      return timeStep.getValue();
   }

   public void setTimeStep(double timeStep)
   {
      this.timeStep.set(timeStep);
   }

   public double getRestitution()
   {
      return restitution.getValue();
   }

   public void setRestitution(double restitution)
   {
      this.restitution.set(restitution);
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations.getValue();
   }

   public void setNumberOfIterations(int numberOfIterations)
   {
      this.numberOfIterations.set(numberOfIterations);
   }

   public double getMaxErrorReduction()
   {
      return maxErrorReduction.getValue();
   }

   public void setMaxErrorReduction(double maxErrorReduction)
   {
      this.maxErrorReduction.set(maxErrorReduction);
   }

   public double getSuccessiveOverRelaxationTerm()
   {
      return successiveOverRelaxationTerm.getValue();
   }

   public void setSuccessiveOverRelaxationTerm(double successiveOverRelaxationTerm)
   {
      this.successiveOverRelaxationTerm.set(successiveOverRelaxationTerm);
   }

   public double getErrorReductionForNonContactConstraints()
   {
      return errorReductionForNonContactConstraints.getValue();
   }

   public void setErrorReductionForNonContactConstraints(double errorReductionForNonContactConstraints)
   {
      this.errorReductionForNonContactConstraints.set(errorReductionForNonContactConstraints);
   }

   public double getErrorReductionForContactConstraints()
   {
      return errorReductionForContactConstraints.getValue();
   }

   public void setErrorReductionForContactConstraints(double errorReductionForContactConstraints)
   {
      this.errorReductionForContactConstraints.set(errorReductionForContactConstraints);
   }

   public double getConstraintForceMixingForContactsAndNonContacts()
   {
      return constraintForceMixingForContactsAndNonContacts.getValue();
   }

   public void setConstraintForceMixingForContactsAndNonContacts(double constraintForceMixingForContactsAndNonContacts)
   {
      this.constraintForceMixingForContactsAndNonContacts.set(constraintForceMixingForContactsAndNonContacts);
   }

   public double getErrorReductionForFrictionConstraints()
   {
      return errorReductionForFrictionConstraints.getValue();
   }

   public void setErrorReductionForFrictionConstraints(double errorReductionForFrictionConstraints)
   {
      this.errorReductionForFrictionConstraints.set(errorReductionForFrictionConstraints);
   }

   public double getConstraintForceMixingForFrictionConstraints()
   {
      return constraintForceMixingForFrictionConstraints.getValue();
   }

   public void setConstraintForceMixingForFrictionConstraints(double constraintForceMixingForFrictionConstraints)
   {
      this.constraintForceMixingForFrictionConstraints.set(constraintForceMixingForFrictionConstraints);
   }

   public int getSplitImpulse()
   {
      return splitImpulse.getValue();
   }

   public void setSplitImpulse(int splitImpulse)
   {
      this.splitImpulse.set(splitImpulse);
   }

   public double getSplitImpulsePenetrationThreshold()
   {
      return splitImpulsePenetrationThreshold.getValue();
   }

   public void setSplitImpulsePenetrationThreshold(double splitImpulsePenetrationThreshold)
   {
      this.splitImpulsePenetrationThreshold.set(splitImpulsePenetrationThreshold);
   }

   public double getSplitImpulseTurnErp()
   {
      return splitImpulseTurnErp.getValue();
   }

   public void setSplitImpulseTurnErp(double splitImpulseTurnErp)
   {
      this.splitImpulseTurnErp.set(splitImpulseTurnErp);
   }

   public double getLinearSlop()
   {
      return linearSlop.getValue();
   }

   public void setLinearSlop(double linearSlop)
   {
      this.linearSlop.set(linearSlop);
   }

   public double getWarmstartingFactor()
   {
      return warmstartingFactor.getValue();
   }

   public void setWarmstartingFactor(double warmstartingFactor)
   {
      this.warmstartingFactor.set(warmstartingFactor);
   }

   public int getSolverMode()
   {
      return solverMode.getValue();
   }

   public void setSolverMode(int solverMode)
   {
      this.solverMode.set(solverMode);
   }

   public int getRestingContactRestitutionThreshold()
   {
      return restingContactRestitutionThreshold.getValue();
   }

   public void setRestingContactRestitutionThreshold(int restingContactRestitutionThreshold)
   {
      this.restingContactRestitutionThreshold.set(restingContactRestitutionThreshold);
   }

   public int getMinimumSolverBatchSize()
   {
      return minimumSolverBatchSize.getValue();
   }

   public void setMinimumSolverBatchSize(int minimumSolverBatchSize)
   {
      this.minimumSolverBatchSize.set(minimumSolverBatchSize);
   }

   public double getMaxGyroscopicForce()
   {
      return maxGyroscopicForce.getValue();
   }

   public void setMaxGyroscopicForce(double maxGyroscopicForce)
   {
      this.maxGyroscopicForce.set(maxGyroscopicForce);
   }

   public double getSingleAxisRollingFrictionThreshold()
   {
      return singleAxisRollingFrictionThreshold.getValue();
   }

   public void setSingleAxisRollingFrictionThreshold(double singleAxisRollingFrictionThreshold)
   {
      this.singleAxisRollingFrictionThreshold.set(singleAxisRollingFrictionThreshold);
   }

   public double getLeastSquaresResidualThreshold()
   {
      return leastSquaresResidualThreshold.getValue();
   }

   public void setLeastSquaresResidualThreshold(double leastSquaresResidualThreshold)
   {
      this.leastSquaresResidualThreshold.set(leastSquaresResidualThreshold);
   }

   public double getRestitutionVelocityThreshold()
   {
      return restitutionVelocityThreshold.getValue();
   }

   public void setRestitutionVelocityThreshold(double restitutionVelocityThreshold)
   {
      this.restitutionVelocityThreshold.set(restitutionVelocityThreshold);
   }
}
