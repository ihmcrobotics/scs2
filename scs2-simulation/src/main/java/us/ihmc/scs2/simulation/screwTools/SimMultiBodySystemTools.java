package us.ihmc.scs2.simulation.screwTools;

import static us.ihmc.mecano.tools.MultiBodySystemTools.filterJoints;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DMatrix;

import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointReadOnly;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyReadOnly;

/**
 * This class provides a variety of tools to facilitate operations that need to navigate through a
 * multi-body system.
 */
public class SimMultiBodySystemTools
{
   /**
    * Retrieves and gets the root body of the multi-body system the given {@code body} belongs to.
    *
    * @param body an arbitrary body that belongs to the multi-body system that this method is to find
    *             the root.
    * @return the root body.
    */
   public static SimRigidBodyReadOnly getRootBody(SimRigidBodyReadOnly body)
   {
      SimRigidBodyReadOnly root = body;

      while (root.getParentJoint() != null)
      {
         root = root.getParentJoint().getPredecessor();
      }

      return root;
   }

   /**
    * Retrieves and gets the root body of the multi-body system the given {@code body} belongs to.
    *
    * @param body an arbitrary body that belongs to the multi-body system that this method is to find
    *             the root.
    * @return the root body.
    */
   public static SimRigidBodyBasics getRootBody(SimRigidBodyBasics body)
   {
      SimRigidBodyBasics root = body;

      while (root.getParentJoint() != null)
      {
         root = root.getParentJoint().getPredecessor();
      }

      return root;
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the joints
    * that implement {@code OneDoFJointBasics} and that are in between and return them as an array.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param start the rigid-body from where to begin the collection of joints.
    * @param end   the rigid-body where to stop the collection of joints.
    * @return the array of joints representing the path from {@code start} to {@code end}.
    */
   public static SimOneDoFJointBasics[] createOneDoFJointPath(SimRigidBodyBasics start, SimRigidBodyBasics end)
   {
      return filterJoints(createJointPath(start, end), SimOneDoFJointBasics.class);
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the joints
    * that are in between and return them as an array.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param start the rigid-body from where to begin the collection of joints.
    * @param end   the rigid-body where to stop the collection of joints.
    * @return the array of joints representing the path from {@code start} to {@code end}, or
    *         {@code null} if the given rigid-bodies are not part of the same multi-body system.
    */
   public static SimJointBasics[] createJointPath(SimRigidBodyBasics start, SimRigidBodyBasics end)
   {
      List<SimJointBasics> jointPath = new ArrayList<>();
      collectJointPath(start, end, jointPath);
      return jointPath.toArray(new SimJointBasics[jointPath.size()]);
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the joints
    * that are in between and return them as an array.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param start the rigid-body from where to begin the collection of joints.
    * @param end   the rigid-body where to stop the collection of joints.
    * @return the array of joints representing the path from {@code start} to {@code end}, or
    *         {@code null} if the given rigid-bodies are not part of the same multi-body system.
    */
   public static SimJointReadOnly[] createJointPath(SimRigidBodyReadOnly start, SimRigidBodyReadOnly end)
   {
      List<SimJointReadOnly> jointPath = new ArrayList<>();
      collectJointPath(start, end, jointPath);
      return jointPath.toArray(new SimJointReadOnly[jointPath.size()]);
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the joints
    * that are in between in the given {@code jointPathToPack}.
    * <p>
    * The resulting joint path represent the shortest path connecting {@code start} and {@code end}. No
    * assumption is made on the relative position of the two rigid-bodies in the multi-body system.
    * </p>
    *
    * @param start           the rigid-body where to begin collecting the joints.
    * @param end             the rigid-body where to stop collecting the joints.
    * @param jointPathToPack the list in which the joint path is stored. Note that the list is first
    *                        cleared before storing the joint path.
    * @return the nearest common ancestor of {@code start} and {@code end}.
    */
   public static SimRigidBodyReadOnly collectJointPath(SimRigidBodyReadOnly start, SimRigidBodyReadOnly end, List<SimJointReadOnly> jointPathToPack)
   {
      jointPathToPack.clear();

      SimRigidBodyReadOnly ancestor = computeNearestCommonAncestor(start, end);
      SimRigidBodyReadOnly currentBody;

      currentBody = start;

      while (currentBody != ancestor)
      {
         SimJointReadOnly parentJoint = currentBody.getParentJoint();
         jointPathToPack.add(parentJoint);
         currentBody = parentJoint.getPredecessor();
      }

      int distance = jointPathToPack.size();
      currentBody = end;

      while (currentBody != ancestor)
      {
         currentBody = currentBody.getParentJoint().getPredecessor();
         distance++;
      }

      while (jointPathToPack.size() < distance)
         jointPathToPack.add(null);

      currentBody = end;

      for (int i = distance - 1; currentBody != ancestor; i--)
      {
         SimJointReadOnly parentJoint = currentBody.getParentJoint();
         jointPathToPack.set(i, parentJoint);
         currentBody = parentJoint.getPredecessor();
      }
      return ancestor;
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the joints
    * that are in between in the given {@code jointPathToPack}.
    * <p>
    * The resulting joint path represent the shortest path connecting {@code start} and {@code end}. No
    * assumption is made on the relative position of the two rigid-bodies in the multi-body system.
    * </p>
    *
    * @param start           the rigid-body where to begin collecting the joints.
    * @param end             the rigid-body where to stop collecting the joints.
    * @param jointPathToPack the list in which the joint path is stored. Note that the list is first
    *                        cleared before storing the joint path.
    * @return the nearest common ancestor of {@code start} and {@code end}.
    */
   public static SimRigidBodyBasics collectJointPath(SimRigidBodyBasics start, SimRigidBodyBasics end, List<SimJointBasics> jointPathToPack)
   {
      jointPathToPack.clear();

      SimRigidBodyBasics ancestor = computeNearestCommonAncestor(start, end);
      SimRigidBodyBasics currentBody;

      currentBody = start;

      while (currentBody != ancestor)
      {
         SimJointBasics parentJoint = currentBody.getParentJoint();
         jointPathToPack.add(parentJoint);
         currentBody = parentJoint.getPredecessor();
      }

      int distance = jointPathToPack.size();
      currentBody = end;

      while (currentBody != ancestor)
      {
         currentBody = currentBody.getParentJoint().getPredecessor();
         distance++;
      }

      while (jointPathToPack.size() < distance)
         jointPathToPack.add(null);

      currentBody = end;

      for (int i = distance - 1; currentBody != ancestor; i--)
      {
         SimJointBasics parentJoint = currentBody.getParentJoint();
         jointPathToPack.set(i, parentJoint);
         currentBody = parentJoint.getPredecessor();
      }
      return ancestor;
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the
    * rigid-bodies that connect {@code start} to {@code end} in the given {@code rigidBodyPathToPack}.
    * <p>
    * The resulting resulting path includes both {@code start} and {@code end} and represent the
    * shortest path connecting the two rigid-bodies. No assumption is made on the relative position of
    * the two rigid-bodies in the multi-body system.
    * </p>
    *
    * @param start               the rigid-body where to begin collecting the rigid-bodies.
    * @param end                 the rigid-body where to stop collecting the rigid-bodies.
    * @param rigidBodyPathToPack the list in which the rigid-body path is stored. Note that the list is
    *                            first cleared before storing the rigid-body path.
    * @return the nearest common ancestor of {@code start} and {@code end}.
    */
   public static SimRigidBodyReadOnly collectRigidBodyPath(SimRigidBodyReadOnly start, SimRigidBodyReadOnly end, List<SimRigidBodyReadOnly> rigidBodyPathToPack)
   {
      rigidBodyPathToPack.clear();

      if (start == end)
      {
         rigidBodyPathToPack.add(end);
         return end;
      }

      SimRigidBodyReadOnly ancestor = computeNearestCommonAncestor(start, end);
      SimRigidBodyReadOnly currentBody;

      currentBody = start;

      if (start == ancestor)
         rigidBodyPathToPack.add(start);

      while (currentBody != ancestor)
      {
         rigidBodyPathToPack.add(currentBody);
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      int distance = rigidBodyPathToPack.size();
      currentBody = end;

      while (currentBody != ancestor)
      {
         currentBody = currentBody.getParentJoint().getPredecessor();
         distance++;
      }

      while (rigidBodyPathToPack.size() < distance)
         rigidBodyPathToPack.add(null);

      currentBody = end;

      if (end == ancestor)
         rigidBodyPathToPack.add(end);

      for (int i = distance - 1; currentBody != ancestor; i--)
      {
         rigidBodyPathToPack.set(i, currentBody);
         currentBody = currentBody.getParentJoint().getPredecessor();
      }
      return ancestor;
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the
    * rigid-bodies that connect {@code start} to {@code end} in the given {@code rigidBodyPathToPack}.
    * <p>
    * The resulting resulting path includes both {@code start} and {@code end} and represent the
    * shortest path connecting the two rigid-bodies. No assumption is made on the relative position of
    * the two rigid-bodies in the multi-body system.
    * </p>
    *
    * @param start               the rigid-body where to begin collecting the rigid-bodies.
    * @param end                 the rigid-body where to stop collecting the rigid-bodies.
    * @param rigidBodyPathToPack the list in which the rigid-body path is stored. Note that the list is
    *                            first cleared before storing the rigid-body path.
    * @return the nearest common ancestor of {@code start} and {@code end}.
    */
   public static SimRigidBodyBasics collectRigidBodyPath(SimRigidBodyBasics start, SimRigidBodyBasics end, List<SimRigidBodyBasics> rigidBodyPathToPack)
   {
      rigidBodyPathToPack.clear();

      if (start == end)
      {
         rigidBodyPathToPack.add(end);
         return end;
      }

      SimRigidBodyBasics ancestor = computeNearestCommonAncestor(start, end);
      SimRigidBodyBasics currentBody;

      currentBody = start;

      if (start == ancestor)
         rigidBodyPathToPack.add(start);

      while (currentBody != ancestor)
      {
         rigidBodyPathToPack.add(currentBody);
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      int distance = rigidBodyPathToPack.size();
      currentBody = end;

      while (currentBody != ancestor)
      {
         currentBody = currentBody.getParentJoint().getPredecessor();
         distance++;
      }

      while (rigidBodyPathToPack.size() < distance)
         rigidBodyPathToPack.add(null);

      currentBody = end;

      if (end == ancestor)
         rigidBodyPathToPack.add(end);

      for (int i = distance - 1; currentBody != ancestor; i--)
      {
         rigidBodyPathToPack.set(i, currentBody);
         currentBody = currentBody.getParentJoint().getPredecessor();
      }
      return ancestor;
   }

   /**
    * Finds the common ancestor of {@code firstBody} and {@code secondBody} that minimizes the distance
    * {@code d}:
    *
    * <pre>
    * d = <i>computeDistanceToAncestor</i>(firstBody, ancestor) + <i>computeDistanceToAncestor</i>(secondBody, ancestor)
    * </pre>
    *
    * @param firstBody  the first rigid-body of the query.
    * @param secondBody the second rigid-body of the query.
    * @return the nearest common ancestor.
    * @throws IllegalArgumentException if the two rigid-bodies do not belong to the same multi-body
    *                                  system.
    */
   public static SimRigidBodyBasics computeNearestCommonAncestor(SimRigidBodyBasics firstBody, SimRigidBodyBasics secondBody)
   {
      return (SimRigidBodyBasics) computeNearestCommonAncestor((SimRigidBodyReadOnly) firstBody, (SimRigidBodyReadOnly) secondBody);
   }

   /**
    * Finds the common ancestor of {@code firstBody} and {@code secondBody} that minimizes the distance
    * {@code d}:
    *
    * <pre>
    * d = <i>computeDistanceToAncestor</i>(firstBody, ancestor) + <i>computeDistanceToAncestor</i>(secondBody, ancestor)
    * </pre>
    *
    * @param firstBody  the first rigid-body of the query.
    * @param secondBody the second rigid-body of the query.
    * @return the nearest common ancestor.
    * @throws IllegalArgumentException if the two rigid-bodies do not belong to the same multi-body
    *                                  system.
    */
   public static SimRigidBodyReadOnly computeNearestCommonAncestor(SimRigidBodyReadOnly firstBody, SimRigidBodyReadOnly secondBody)
   {
      return (SimRigidBodyReadOnly) MultiBodySystemTools.computeNearestCommonAncestor(firstBody, secondBody);
   }

   /**
    * Collects in order the successor of each joint, i.e. {@link SimJointReadOnly#getSuccessor()}.
    * <p>
    * Note on kinematic loops: if {@code joints} contains the two joints terminating a kinematic loop,
    * then the common successor will occur twice in the result.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param joints the joints to collect the successors of.
    * @return the array containing in order the successor of each joint.
    */
   // TODO Consider explicitly handling successor of kinematic loops
   public static SimRigidBodyReadOnly[] collectSuccessors(SimJointReadOnly... joints)
   {
      return Stream.of(joints).map(SimJointReadOnly::getSuccessor).toArray(SimRigidBodyReadOnly[]::new);
   }

   /**
    * Collects in order the successor of each joint, i.e. {@link SimJointReadOnly#getSuccessor()}.
    * <p>
    * Note on kinematic loops: if {@code joints} contains the two joints terminating a kinematic loop,
    * then the common successor will occur twice in the result.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param joints the joints to collect the successors of.
    * @return the array containing in order the successor of each joint.
    */
   // TODO Consider explicitly handling successor of kinematic loops
   public static SimRigidBodyBasics[] collectSuccessors(SimJointBasics... joints)
   {
      return Stream.of(joints).map(SimJointBasics::getSuccessor).toArray(SimRigidBodyBasics[]::new);
   }

   /**
    * Collects any rigid-body that composes any of the subtrees originating at the given
    * {@code joints}.
    * <p>
    * Note on kinematic loops: if {@code joints} contains the two joints terminating a kinematic loop,
    * then the subtree of the common successor will occur twice in the result.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param joints the joints indicating the start of each subtree to collect.
    * @return the array containing all the rigid-bodies composing the subtrees.
    */
   // TODO Consider explicitly handling successor of kinematic loops
   public static SimRigidBodyReadOnly[] collectSubtreeSuccessors(SimJointReadOnly... joints)
   {
      return Stream.of(joints).map(SimJointReadOnly::getSuccessor).flatMap(SimRigidBodyReadOnly::subtreeStream).distinct().toArray(SimRigidBodyReadOnly[]::new);
   }

   /**
    * Collects any rigid-body that composes any of the subtrees originating at the given
    * {@code joints}.
    * <p>
    * Note on kinematic loops: if {@code joints} contains the two joints terminating a kinematic loop,
    * then the subtree of the common successor will occur twice in the result.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param joints the joints indicating the start of each subtree to collect.
    * @return the array containing all the rigid-bodies composing the subtrees.
    */
   // TODO Consider explicitly handling successor of kinematic loops
   public static SimRigidBodyBasics[] collectSubtreeSuccessors(SimJointBasics... joints)
   {
      return Stream.of(joints).map(SimJointBasics::getSuccessor).flatMap(SimRigidBodyBasics::subtreeStream).distinct().toArray(SimRigidBodyBasics[]::new);
   }

   /**
    * Collects and returns all the joints located between the given {@code rigidBody} and the root
    * body.
    * <p>
    * Note on kinematic loops: this method does not collect the joints on the secondary branch of a
    * kinematic loop, i.e. the branch that starts off the primary branch and ends with the loop closure
    * joint.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect the support joints of.
    * @return the array containing the support joints of the given rigid-body.
    */
   public static SimJointReadOnly[] collectSupportJoints(SimRigidBodyReadOnly rigidBody)
   {
      return createJointPath(getRootBody(rigidBody), rigidBody);
   }

   /**
    * Collects and returns all the joints located between the given {@code rigidBody} and the root
    * body.
    * <p>
    * Note on kinematic loops: this method does not collect the joints on the secondary branch of a
    * kinematic loop, i.e. the branch that starts off the primary branch and ends with the loop closure
    * joint.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect the support joints of.
    * @return the array containing the support joints of the given rigid-body.
    */
   public static SimJointBasics[] collectSupportJoints(SimRigidBodyBasics rigidBody)
   {
      return createJointPath(getRootBody(rigidBody), rigidBody);
   }

   /**
    * Collects for each rigid-body all their support joints, i.e. the joints that are between the
    * rigid-body and the root body, and returns an array containing no duplicate elements.
    * <p>
    * Note on kinematic loops: this method does not collect the joints on the secondary branch of a
    * kinematic loop, i.e. the branch that starts off the primary branch and ends with the loop closure
    * joint.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBodies the rigid-bodies to collect the support joints of.
    * @return the array containing the support joints of all the given rigid-bodies.
    */
   public static SimJointReadOnly[] collectSupportJoints(SimRigidBodyReadOnly... rigidBodies)
   {
      return Stream.of(rigidBodies).map(SimMultiBodySystemTools::collectSupportJoints).flatMap(Stream::of).distinct().toArray(SimJointReadOnly[]::new);
   }

   /**
    * Collects for each rigid-body all their support joints, i.e. the joints that are between the
    * rigid-body and the root body, and returns an array containing no duplicate elements.
    * <p>
    * Note on kinematic loops: this method does not collect the joints on the secondary branch of a
    * kinematic loop, i.e. the branch that starts off the primary branch and ends with the loop closure
    * joint.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBodies the rigid-bodies to collect the support joints of.
    * @return the array containing the support joints of all the given rigid-bodies.
    */
   public static SimJointBasics[] collectSupportJoints(SimRigidBodyBasics... rigidBodies)
   {
      return Stream.of(rigidBodies).map(SimMultiBodySystemTools::collectSupportJoints).flatMap(Stream::of).distinct().toArray(SimJointBasics[]::new);
   }

   /**
    * Collects all the joints that are part of any of the subtrees originating from the given
    * {@code rootBodies}, and returns an array containing no duplicate elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rootBodies the rigid-bodies from which the subtree to collect start off.
    * @return the array containing all the joint composing the subtrees.
    */
   public static SimJointReadOnly[] collectSubtreeJoints(SimRigidBodyReadOnly... rootBodies)
   {
      return Stream.of(rootBodies).flatMap(SubtreeStreams::fromChildren).distinct().toArray(SimJointReadOnly[]::new);
   }

   /**
    * Collects all the joints that are part of any of the subtrees originating from the given
    * {@code rootBodies}, and returns an array containing no duplicate elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rootBodies the rigid-bodies from which the subtree to collect start off.
    * @return the array containing all the joint composing the subtrees.
    */
   public static SimJointBasics[] collectSubtreeJoints(SimRigidBodyBasics... rootBodies)
   {
      return Stream.of(rootBodies).flatMap(SubtreeStreams::fromChildren).distinct().toArray(SimJointBasics[]::new);
   }

   /**
    * Collects all the joints that are part of any of the subtrees originating from the given
    * {@code rootBodies}, and returns an array containing no duplicate elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rootBodies the rigid-bodies from which the subtree to collect start off.
    * @return the array containing all the joint composing the subtrees.
    */
   public static SimJointReadOnly[] collectSubtreeJoints(List<? extends SimRigidBodyReadOnly> rootBodies)
   {
      return rootBodies.stream().flatMap(SubtreeStreams::fromChildren).distinct().toArray(SimJointReadOnly[]::new);
   }

   /**
    * Combines {@link #collectSupportJoints(SimRigidBodyReadOnly)} with
    * {@link #collectSubtreeJoints(SimRigidBodyReadOnly...)}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect the support and subtree joints of.
    * @return the array containing the support and subtree joints.
    * @see #collectSupportJoints(SimRigidBodyReadOnly)
    * @see #collectSubtreeJoints(SimRigidBodyReadOnly...)
    */
   public static SimJointReadOnly[] collectSupportAndSubtreeJoints(SimRigidBodyReadOnly rigidBody)
   {
      List<SimJointReadOnly> supportAndSubtreeJoints = SubtreeStreams.fromChildren(SimJointReadOnly.class, rigidBody).collect(Collectors.toList());
      supportAndSubtreeJoints.addAll(Arrays.asList(collectSupportJoints(rigidBody)));
      return supportAndSubtreeJoints.toArray(new SimJointReadOnly[supportAndSubtreeJoints.size()]);
   }

   /**
    * Combines {@link #collectSupportJoints(SimRigidBodyBasics)} with
    * {@link #collectSubtreeJoints(SimRigidBodyBasics...)}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect the support and subtree joints of.
    * @return the array containing the support and subtree joints.
    * @see #collectSupportJoints(SimRigidBodyBasics)
    * @see #collectSubtreeJoints(SimRigidBodyBasics...)
    */
   public static SimJointBasics[] collectSupportAndSubtreeJoints(SimRigidBodyBasics rigidBody)
   {
      List<SimJointBasics> supportAndSubtreeJoints = new ArrayList<>();
      Stream.of(collectSupportJoints(rigidBody)).forEach(supportAndSubtreeJoints::add);
      rigidBody.childrenSubtreeIterable().forEach(supportAndSubtreeJoints::add);
      return supportAndSubtreeJoints.toArray(new SimJointBasics[supportAndSubtreeJoints.size()]);
   }

   /**
    * Combines {@link #collectSupportJoints(SimRigidBodyReadOnly...)} with
    * {@link #collectSubtreeJoints(SimRigidBodyReadOnly...)}, and returns an array containing no
    * duplicate elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBodies the rigid-bodies to collect the support and subtree joints of.
    * @return the array containing the support and subtree joints.
    * @see #collectSupportJoints(SimRigidBodyReadOnly)
    * @see #collectSubtreeJoints(SimRigidBodyReadOnly...)
    */
   public static SimJointReadOnly[] collectSupportAndSubtreeJoints(SimRigidBodyReadOnly... rigidBodies)
   {
      return Stream.of(rigidBodies).map(SimMultiBodySystemTools::collectSupportAndSubtreeJoints).flatMap(Stream::of).distinct()
                   .toArray(SimJointReadOnly[]::new);
   }

   /**
    * Combines {@link #collectSupportJoints(SimRigidBodyBasics...)} with
    * {@link #collectSubtreeJoints(SimRigidBodyBasics...)}, and returns an array containing no
    * duplicate elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBodies the rigid-bodies to collect the support and subtree joints of.
    * @return the array containing the support and subtree joints.
    * @see #collectSupportJoints(SimRigidBodyBasics)
    * @see #collectSubtreeJoints(SimRigidBodyBasics...)
    */
   public static SimJointBasics[] collectSupportAndSubtreeJoints(SimRigidBodyBasics... rigidBodies)
   {
      return Stream.of(rigidBodies).map(SimMultiBodySystemTools::collectSupportAndSubtreeJoints).flatMap(Stream::of).distinct().toArray(SimJointBasics[]::new);
   }

   /**
    * Collects starting from the given {@code rigidBody} all descendant that has no children, i.e. all
    * end-effector.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect of descendant end-effectors of.
    * @return the array containing the end-effectors.
    */
   public static SimRigidBodyBasics[] collectSubtreeEndEffectors(SimRigidBodyBasics rigidBody)
   {
      return rigidBody.subtreeStream().filter(body -> body.getChildrenJoints().isEmpty()).toArray(SimRigidBodyBasics[]::new);
   }

   /**
    * Collects starting from the given {@code rigidBody} all descendant that has no children, i.e. all
    * end-effector.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect of descendant end-effectors of.
    * @return the array containing the end-effectors.
    */
   public static SimRigidBodyReadOnly[] collectSubtreeEndEffectors(SimRigidBodyReadOnly rigidBody)
   {
      return rigidBody.subtreeStream().filter(body -> body.getChildrenJoints().isEmpty()).toArray(SimRigidBodyReadOnly[]::new);
   }

   /**
    * Copies the requested state from the {@code source} joints to the {@code destination} joints.
    * <p>
    * The two lists should be of same length and each pair (source, destination) joint for any given
    * index should be of the same type.
    * </p>
    *
    * @param source         the joints holding the state to copy over. Not modified.
    * @param destination    the joints which state is to be be updated. Modified.
    * @param stateSelection the state that is to be copied over.
    */
   public static void copyJointsState(List<? extends SimJointReadOnly> source, List<? extends SimJointBasics> destination, SimJointStateType stateSelection)
   {
      if (source.size() != destination.size())
         throw new IllegalArgumentException("Inconsistent argument size: source = " + source.size() + ", destination = " + destination.size() + ".");

      switch (stateSelection)
      {
         case VELOCITY_CHANGE:
            copyJointsDeltaVelocity(source, destination);
            return;
         default:
            MultiBodySystemTools.copyJointsState(source, destination, stateSelection.toJointStateType());
            return;
      }
   }

   private static void copyJointsDeltaVelocity(List<? extends SimJointReadOnly> source, List<? extends SimJointBasics> destination)
   {
      for (int jointIndex = 0; jointIndex < source.size(); jointIndex++)
      {
         SimJointReadOnly sourceJoint = source.get(jointIndex);
         SimJointBasics destinationJoint = destination.get(jointIndex);
         destinationJoint.setJointDeltaTwist(sourceJoint);
      }
   }

   /**
    * Iterates through the given {@code joints}, extract the requested state {@code stateSelection} for
    * each joint, and finally stores the states in order in the given matrix {@code matrixToPack}.
    *
    * @param joints         the joints to extract the state of. Not modified.
    * @param stateSelection indicates what state is to be extract, i.e. it can be either configuration,
    *                       velocity, acceleration, or tau (or effort).
    * @param matrixToPack   the matrix in which the state of the joints is to be stored. Modified.
    * @return the number of rows used to store the information in the matrix.
    */
   public static int extractJointsState(List<? extends SimJointReadOnly> joints, SimJointStateType stateSelection, DMatrix matrixToPack)
   {
      switch (stateSelection)
      {
         case VELOCITY_CHANGE:
            return extractJointsDeltaVelocity(joints, 0, matrixToPack);
         default:
            return MultiBodySystemTools.extractJointsState(joints, stateSelection.toJointStateType(), matrixToPack);
      }
   }

   private static int extractJointsDeltaVelocity(List<? extends SimJointReadOnly> joints, int startIndex, DMatrix matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         SimJointReadOnly joint = joints.get(jointIndex);
         startIndex = joint.getJointDeltaVelocity(startIndex, matrixToPack);
      }

      return startIndex;
   }

   /**
    * Iterates through the given {@code joints}, extract the requested state {@code stateSelection} for
    * each joint, and finally stores the states in order in the given matrix {@code matrixToPack}.
    *
    * @param joints         the joints to extract the state of. Not modified.
    * @param stateSelection indicates what state is to be extract, i.e. it can be either configuration,
    *                       velocity, acceleration, or tau (or effort).
    * @param matrixToPack   the matrix in which the state of the joints is to be stored. Modified.
    * @return the number of rows used to store the information in the matrix.
    */
   public static int extractJointsState(SimJointReadOnly[] joints, SimJointStateType stateSelection, DMatrix matrixToPack)
   {
      switch (stateSelection)
      {
         case VELOCITY_CHANGE:
            return extractJointsDeltaVelocity(joints, 0, matrixToPack);
         default:
            return MultiBodySystemTools.extractJointsState(joints, stateSelection.toJointStateType(), matrixToPack);
      }
   }

   private static int extractJointsDeltaVelocity(SimJointReadOnly[] joints, int startIndex, DMatrix matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         SimJointReadOnly joint = joints[jointIndex];
         startIndex = joint.getJointDeltaVelocity(startIndex, matrixToPack);
      }

      return startIndex;
   }

   /**
    * Iterates through the given {@code joints}, and update their requested state
    * {@code stateSelection} using the given {@code matrix} assuming the state has been previously
    * stored in the proper order.
    *
    * @param joints         the joints to update the state of. Modified.
    * @param stateSelection indicates what state is to be updated, i.e. it can be either configuration,
    *                       velocity, acceleration, or tau (or effort).
    * @param matrix         the matrix in which the new state of the joints is stored. The data is
    *                       expected to be stored as a column vector starting at the first row.
    *                       Modified.
    * @return the number of rows that were used from the matrix.
    */
   public static int insertJointsState(List<? extends SimJointBasics> joints, SimJointStateType stateSelection, DMatrix matrix)
   {
      switch (stateSelection)
      {
         case VELOCITY_CHANGE:
            return insertJointsDeltaVelocity(joints, 0, matrix);
         default:
            return MultiBodySystemTools.insertJointsState(joints, stateSelection.toJointStateType(), matrix);
      }
   }

   private static int insertJointsDeltaVelocity(List<? extends SimJointBasics> joints, int startIndex, DMatrix matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         SimJointBasics joint = joints.get(jointIndex);
         startIndex = joint.setJointDeltaVelocity(startIndex, matrix);
      }

      return startIndex;
   }

   /**
    * Iterates through the given {@code joints}, and update their requested state
    * {@code stateSelection} using the given {@code matrix} assuming the state has been previously
    * stored in the proper order.
    *
    * @param joints         the joints to update the state of. Modified.
    * @param stateSelection indicates what state is to be updated, i.e. it can be either configuration,
    *                       velocity, acceleration, or tau (or effort).
    * @param matrix         the matrix in which the new state of the joints is stored. The data is
    *                       expected to be stored as a column vector starting at the first row.
    *                       Modified.
    * @return the number of rows that were used from the matrix.
    */
   public static int insertJointsState(SimJointBasics[] joints, SimJointStateType stateSelection, DMatrix matrix)
   {
      switch (stateSelection)
      {
         case VELOCITY_CHANGE:
            return insertJointsDeltaVelocity(joints, 0, matrix);
         default:
            return MultiBodySystemTools.insertJointsState(joints, stateSelection.toJointStateType(), matrix);
      }
   }

   private static int insertJointsDeltaVelocity(SimJointBasics[] joints, int startIndex, DMatrix matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         SimJointBasics joint = joints[jointIndex];
         startIndex = joint.setJointDeltaVelocity(startIndex, matrix);
      }

      return startIndex;
   }
}