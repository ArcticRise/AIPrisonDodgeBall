using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

using GameAI;

[DefaultExecutionOrder(100)]
[RequireComponent(typeof(MinionScript))]
public class MinionStateMachine : MonoBehaviour
{
    public const string StudentName = "Michael Vo";

    public const string GlobalTransitionStateName = "GlobalTransition";
    public const string CollectBallStateName = "CollectBall";
    public const string GoToThrowSpotStateName = "GoToThrowBall";
    public const string ThrowBallStateName = "ThrowBall";
    public const string DefensiveDemoStateName = "DefensiveDemo";
    public const string GoToPrisonStateName = "GoToPrison";
    public const string LeavePrisonStateName = "LeavePrison";
    public const string GoHomeStateName = "GoHome";
    public const string RescueStateName = "Rescue";
    public const string RestStateName = "Rest";


    // For throws...
    public static float MaxAllowedThrowPositionError = (0.25f + 0.5f) * 0.99f;

    // Data that each FSM state gets initialized with (passed as init param)
    FiniteStateMachine<MinionFSMData> fsm;

    public MinionScript Minion { get; private set; }

    PrisonDodgeballManager Mgr;
    public TeamShare TeamData { get; private set; }

    struct MinionFSMData
    {
        public MinionStateMachine MinionFSM { get; private set; }
        public MinionScript Minion { get; private set; }
        public PrisonDodgeballManager Mgr { get; private set; }
        public PrisonDodgeballManager.Team Team { get; private set; }
        public TeamShare TeamData { get; private set; }

        public MinionFSMData(
            MinionStateMachine minionFSM,
            MinionScript minion,
            PrisonDodgeballManager mgr,
            PrisonDodgeballManager.Team team,
            TeamShare teamData
            )
        {
            MinionFSM = minionFSM;
            Minion = minion;
            Mgr = mgr;
            Team = team;
            TeamData = teamData;
        }
    }


    public static bool LoC(
      // The initial launch position of the projectile
      Vector3 projectilePos,
      // The initial ballistic speed of the projectile
      float maxProjectileSpeed,
      // The gravity vector affecting the projectile (likely passed as Physics.gravity)
      Vector3 projectileGravity,
      // The initial position of the target
      Vector3 targetInitPos,
      // The constant velocity of the target (zero acceleration assumed)
      Vector3 targetConstVel,
      // The forward facing direction of the target. Possibly of use if the target
      // velocity is zero
      Vector3 targetForwardDir,
      // For algorithms that approximate the solution, this sets a limit for how far
      // the target and projectile can be from each other at the interceptT time
      // and still count as a successful prediction
      float maxAllowedErrorDist,
      // Output param: The solved projectileDir for ballistic trajectory that intercepts target
      out Vector3 projectileDir,
      // Output param: The speed the projectile is launched at in projectileDir such that
      // there is a collision with target. projectileSpeed must be <= maxProjectileSpeed
      out float projectileSpeed,
      // Output param: The time at which the projectile and target collide
      out float interceptT,
      // Output param: An alternate time at which the projectile and target collide
      // Note that this is optional to use and does NOT coincide with the solved projectileDir
      // and projectileSpeed. It is possibly useful to pass on to an incremental solver.
      // It only exists to simplify compatibility with the ShootingRange
      out float altT)
    {
        var a_scaler = projectilePos - targetInitPos;
        float a_scarer_mag = Mathf.Sqrt(a_scaler.sqrMagnitude);
        var s_t = Mathf.Sqrt(targetConstVel.sqrMagnitude);
        var a_scalar_norm = a_scaler;
        a_scalar_norm.Normalize();
        var target_norm = targetConstVel;
        target_norm.Normalize();
        var cos_theta = Vector3.Dot(a_scalar_norm, target_norm);
        if (s_t < 0)
        {
            cos_theta = 1.0f;
        }
        bool shoot = true;


        var A = (maxProjectileSpeed * maxProjectileSpeed) - targetConstVel.sqrMagnitude;
        var B = 2.0f * a_scarer_mag * s_t * cos_theta;
        var C = -a_scaler.sqrMagnitude;
        var radicand = (B * B) - 4.0f * A * C;
        var denom = 2.0f * A;
        if (radicand < 0.0f)
        {
            shoot = false;
            altT = -1f;
            projectileDir = Vector3.zero;
            interceptT = -1f;
            projectileSpeed = -1f;
            return shoot;
        }
        if (denom == 0.0f && cos_theta > 0.0f)
        {
            interceptT = a_scarer_mag / (2.0f * s_t * cos_theta);

        }else if(denom == 0.0f && cos_theta <= 0.0f)
        {
            shoot = false;
            altT = -1f;
            projectileDir = Vector3.zero;
            interceptT = -1f;
            projectileSpeed = -1f;
            return shoot;
        }
        else
        {
            float discr = Mathf.Sqrt(radicand);
            var quad_min = (-B - discr) / denom;
            var quad_plus = (-B + discr) / denom;

            if (quad_min < 0.0f && quad_plus >= 0.0f)
            {
                interceptT = quad_plus;
            }
            else if (quad_min >= 0.0f && quad_plus < 0.0f)
            {
                interceptT = quad_min;

            }
            else if (quad_min >= 0.0f && quad_plus >= 0.0f)
            {
                interceptT = Mathf.Min(quad_min, quad_plus);
            }
            else
            {
                shoot = false;
                altT = -1f;
                projectileDir = Vector3.zero;
                interceptT = -1f;
                projectileSpeed = -1f;
                return shoot;
            }
        }

        var p = targetInitPos - projectilePos;
        var v_p = (p / interceptT) + targetConstVel - ((0.5f) * projectileGravity * interceptT);
        // Only going 2D for simple demo. this is not useful for proper prediction
        // Basically, avoiding throwing down at enemies since we aren't predicting accurately here.
        /*        var targetPos2d = new Vector3(targetInitPos.x, 0f, targetInitPos.z);
                var launchPos2d = new Vector3(projectilePos.x, 0f, projectilePos.z);

                var relVec = (targetPos2d - launchPos2d);
                interceptT = relVec.magnitude / maxProjectileSpeed;
                altT = -1f;

                // This is a hard-coded approximate sort of of method to figure out a loft angle
                // This is NOT the right thing to do for your prediction code!
                var normAngle = Mathf.Lerp(0f, 20f, interceptT * 0.007f);
                var v = Vector3.Slerp(relVec.normalized, Vector3.up, normAngle);*/

        // Make sure this is normalized! (The direction of your throw)
        altT = -1f;
        // You'll probably want to leave this as is. For advanced prediction you can slow your throw down
        // You don't need to predict the speed of your throw. Only the direction assuming full speed
        projectileDir = v_p.normalized;
        projectileSpeed = v_p.magnitude;

        // TODO return true or false based on whether target can actually be hit
        // This implementation just thinks, "I guess so?", and returns true
        // Implementations that don't exactly solve intercepts will need to test the approximate
        // solution with maxAllowedErrorDist. If your solution does solve exactly, you will
        // probably want to add a debug assertion to check your solution against it.
        return shoot;

    }

    // Note: You have to implement the following method with prediction:
    // Either directly solved (e.g. Law of Cosines or similar) or iterative.
    // You cannot modify the method signature. However, if you want to do more advanced
    // prediction (such as analysis of the navmesh) then you can make another method that calls
    // this one. 
    // Be sure to run the editor mode unit test to confirm that this method runs without
    // any gamemode-only logic
    public static bool PredictThrow(
        // The initial launch position of the projectile
        Vector3 projectilePos,
        // The initial ballistic speed of the projectile
        float maxProjectileSpeed,
        // The gravity vector affecting the projectile (likely passed as Physics.gravity)
        Vector3 projectileGravity,
        // The initial position of the target
        Vector3 targetInitPos,
        // The constant velocity of the target (zero acceleration assumed)
        Vector3 targetConstVel,
        // The forward facing direction of the target. Possibly of use if the target
        // velocity is zero
        Vector3 targetForwardDir,
        // For algorithms that approximate the solution, this sets a limit for how far
        // the target and projectile can be from each other at the interceptT time
        // and still count as a successful prediction
        float maxAllowedErrorDist,
        // Output param: The solved projectileDir for ballistic trajectory that intercepts target
        out Vector3 projectileDir,
        // Output param: The speed the projectile is launched at in projectileDir such that
        // there is a collision with target. projectileSpeed must be <= maxProjectileSpeed
        out float projectileSpeed,
        // Output param: The time at which the projectile and target collide
        out float interceptT,
        // Output param: An alternate time at which the projectile and target collide
        // Note that this is optional to use and does NOT coincide with the solved projectileDir
        // and projectileSpeed. It is possibly useful to pass on to an incremental solver.
        // It only exists to simplify compatibility with the ShootingRange
        out float altT)
    {
        var shoot = LoC(projectilePos, maxProjectileSpeed * 0.9f, projectileGravity, targetInitPos, targetConstVel, targetForwardDir, maxAllowedErrorDist, out Vector3 projectileDir2, out float projectileSpeed2, out float interceptT2, out float altT2);
        altT = altT2;
        projectileDir = Vector3.zero;
        interceptT = -1f;
        projectileSpeed = -1f;
        if (!shoot)
        {
            altT = altT2;
            projectileDir = Vector3.zero;
            interceptT = -1f;
            projectileSpeed = -1f;
            return false;
        }
        else
        {
            if (projectileSpeed2 >= maxProjectileSpeed)
            {
                projectileSpeed2 = maxProjectileSpeed;
                var new_v_clamped = projectileDir2 * projectileSpeed2;
                var kin1 = targetInitPos + targetConstVel * interceptT2;
                var kin2 = projectilePos + new_v_clamped * interceptT2 + 1 / 2 * projectileGravity * (interceptT2 * interceptT2);
                if (Vector3.Distance(kin1,kin2) > maxAllowedErrorDist)
                {
                    altT = altT2;
                    projectileDir = Vector3.zero;
                    interceptT = -1f;
                    projectileSpeed = -1f;
                    return false;
                }
                else
                {
                    altT = altT2;
                    projectileDir = projectileDir2;
                    interceptT = interceptT2;
                    projectileSpeed = projectileSpeed2;
                }
            }
            else
            {
                altT = altT2;
                projectileDir = projectileDir2;
                interceptT = interceptT2;
                projectileSpeed = projectileSpeed2;
            }
        }
        return shoot;
    }



    // Simple demo of shared info amongst the team
    // You can modify this as necessary for advanced team strategy
    // Tracking teammates is added to get you started.
    // Also, some expensive queries of opponent and dodgeballs are
    // shared across the team
    public class TeamShare
    {
        public PrisonDodgeballManager.Team Team { get; private set; }
        public MinionScript[] TeamMates { get; private set; }
        public int TeamSize { get; private set; }
        public int NumBalls { get; private set; }
        protected int currTeamMateRegSpot = 0;

        // These are used to track whether data is stale
        protected float timeOfDBQuery = float.MinValue;

        protected PrisonDodgeballManager.DodgeballInfo[] dbInfo;

        public PrisonDodgeballManager.DodgeballInfo[] DBInfo
        {
            get
            {
                var t = Time.timeSinceLevelLoad;

                if (t != timeOfDBQuery)
                {
                    timeOfDBQuery = t;
                    PrisonDodgeballManager.Instance.GetAllDodgeballInfo(Team, ref dbInfo, true);
                }

                return dbInfo;
            }

            private set { dbInfo = value; }
        }

        public TeamShare(PrisonDodgeballManager.Team team, int teamSize, int numBalls)
        {
            Team = team;
            TeamSize = teamSize;
            NumBalls = numBalls;
            TeamMates = new MinionScript[TeamSize];

            DBInfo = new PrisonDodgeballManager.DodgeballInfo[NumBalls];
        }

        public void AddTeamMember(MinionScript m)
        {
            TeamMates[currTeamMateRegSpot] = m;
            ++currTeamMateRegSpot;
        }

        public bool IsFullyInitialized
        {
            get => currTeamMateRegSpot >= TeamSize;
        }

    }

    // Create a base class for our states to have access to the parent MinionStateMachine, and other info
    // This class can be modified!
    abstract class MinionStateBase
    {
        public virtual string Name => throw new System.NotImplementedException();

        protected IFiniteStateMachine<MinionFSMData> ParentFSM;
        protected MinionStateMachine MinionFSM;
        protected MinionScript Minion;
        protected PrisonDodgeballManager Mgr;
        protected PrisonDodgeballManager.Team Team;
        protected TeamShare TeamData;


        public virtual void Init(IFiniteStateMachine<MinionFSMData> parentFSM,
            MinionFSMData minFSMData)
        {
            ParentFSM = parentFSM;
            MinionFSM = minFSMData.MinionFSM;
            Minion = minFSMData.Minion;
            Mgr = minFSMData.Mgr;
            Team = minFSMData.Team;
            TeamData = minFSMData.TeamData;
        }

        // Note: You can add extra methods here that you want to be available to all states


        protected bool FindClosestAvailableDodgeball(
            out PrisonDodgeballManager.DodgeballInfo dodgeballInfo)
        {

            var dist = float.MaxValue;
            bool found = false;

            dodgeballInfo = default;

            if (TeamData == null)
                return false;

            var dbInfo = TeamData.DBInfo;

            if (dbInfo == null)
                return false;

            foreach (var db in dbInfo)
            {
                if (!db.IsHeld && db.State == PrisonDodgeballManager.DodgeballState.Neutral && db.Reachable)
                {
                    var d = Vector3.Distance(db.Pos, Minion.transform.position);

                    if (d < dist)
                    {
                        found = true;
                        dist = d;
                        dodgeballInfo = db;
                    }

                }
            }

            return found;
        }


        public bool FindRescuableTeammate(out MinionScript firstHelplessMinion)
        {

            firstHelplessMinion = null;

            if (TeamData == null)
                return false;

            var teammates = TeamData.TeamMates;

            if (teammates == null)
                return false;

            foreach (var m in teammates)
            {
                if (m == null)
                    continue;

                if (m.CanBeRescued)
                {
                    firstHelplessMinion = m;
                    return true;
                }
            }
            return false;
        }


        protected void InternalEnter()
        {
            MinionFSM.Minion.DisplayText(Name);
        }

        // globalTransition parameter is to notify if transition was triggered
        // by a global transition (wildcard)
        public virtual void Exit(bool globalTransition) { }
        public virtual void Exit() { Exit(false); }

        public virtual DeferredStateTransitionBase<MinionFSMData> Update()
        {
            return null;
        }

    }

    // Create a base class for our states to have access to the parent MinionStateMachine, and other info
    abstract class MinionState : MinionStateBase, IState<MinionFSMData>
    {
        public virtual void Enter() { InternalEnter(); }
    }

    // Create a base class for our states to have access to the parent MinionStateMachine, and other info
    abstract class MinionState<S0> : MinionStateBase, IState<MinionFSMData, S0>
    {
        public virtual void Enter(S0 s) { InternalEnter(); }
    }

    // Create a base class for our states to have access to the parent MinionStateMachine, and other info
    abstract class MinionState<S0, S1> : MinionStateBase, IState<MinionFSMData, S0, S1>
    {
        public virtual void Enter(S0 s0, S1 s1) { InternalEnter(); }
    }

    // If you need MinionState<>s with more parameters (up to four total), you can add them following the pattern above

    // Go get a ball!
    class CollectBallState : MinionState
    {
        public override string Name => CollectBallStateName;

        bool hasDestBall = false;
        PrisonDodgeballManager.DodgeballInfo destBall;

        DeferredStateTransition<MinionFSMData> GoToThrowSpotTransition;
        DeferredStateTransition<MinionFSMData> DefenseDemoTransition;

        public override void Init(IFiniteStateMachine<MinionFSMData> parentFSM, MinionFSMData minFSMData)
        {
            base.Init(parentFSM, minFSMData);

            // create deferred transitions in advanced and reuse them to avoid garbage collection hit during game
            GoToThrowSpotTransition = ParentFSM.CreateStateTransition(GoToThrowSpotStateName);
            DefenseDemoTransition = ParentFSM.CreateStateTransition(DefensiveDemoStateName);
        }

        public override void Enter()
        {
            base.Enter();

            if (FindClosestAvailableDodgeball(out destBall))
            {
                hasDestBall = true;
                Minion.GoTo(destBall.Pos);
            }

        }

        public override void Exit(bool globalTransition)
        {

        }

        public override DeferredStateTransitionBase<MinionFSMData> Update()
        {
            DeferredStateTransitionBase<MinionFSMData> ret = null;

            // could pick up a ball accidentally before getting to desired ball
            if (Minion.HasBall)
                return GoToThrowSpotTransition;

            var dbInfo = TeamData.DBInfo;

            if (dbInfo == null)
                return null;

            if (hasDestBall)
            {
                destBall = dbInfo[destBall.Index];

                if (destBall.IsHeld || destBall.State != PrisonDodgeballManager.DodgeballState.Neutral || !destBall.Reachable)
                {
                    hasDestBall = false;
                }

            }

            if (!hasDestBall)
            {
                if (FindClosestAvailableDodgeball(out destBall))
                {
                    hasDestBall = true;

                }
            }

            if (hasDestBall)
            {
                // The ball might be moving, so keep updating. GoTo() is smart enough
                // to not keep performing full A* if it doesn't need to, so safe to call often.
                Minion.GoTo(destBall.NavMeshPos);
            }
            else
            {
                // No ball, so focus on defense
                ret = DefenseDemoTransition;
            }

            return ret;
        }
    }


    // This state gets the minion close to the enemy for a throw (or a rescue of a buddy)
    class GoToThrowSpotState : MinionState
    {

        public override string Name => GoToThrowSpotStateName;

        int opponentIndex = -1;
        PrisonDodgeballManager.OpponentInfo opponentInfo;
        bool hasOpponent = false;
        DeferredStateTransition<MinionFSMData> CollectBallTransition;
        DeferredStateTransition<MinionFSMData, MinionScript> RescueTransition;
        DeferredStateTransition<MinionFSMData> ThrowBallTransition;
        DeferredStateTransition<MinionFSMData> DefenseDemoTransition;

        public override void Init(IFiniteStateMachine<MinionFSMData> parentFSM, MinionFSMData minFSMData)
        {
            base.Init(parentFSM, minFSMData);

            CollectBallTransition = ParentFSM.CreateStateTransition(CollectBallStateName);
            RescueTransition = ParentFSM.CreateStateTransition<MinionScript>(RescueStateName, null, true);
            ThrowBallTransition = ParentFSM.CreateStateTransition(ThrowBallStateName);
            DefenseDemoTransition = ParentFSM.CreateStateTransition(DefensiveDemoStateName);
        }

        public override void Enter()
        {
            base.Enter();

            Minion.GoTo(Mgr.TeamAdvance(Team).position);
        }

        public override void Exit(bool globalTransition)
        {

        }

        public override DeferredStateTransitionBase<MinionFSMData> Update()
        {
            DeferredStateTransitionBase<MinionFSMData> ret = null;

            // just in case something bad happened
            if (!Minion.HasBall)
            {
                return CollectBallTransition;
            }
            // Check if opponent still valid
            if (
                !(hasOpponent = Mgr.GetOpponentInfo(Team, opponentIndex, out opponentInfo)) ||
                opponentInfo.IsPrisoner || opponentInfo.IsFreedPrisoner)
            {
                if (Mgr.FindClosestNonPrisonerOpponentIndex(Minion.transform.position, Team, out opponentIndex))
                {
                    hasOpponent = Mgr.GetOpponentInfo(Team, opponentIndex, out opponentInfo);
                }
            }
            //Shoot them!! they're in my radius
            if (Mathf.Abs(Vector3.Distance(Minion.transform.position, opponentInfo.Pos)) < 15.5f)
            {
                return ThrowBallTransition;
            }

            // Nothing to do without opponent...
            if (!hasOpponent)
                return DefenseDemoTransition;


            if (Minion.ReachedTarget())
            {
                if (FindRescuableTeammate(out var m))
                {
                    RescueTransition.Arg0 = m;
                    if (m.transform.forward.magnitude != 1f)
                    {
                        ret = ThrowBallTransition;

                    }
                    else
                    {
                        ret = RescueTransition;
                    }
                }
                else
                    ret = ThrowBallTransition;
            }

            return ret;
        }
    }


    // Rescue a buddy
    class RescueState : MinionState<MinionScript>
    {
        public override string Name => RescueStateName;

        MinionScript buddy;

        DeferredStateTransition<MinionFSMData> CollectBallTransition;
        DeferredStateTransition<MinionFSMData> ThrowBallTransition;

        public override void Init(IFiniteStateMachine<MinionFSMData> parentFSM, MinionFSMData minFSMData)
        {
            base.Init(parentFSM, minFSMData);

            CollectBallTransition = ParentFSM.CreateStateTransition(CollectBallStateName);
            ThrowBallTransition = ParentFSM.CreateStateTransition(ThrowBallStateName);
        }

        public override void Enter(MinionScript m)
        {
            base.Enter(m);

            buddy = m;

            Minion.FaceTowards(buddy.transform.position);

        }

        public override void Exit(bool globalTransition)
        {

        }

        public override DeferredStateTransitionBase<MinionFSMData> Update()
        {
            DeferredStateTransitionBase<MinionFSMData> ret = null;

            // just in case something bad happened
            if (!Minion.HasBall)
            {
                return CollectBallTransition;
            }

            if (buddy == null || !buddy.CanBeRescued)
            {

                if (FindRescuableTeammate(out buddy))
                {
                    buddy = null;
                }

            }

            // Nothing to do without buddy in prison...
            if (buddy == null)
                return ThrowBallTransition; // we should have a ball still...


            var canThrow = PredictThrow(Minion.HeldBallPosition, Minion.ThrowSpeed, Physics.gravity, buddy.transform.position,
                    buddy.Velocity, buddy.transform.forward, MaxAllowedThrowPositionError,
                    out var univVDir, out var speedScalar, out var interceptT, out var altT);


            var intercept = Minion.HeldBallPosition + univVDir * speedScalar * interceptT;
            Minion.FaceTowardsForThrow(intercept);

            if (canThrow)
            {
                var speedNorm = speedScalar / Minion.ThrowSpeed;

                if (Minion.ThrowBall(univVDir, speedNorm))
                    ret = CollectBallTransition;
            }

            return ret;
        }
    }


    // Throw the ball at the enemy
    class ThrowBallState : MinionState
    {
        public override string Name => ThrowBallStateName;

        int opponentIndex = -1;
        PrisonDodgeballManager.OpponentInfo opponentInfo;
        bool hasOpponent = false;
        private Vector3 lastAcc;
        private Vector3 prevVel;
        private Vector3 prevPos;

        DeferredStateTransition<MinionFSMData> CollectBallTransition;
        DeferredStateTransition<MinionFSMData> DefenseDemoTransition;

        public override void Init(IFiniteStateMachine<MinionFSMData> parentFSM, MinionFSMData minFSMData)
        {
            base.Init(parentFSM, minFSMData);

            // create deferred transitions in advanced and reuse them to avoid garbage collection hit during game
            CollectBallTransition = ParentFSM.CreateStateTransition(CollectBallStateName);
            DefenseDemoTransition = ParentFSM.CreateStateTransition(DefensiveDemoStateName);
        }


        public override void Enter()
        {
            base.Enter();


            if (Mgr.FindClosestNonPrisonerOpponentIndex(Minion.transform.position, Team, out opponentIndex))
            {
                if (hasOpponent = Mgr.GetOpponentInfo(Team, opponentIndex, out opponentInfo))
                {
                    Minion.FaceTowards(opponentInfo.Pos);
                }
            }
        }

        public override void Exit(bool globalTransition)
        {

        }

        public override DeferredStateTransitionBase<MinionFSMData> Update()
        {
            DeferredStateTransitionBase<MinionFSMData> ret = null;

            // just in case something bad happened
            if (!Minion.HasBall)
            {
                return CollectBallTransition;
            }

            // Check if opponent still valid
            if (
                !(hasOpponent = Mgr.GetOpponentInfo(Team, opponentIndex, out opponentInfo)) ||
                opponentInfo.IsPrisoner || opponentInfo.IsFreedPrisoner)
            {

                if (Mgr.FindClosestNonPrisonerOpponentIndex(Minion.transform.position, Team, out opponentIndex))
                {
                    hasOpponent = Mgr.GetOpponentInfo(Team, opponentIndex, out opponentInfo);
                }
            }

            // Nothing to do without opponent...
            if (!hasOpponent)
                return DefenseDemoTransition;


            var canThrow = PredictThrow(Minion.HeldBallPosition, Minion.ThrowSpeed, Physics.gravity,
                opponentInfo.Pos, opponentInfo.Vel, opponentInfo.Forward, MaxAllowedThrowPositionError,
                out var univVDir, out var speedScalar, out var interceptT, out var altT);


            var intercept = Minion.HeldBallPosition + univVDir * speedScalar * interceptT;
            Minion.FaceTowardsForThrow(intercept);

            //Shot selection
            var opVel = (opponentInfo.Pos - prevPos) / interceptT;
            var acel = (opVel - prevVel) / interceptT;
            //Debug.Log(Mathf.Abs(acel.magnitude - lastAcc.magnitude));
            if (Mathf.Abs(acel.magnitude-lastAcc.magnitude) < 0.10f)
            {
                canThrow = true;
            }
            //Acceleratiing
            else if(acel.magnitude > lastAcc.magnitude)
            {
                canThrow = false;
            }

            lastAcc = acel;
            prevVel = opponentInfo.Vel;
            prevPos = opponentInfo.Pos;
            if (canThrow)
            {
                var speedNorm = speedScalar / Minion.ThrowSpeed;

                if (Minion.ThrowBall(univVDir, speedNorm))
                    ret = CollectBallTransition;
            }


            return ret;
        }
    }


    // A not very effective defensive strategy. Mainly a demonstration of calling
    // Minion.Evade()
    class DefensiveDemoState : MinionState
    {
        public override string Name => DefensiveDemoStateName;

        float lastEvade;
        float evadeWaitTimeSec;
        bool doPause = false;
        float pauseStart;
        float pauseDuration;

        DeferredStateTransition<MinionFSMData> GoToThrowSpotTransition;
        DeferredStateTransition<MinionFSMData> CollectBallTransition;

        public override void Init(IFiniteStateMachine<MinionFSMData> parentFSM, MinionFSMData minFSMData)
        {
            base.Init(parentFSM, minFSMData);

            // create deferred transitions in advanced and reuse them to avoid garbage collection hit during game
            GoToThrowSpotTransition = ParentFSM.CreateStateTransition(GoToThrowSpotStateName);
            CollectBallTransition = ParentFSM.CreateStateTransition(CollectBallStateName);
        }


        protected bool RandomGoTo()
        {
            var r = Minion.GoTo(Mgr.TeamHome(Team).position + 6f * (new Vector3(Random.value, 0f, Random.value)));

            if (!r)
            {
                Debug.LogWarning("Could not GOTO in DefenseDemoState");
            }

            return r;
        }



        public override void Enter()
        {
            base.Enter();

            RandomGoTo();

            lastEvade = Time.timeSinceLevelLoad;

            evadeWaitTimeSec = 2f * Minion.EvadeCoolDownTimeSec + 0.1f;
        }

        public override void Exit(bool globalTransition)
        {

        }

        public override DeferredStateTransitionBase<MinionFSMData> Update()
        {
            DeferredStateTransitionBase<MinionFSMData> ret = null;

            if (Minion.HasBall)
                return GoToThrowSpotTransition;

            PrisonDodgeballManager.DodgeballInfo ball;

            if (FindClosestAvailableDodgeball(out ball))
            {
                return CollectBallTransition;
            }

            if (!doPause && Minion.ReachedTarget())
            {
                pauseStart = Time.timeSinceLevelLoad;
                doPause = true;
                pauseDuration = Random.value * 3f;
            }

            if (doPause)
            {
                Minion.FaceTowards(Mgr.TeamPrison(Team).position);

                if (Time.timeSinceLevelLoad - pauseStart >= pauseDuration)
                {
                    doPause = false;
                    RandomGoTo();
                }
            }
            else if (Time.timeSinceLevelLoad - lastEvade >= evadeWaitTimeSec)
            {

                lastEvade = Time.timeSinceLevelLoad;

                var r = Random.Range(0, 3);

                MinionScript.EvasionDirection ev;

                switch (r)
                {
                    case 0:
                        ev = MinionScript.EvasionDirection.Brake;
                        break;
                    case 1:
                        ev = MinionScript.EvasionDirection.Left;
                        break;
                    case 2:
                        ev = MinionScript.EvasionDirection.Right;
                        break;
                    default:
                        ev = MinionScript.EvasionDirection.Brake;
                        break;
                }

                Minion.Evade(ev, Random.Range(0.6f, 1.0f));
            }


            return ret;
        }
    }


    // Go directly to jail. Do not pass go. Do not collect $200 
    class GoToPrisonState : MinionState
    {
        public override string Name => GoToPrisonStateName;

        int waypointIndex = 0;

        DeferredStateTransition<MinionFSMData> LeavePrisonTransition;

        public override void Init(IFiniteStateMachine<MinionFSMData> parentFSM, MinionFSMData minFSMData)
        {
            base.Init(parentFSM, minFSMData);

            // create deferred transitions in advanced and reuse them to avoid garbage collection hit during game
            LeavePrisonTransition = ParentFSM.CreateStateTransition(LeavePrisonStateName);
        }


        public override void Enter()
        {
            base.Enter();

            waypointIndex = 0;

            Minion.GoTo(Mgr.TeamGutterEntranceLeft(Team).position);
        }

        public override void Exit(bool globalTransition)
        {

        }

        public override DeferredStateTransitionBase<MinionFSMData> Update()
        {
            DeferredStateTransitionBase<MinionFSMData> ret = null;

            if (!Minion.IsPrisoner)
            {
                return LeavePrisonTransition;
                //if (Minion.HasBall)
                //    return GoToThrowSpotBallStateName;
                //else
                //    return GoHomeStateName;
            }

            if (Minion.ReachedTarget())
            {
                if (waypointIndex == 0)
                {
                    ++waypointIndex;
                    Minion.GoTo(Mgr.TeamGutterEndLeft(Team).position);
                }
                else if (waypointIndex == 1)
                {
                    ++waypointIndex;
                    Minion.GoTo(Mgr.TeamPrison(Team).position);
                }
                else
                {
                    Minion.FaceTowards(Mgr.TeamHome(Team).position);
                }
            }

            return ret;
        }
    }

    // Free! 
    class LeavePrisonState : MinionState
    {
        public override string Name => LeavePrisonStateName;

        int waypointIndex = 0;

        DeferredStateTransition<MinionFSMData> GoToThrowSpotTransition;
        DeferredStateTransition<MinionFSMData> GoHomeTransition;

        public override void Init(IFiniteStateMachine<MinionFSMData> parentFSM, MinionFSMData minFSMData)
        {
            base.Init(parentFSM, minFSMData);

            // create deferred transitions in advanced and reuse them to avoid garbage collection hit during game
            GoToThrowSpotTransition = ParentFSM.CreateStateTransition(GoToThrowSpotStateName);
            GoHomeTransition = ParentFSM.CreateStateTransition(GoHomeStateName);
        }


        public override void Enter()
        {
            base.Enter();

            waypointIndex = 0;

            Minion.GoTo(Mgr.TeamGutterEndRight(Team).position);
        }

        public override void Exit(bool globalTransition)
        {

        }

        public override DeferredStateTransitionBase<MinionFSMData> Update()
        {
            DeferredStateTransitionBase<MinionFSMData> ret = null;

            if (Minion.ReachedTarget())
            {
                if (waypointIndex == 0)
                {
                    ++waypointIndex;
                    Minion.GoTo(Mgr.TeamGutterEntranceRight(Team).position);
                }
                else
                {
                    if (Minion.HasBall)
                        return GoToThrowSpotTransition;
                    else
                        return GoHomeTransition;

                }
            }

            return ret;
        }
    }


    // Going home. Maybe after a jailbreak
    class GoHomeState : MinionState
    {
        public override string Name => GoHomeStateName;

        DeferredStateTransition<MinionFSMData> CollectBallTransition;

        public override void Init(IFiniteStateMachine<MinionFSMData> parentFSM, MinionFSMData minFSMData)
        {
            base.Init(parentFSM, minFSMData);

            // create deferred transitions in advanced and reuse them to avoid garbage collection hit during game
            CollectBallTransition = ParentFSM.CreateStateTransition(CollectBallStateName);
        }


        public override void Enter()
        {
            base.Enter();

            if (!Minion.GoTo(Mgr.TeamHome(Team).position))
            {
                Debug.LogWarning($"Could not find a way home! NavMesh Mask: {Minion.NavMeshMaskToString()}");
            }
        }

        public override void Exit(bool globalTransition)
        {

        }

        public override DeferredStateTransitionBase<MinionFSMData> Update()
        {
            DeferredStateTransitionBase<MinionFSMData> ret = null;

            if (Minion.ReachedTarget())
            {
                ret = CollectBallTransition;
            }

            return ret;
        }
    }


    class RestState : MinionState
    {
        public override string Name => RestStateName;

        public override void Enter()
        {
            base.Enter();

            if (!Minion.GoTo(Mgr.TeamHome(Team).position))
            {
                Debug.LogWarning($"Could not find a way home! NavMesh Mask: {Minion.NavMeshMaskToString()}");
            }
        }

        public override DeferredStateTransitionBase<MinionFSMData> Update()
        {
            DeferredStateTransitionBase<MinionFSMData> ret = null;

            return ret;
        }
    }


    // This is a special state that never exits. It coexists with the current state.
    // It's always evaluated first. It's only job is supposed to identify global/wildcard
    // transitions (it shouldn't do anything that modifies anything externally other than
    // return a desired transition).
    class GlobalTransitionState : MinionState
    {
        public override string Name => GlobalTransitionStateName;

        bool wasPrisioner = false;

        DeferredStateTransition<MinionFSMData> RestTransition;
        DeferredStateTransition<MinionFSMData> PrisonTransition;

        public override void Init(IFiniteStateMachine<MinionFSMData> parentFSM, MinionFSMData minFSMData)
        {
            base.Init(parentFSM, minFSMData);

            // create deferred transitions in advanced and reuse them to avoid garbage collection hit during game
            RestTransition = ParentFSM.CreateStateTransition(RestStateName);
            PrisonTransition = ParentFSM.CreateStateTransition(GoToPrisonStateName);
        }


        public override void Enter()
        {
            base.Enter();
        }

        // The global state never exits
        //public override void Exit(bool globalTransition)
        //{
        //}

        public override DeferredStateTransitionBase<MinionFSMData> Update()
        {
            DeferredStateTransitionBase<MinionFSMData> ret = null;

            if (Mgr.IsGameOver && !ParentFSM.CurrentState.Name.Equals(RestStateName))
            {
                ret = RestTransition;
            }
            else if (Minion.IsPrisoner && !wasPrisioner)
            {
                // Just switched to prisoner! Uh oh. Gotta head to prison. :-(
                ret = PrisonTransition;

                wasPrisioner = true;
            }
            else if (!Minion.IsPrisoner && wasPrisioner)
            {
                wasPrisioner = false;
            }

            return ret;
        }
    }


    private void Awake()
    {
        Minion = GetComponent<MinionScript>();

        if (Minion == null)
        {
            Debug.LogWarning("No minion script");
        }
    }


    protected void InitTeamData()
    {
        Mgr.SetTeamText(Minion.Team, StudentName);

        var o = Mgr.GetTeamDataShare(Minion.Team);

        if (o == null)
        {
            TeamData = new TeamShare(Minion.Team, Mgr.TeamSize, Mgr.TotalBalls);
            Mgr.SetTeamDataShare(Minion.Team, TeamData);
        }
        else
        {
            TeamData = o as TeamShare;

            if (TeamData == null)
            {
                Debug.LogWarning("TeamData is null!");
            }

        }

        TeamData.AddTeamMember(Minion);
    }


    // Start is called before the first frame update
    protected void Start()
    {

        Mgr = PrisonDodgeballManager.Instance;

        InitTeamData();

        var minionFSMData = new MinionFSMData(this, Minion, Mgr, Minion.Team, TeamData);

        fsm = new FiniteStateMachine<MinionFSMData>(minionFSMData);

        // Handles global/wildcard transitions. This state is a co-state that
        // never exits. Triggered transitions only change the current state.
        // The global state should only handle initiating transitions
        fsm.SetGlobalTransitionState(new GlobalTransitionState());

        fsm.AddState(new CollectBallState(), true);
        fsm.AddState(new GoToThrowSpotState());
        fsm.AddState(new ThrowBallState());
        fsm.AddState(new DefensiveDemoState());
        fsm.AddState(new GoToPrisonState());
        fsm.AddState(new LeavePrisonState());
        fsm.AddState(new GoHomeState());
        fsm.AddState(new RescueState());
        fsm.AddState(new RestState());

        //MinionStateMachine, GameAIStudentWork, Version=0.0.0.0, Culture=neutral, PublicKeyToken=null
        //Debug.Log(this.GetType().AssemblyQualifiedName);

    }

    protected void Update()
    {
        // Don't start until all the team is ready to go
        if (TeamData == null || !TeamData.IsFullyInitialized)
            return;

        fsm.Update();

        // For debugging, could repurpose the DisplayText of the Minion.
        // To do so affecting all states, implement the FSM's Update like so:
        //Minion.DisplayText(Minion.NavMeshCurrentSurfaceToString());

    }

}
