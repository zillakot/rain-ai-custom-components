using System;
using RAIN.Motion;
using RAIN.Navigation.Graph;
using RAIN.Navigation.Pathfinding;
using RAIN.Serialization;
using UnityEngine;

//http://rivaltheory.com/forums/topic/custom-navigator-for-a-pathfinding-project/page/2/#post-36182
namespace RAIN.Navigation
{
	[RAINSerializableClass]
	public class AStarNavigator : RAINNavigator, IDisposable
    {
        #region Not used at this time
        public override RAINPath CurrentPath { get; set; }
        public override RAINNavigationGraph CurrentGraph { get; set; }
        #endregion

        public override bool IsPathfinding { get { return _aStarAIPath.CanSearchAgain; } }
        public override int NextWaypoint { get; set; }

        private AstarAIPath _aStarAIPath;
        private Waypoints.WaypointSet wpSet;
        private Waypoints.Waypoint wp;

        public override void BodyInit()
        {
            base.BodyInit();

            //Get the custom AStarAIPath script used to communicate with AIPath and Seeker.
            _aStarAIPath = base.AI.Body.GetComponentInChildren<AstarAIPath>();

            if (_aStarAIPath == null)
                Debug.Log("You must assign the script type AstarAIPath.");

            _aStarAIPath.endReachedDistance = base.AI.Motor.DefaultCloseEnoughDistance;
            _aStarAIPath.speed = base.AI.Motor.DefaultSpeed;
            _aStarAIPath.turningSpeed = base.AI.Motor.RotationSpeed;

            //Get the WayPoint Set from the RAIN Navigation manager instance
            var wpSetName = _aStarAIPath.WayPointRouteName;
            if (string.IsNullOrEmpty(wpSetName))
                Debug.Log("You must provide the name of the waypoint route to use.");

            wpSet = NavigationManager.Instance.GetWaypointSet(wpSetName);

            //Instead of using the RAIN path, we'll set the property of the next waypoint
            NextWaypoint = 0;

            //Get the next waypoint from the waypoint set 
            getNextWayPoint();

            //Subscribe to arrived at target event
            AstarAIPath.ArrivedAtTarget += getNextWayPoint;
        }

        #region Required when deriving from RAINNavigator. Currently not used
        public override Vector3 ClosestPointOnGraph(Vector3 aPosition, float aMaxYOffset = 0.0f)
        {
            return aPosition;
        }

        public override bool GetPathToMoveTarget(bool allowOffGraphMovement, out RAINPath path)
        {
            path = null;
            return true;
        }

        public override bool GetPathTo(Vector3 position, int maxPathfindSteps, bool allowOffGraphMovement, out RAINPath path)
        {
            path = null;
            return true;

        }

        public override void RestartPathfindingSearch()
        {

        }

        public override bool OnGraph(Vector3 aPosition, float aMaxYOffset = 0.0f)
        {

            return true;
        }
        #endregion

        //Destructor
        ~AstarNavigator(){
            Dispose();
        }

        //Clean up
        public void Dispose()
        {
            AstarAIPath.ArrivedAtTarget -= getNextWayPoint;
        }

        //Get current waypoint and increment count
        private void getNextWayPoint()
        {
            wp = wpSet.Waypoints[NextWaypoint];
            NextWaypoint++;

            if (NextWaypoint == wpSet.Waypoints.Count)
                NextWaypoint = 0;
        }

        public override bool IsAt(MoveLookTarget aPosition)
        {
            var fMaxCloseEnoughDistance = Mathf.Max(aPosition.CloseEnoughDistance, AI.Motor.CloseEnoughDistance);

            bool flag = false;

            if (AI.Motor.Allow3DMovement)
            {
                var vector2 = AI.Kinematic.Position - aPosition.Position;

                if (vector2.magnitude <= fMaxCloseEnoughDistance)
                {
                    flag = true;
                }
            }
            else
            {
                var position = aPosition.Position;
                position.y = AI.Kinematic.Position.y;
                var vector3 = AI.Kinematic.Position - position;

                if (vector3.magnitude <= fMaxCloseEnoughDistance)
                {              
                    flag = true;
                }
            }

            return flag;
        }

        public override MoveLookTarget GetNextPathWaypoint(bool allow3DMovement, bool allowOffGraphMovement, MoveLookTarget moveLookTarget)
        {
            if (moveLookTarget == null)
            {
                moveLookTarget = new RAIN.Motion.MoveLookTarget();
            }

            if ((base.pathTarget == null) || !base.pathTarget.IsValid)
            {
                moveLookTarget.TargetType = MoveLookTarget.MoveLookTargetType.None;
                return moveLookTarget;
            }

            if (_aStarAIPath.canMove)
            {
                _aStarAIPath.TargetPosition = wp.position;

                moveLookTarget.VectorTarget = _aStarAIPath.TargetPoint;
            }

            return moveLookTarget;
        }

        public override bool GetPathToMoveTarget(MoveLookTarget aPathTarget, bool aAllowOffGraphMovement, out RAINPath aPath)
        {
            throw new NotImplementedException();
        }

        public override bool GetPathTo(Vector3 aPosition, int aMaxPathfindingSteps, float aMaxPathLength, bool aAllowOffGraphMovement, out RAINPath aPath)
        {
            throw new NotImplementedException();
        }

        public override MoveLookTarget GetNextPathWaypoint(MoveLookTarget aPathTarget, bool aAllow3DMovement, bool aAllowOffGraphMovement, MoveLookTarget aCachedMoveLookTarget = null)
        {
            throw new NotImplementedException();
        }
    }
}