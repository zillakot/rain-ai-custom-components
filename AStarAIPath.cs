using UnityEngine;
using System.Collections;
using Pathfinding;
using System.Collections.Generic;
using RAIN.Core;
using RAIN.Navigation.Pathfinding;
using RAIN.Navigation;
using Pathfinding.RVO;
using System;
/*
 * http://www.youtube.com/watch?feature=player_embedded&v=PUJSvd53v4k - YT vid
 * http://arongranberg.com/astar/docs/getstarted.php - Aron A* project
 * http://rivaltheory.com/forums/topic/custom-navigator-for-a-pathfinding-project/page/2/ - Rain blog
 */
[RequireComponent(typeof(Seeker))]
[AddComponentMenu("Pathfinding/AI/AIPath (generic)")]
public class AStarAIPath : MonoBehaviour
{
    #region Used in RAIN custom navigator
    public delegate void EndPathFinding();
    public static event EndPathFinding ArrivedAtTarget;

    public string WayPointRouteName = string.Empty;
    public Vector3 TargetPosition;

    /** Maximum velocity.
    * This is the maximum speed in world units per second.
    */
    [HideInInspector]
    public float speed = 3;

    /** Distance to the end point to consider the end of path to be reached.
    * When this has been reached, the AI will not move anymore until the target changes and OnTargetReached will be called.
    */
    [HideInInspector]
    public float endReachedDistance = 0.2F;

    /** Rotation speed.
    * Rotation is calculated using Quaternion.SLerp. This variable represents the damping, the higher, the faster it will be able to rotate.
    */
    [HideInInspector]
    public float turningSpeed = 5;

    /** Only when the previous path has been returned should be search for a new path */
    [HideInInspector]
    public bool CanSearchAgain = true;

    // Enables or disables movement.
    public bool canMove = true;

    protected Vector3 targetPoint;
    public Vector3 TargetPoint
    {
        get { return targetPoint; }
    }
    public virtual void OnTargetReached()
    {
        if (ArrivedAtTarget != null)
        {
            ArrivedAtTarget();
        }
    }
    #endregion

    #region Not integrated with RAIN yet
    public float forwardLook = 1;
    public float repathRate = 0.5F;
    public bool canSearch = true;
    public bool closestOnPathCheck = true;
    public float pickNextWaypointDist = 2;
    public float slowdownDistance = 0.6F;
    public Path Path
    {
        get { return path; }
    }

    protected Vector3 targetDirection;
    protected Transform tr;
    protected float minMoveScale = 0.05F;
    protected Path path;
    protected Seeker seeker;
    protected float lastRepath = -9999;
    protected int currentWaypointIndex = 0;
    protected Vector3 lastFoundWaypointPosition;
    protected float lastFoundWaypointTime = -9999;

    private bool _startHasRun = false;
    #endregion

	protected virtual void Awake()
	{
		seeker = GetComponent<Seeker>();
		tr = transform.parent;
	}
	
	protected virtual void Start()
	{
		_startHasRun = true;
		OnEnable();
	}
	
	protected virtual void OnEnable()
	{
		lastRepath = -9999;
		CanSearchAgain = true;
	
		lastFoundWaypointPosition = GetFeetPosition();
	
		if (_startHasRun)
		{
			seeker.pathCallback += OnPathComplete;
	
			StartCoroutine(RepeatTrySearchPath());
		}
	}
	
	public void OnDisable()
	{
		if (seeker != null && !seeker.IsDone()) seeker.GetCurrentPath().Error();
		
		if (path != null) path.Release(this);
		path = null;
		
		seeker.pathCallback -= OnPathComplete;
	}
	
	public virtual Vector3 GetFeetPosition()
	{
		return tr.position;
	}
	
	protected bool targetReached = false;
	public bool TargetReached
	{
		get
		{
			return targetReached;
		}
	}
	
	protected IEnumerator RepeatTrySearchPath()
	{
		while (true)
		{
			float v = TrySearchPath();
			yield return new WaitForSeconds(v);
		}
	}
	
	public float TrySearchPath()
	{
		if (Time.time - lastRepath >= repathRate && CanSearchAgain && canSearch && (TargetPosition != Vector3.zero)
		{
			SearchPath();
			return repathRate;
		}
		else
		{
			float v = repathRate - (Time.time - lastRepath);
			return v < 0 ? 0 : v;
		}
	}
	
	public virtual void SearchPath()
	{
		lastRepath = Time.time;
		
		CanSearchAgain = false;
		
		seeker.StartPath(GetFeetPosition(), TargetPosition);
	}
	
	public virtual void OnPathComplete(Path _p)
	{
		ABPath p = _p as ABPath;
		if (p == null) throw new System.Exception("This function only handles ABPaths, do not use special path types");
		
		p.Claim(this);
		
		if (p.error)
		{
			p.Release(this);
			return;
		}
		
		if (path != null) path.Release(this);
		
		path = p;
		
		currentWaypointIndex = 0;
		targetReached = false;
		CanSearchAgain = true;
		
		if (closestOnPathCheck)
		{
			Vector3 p1 = Time.time - lastFoundWaypointTime < 0.3f ? lastFoundWaypointPosition : p.originalStartPoint;
			Vector3 p2 = GetFeetPosition();
			Vector3 dir = p2 - p1;
			float magn = dir.magnitude;
			dir /= magn;
			int steps = (int)(magn / pickNextWaypointDist);
			
			#if ASTARDEBUG
			Debug.DrawLine (p1,p2,Color.red,1);
			#endif
			
			for (int i = 0; i <= steps; i++)
			{
				CalculateVelocity(p1);
				p1 += dir;
			}
		}
	}
	
	protected float XZSqrMagnitude(Vector3 a, Vector3 b)
	{
		float dx = b.x - a.x;
		float dz = b.z - a.z;
		return dx * dx + dz * dz;
	}
	
	protected Vector3 CalculateTargetPoint(Vector3 p, Vector3 a, Vector3 b)
	{
		a.y = p.y;
		b.y = p.y;
		
		float magn = (a - b).magnitude;
		if (magn == 0) return a;
		
		float closest = AstarMath.Clamp01(AstarMath.NearestPointFactor(a, b, p));
		Vector3 point = (b - a) * closest + a;
		float distance = (point - p).magnitude;
		
		float lookAhead = Mathf.Clamp(forwardLook - distance, 0.0F, forwardLook);
		
		float offset = lookAhead / magn;
		offset = Mathf.Clamp(offset + closest, 0.0F, 1.0F);
		return (b - a) * offset + a;
	}
	
	protected Vector3 CalculateVelocity(Vector3 currentPosition)
	{
		if (path == null || path.vectorPath == null || path.vectorPath.Count == 0) return Vector3.zero;
		
		List<Vector3> vPath = path.vectorPath;
		
		if (vPath.Count == 1)
		{
			vPath.Insert(0, currentPosition);
		}
		
		if (currentWaypointIndex >= vPath.Count) { currentWaypointIndex = vPath.Count - 1; }
		
		if (currentWaypointIndex <= 1) currentWaypointIndex = 1;
		
		while (true)
		{
			if (currentWaypointIndex < vPath.Count - 1)
			{
				float dist = XZSqrMagnitude(vPath[currentWaypointIndex], currentPosition);
			
				if (dist < pickNextWaypointDist * pickNextWaypointDist)
				{
					lastFoundWaypointPosition = currentPosition;
					lastFoundWaypointTime = Time.time;
					currentWaypointIndex++;
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}	
		}
		
		Vector3 dir = vPath[currentWaypointIndex] - vPath[currentWaypointIndex - 1];
		Vector3 targetPosition = CalculateTargetPoint(currentPosition, vPath[currentWaypointIndex - 1], vPath[currentWaypointIndex]);
		
		dir = targetPosition - currentPosition;
		dir.y = 0;
		float targetDist = dir.magnitude;
		
		float slowdown = Mathf.Clamp01(targetDist / slowdownDistance);
		
		this.targetDirection = dir;
		this.targetPoint = targetPosition;
		
		if (currentWaypointIndex == vPath.Count - 1 && targetDist <= endReachedDistance)
		{
			if (!targetReached) { targetReached = true; OnTargetReached(); }
			
			return Vector3.zero;
		}
		
		Vector3 forward = tr.forward;
		float dot = Vector3.Dot(dir.normalized, forward);
		float sp = speed * Mathf.Max(dot, minMoveScale) * slowdown;
		
		#if ASTARDEBUG
		Debug.DrawLine (vPath[currentWaypointIndex-1] , vPath[currentWaypointIndex],Color.black);
		Debug.DrawLine (GetFeetPosition(),targetPosition,Color.red);
		Debug.DrawRay (targetPosition,Vector3.up, Color.red);
		Debug.DrawRay (GetFeetPosition(),dir,Color.yellow);
		Debug.DrawRay (GetFeetPosition(),forward*sp,Color.cyan);
		#endif
		
		if (Time.deltaTime > 0)
		{
			sp = Mathf.Clamp(sp, 0, targetDist / (Time.deltaTime * 2));
		}
		return forward * sp;
	}
	
	protected virtual void RotateTowards(Vector3 dir)
	{
	
	if (dir == Vector3.zero) return;
	
	Quaternion rot = tr.rotation;
	Quaternion toTarget = Quaternion.LookRotation(dir);
	
	rot = Quaternion.Slerp(rot, toTarget, turningSpeed * Time.deltaTime);
	Vector3 euler = rot.eulerAngles;
	euler.z = 0;
	euler.x = 0;
	rot = Quaternion.Euler(euler);
	
	tr.rotation = rot;
	}
	
	public virtual void Update()
	{
	if (!canMove) { return; }
	
	if (path == null || path.vectorPath == null || path.vectorPath.Count == 0) return;
	
	List<Vector3> vPath = path.vectorPath;
	Vector3 currentPosition = GetFeetPosition();
	
	if (vPath.Count == 1)
	{
	vPath.Insert(0, currentPosition);
	}
	
	if (currentWaypointIndex >= vPath.Count) { currentWaypointIndex = vPath.Count – 1; }
	
	if (currentWaypointIndex <= 1) currentWaypointIndex = 1;
	
	this.targetPoint = vPath[currentWaypointIndex];
	
	while (true)
	{
	if (currentWaypointIndex < vPath.Count - 1)
	{
	float dist = XZSqrMagnitude(vPath[currentWaypointIndex], currentPosition);
	
	if (dist < pickNextWaypointDist * pickNextWaypointDist)
	{
	currentWaypointIndex++;
	}
	else
	{
	break;
	}
	}
	else
	{
	break;
	}
	}
	}

}