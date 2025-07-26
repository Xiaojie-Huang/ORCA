using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

// 速度障碍域数据结构
class VelocityObstacle
{
    public Vector3 collisionPoint;   // 预测碰撞点位置
    public float coneAngle;          // 锥形区域角度（弧度）
    public Vector3 leftBoundary;     // 锥形左边界方向
    public Vector3 rightBoundary;    // 锥形右边界方向
}


public class VOAvoiding : MonoBehaviour
{
    [Header("移动参数")]
    public float maxSpeed = 10f;          // 移动速度
    public float agentRadius = 0.5f;


    [Header("避障参数")] 
    [Range(0.5f, 5.0f)] 
    public float timeHorizon = 2f;
    public float rotationSmoothness = 5f;          // 方向平滑系数
    
    [Header("目标设置")]
    public Transform target;              // 目标位置
    public float targetReachDistance = 0.5f;  // 到达目标的距离
    
    [Header("可视化")]
    public bool drawVelocityArrows = true;
    public Color preferredVelocityColor = Color.green;
    
    //私有变量
    private List<VOAvoiding> otherAgents = new List<VOAvoiding>();
    private Vector3 currentVelocity = Vector3.zero;
    private Vector3 preferredVelocity = Vector3.zero;
    private bool hasTarget = false;
    
    
    // Start is called before the first frame update
    void Start()
    {
        // 获取场景中所有Agent
        VOAvoiding[] allAgents = FindObjectsOfType<VOAvoiding>();
        foreach (VOAvoiding agent in allAgents)
        {
            if (agent != this) 
                otherAgents.Add(agent);
        }
        
        // 检查是否有目标
        hasTarget = target != null;
    }

    // Update is called once per frame
    void Update()
    {
        if (hasTarget && Vector3.Distance(transform.position, target.position) < targetReachDistance)
        {
            // 到达目标，停止移动
            currentVelocity = Vector3.zero;
            return;
        }
        
        // 计算首选速度（朝向目标）
        CalculatePreferredVelocity();
        
        // VO算法避障
        currentVelocity = CalculateAvoidanceVelocity();
        
        // 应用速度和旋转
        transform.position += currentVelocity * Time.deltaTime;
        if (currentVelocity != Vector3.zero)
        {
            Quaternion targetRotation = Quaternion.LookRotation(currentVelocity);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotationSmoothness * Time.deltaTime);
        }
        
        DrawVelocityArrows();
    }
    
    private Vector3 CalculateAvoidanceVelocity()
    {
        // 1. 初始化最优速度为首选速度
        Vector3 optimalVelocity = preferredVelocity;
    
        // 2. 构建可行的速度空间（最初为整个速度圆）
        List<VelocityObstacle> obstacles = new List<VelocityObstacle>();
    
        // 3. 计算所有潜在碰撞威胁
        foreach (VOAvoiding other in otherAgents)
        {
            
            if (other == this || !other.gameObject.activeSelf) continue;
        
            //计算相对位置和速度
            Vector3 relativePos = other.transform.position - transform.position;
            Vector3 relativeVel = other.currentVelocity - optimalVelocity;
            
            float combinedRadius = agentRadius + other.agentRadius;
            float timeToCollision = CalculateTimeToCollision(
                relativePos, 
                relativeVel, 
                agentRadius
            );

            //只处理近期潜在碰撞
            if (timeToCollision > 0 && timeToCollision <= timeHorizon)
            {
                VelocityObstacle vo = CreateVelocityObstacle(
                    relativePos, 
                    relativeVel, 
                    timeToCollision,
                    combinedRadius
                );
            
                obstacles.Add(vo);
            }
        }
        
        if (obstacles.Count != 0)
        {
            foreach (var ob in obstacles)
            {
                DrawArrow(transform.position, ob.leftBoundary, Color.yellow);
                DrawArrow(transform.position, ob.rightBoundary, Color.yellow);
            }
        }
    
        
        //构建速度可行空间
        List<Vector3> candidateVelocities = GenerateCandidateVelocities();
    
        //筛选所有不处于速度障碍域中的可行速度
        List<Vector3> feasibleVelocities = FilterFeasibleVelocities(
            candidateVelocities, 
            obstacles
        );
    
        //选择最优避障速度
        if (feasibleVelocities.Count > 0)
        {
            optimalVelocity = SelectBestVelocity(feasibleVelocities);
        }
        
    
        return optimalVelocity;
    }
    
    private float CalculateTimeToCollision(Vector3 relativePos, Vector3 relativeVel, float combinedRadius)
    {
        float distance = relativePos.magnitude;
    
        // 已经发生碰撞的情况
        if (distance < combinedRadius) return 0f;
    
        float relativeSpeedSq = relativeVel.sqrMagnitude;
    
        // 处理近乎静止的情况
        if (relativeSpeedSq < 0.0001f) return float.MaxValue;

        float a = Vector3.Dot(relativeVel, relativeVel);
        float b = 2*Vector3.Dot(relativePos, relativeVel);
        float c = Vector3.Dot(relativePos, relativePos)-4*agentRadius*agentRadius;
        float discriminant = b * b - 4* a * c;
        if (discriminant < 0)
            return -1;
        else
        {
            float t1 = (-b - Mathf.Sqrt(discriminant)) / (2 * a);
            float t2 = (-b + Mathf.Sqrt(discriminant)) / (2 * a);
            if(t1 >= 0)
                return t1;
            else if(t2 >= 0)
                return t2;
            else
                return -1;
        }
    }
    
    // 创建速度障碍域
    private VelocityObstacle CreateVelocityObstacle(
        Vector3 relativePos, 
        Vector3 relativeVel, 
        float timeToCollision,
        float combinedRadius)
    {
        VelocityObstacle vo = new VelocityObstacle();
    
        // 1. 计算碰撞点位置
        vo.collisionPoint = transform.position + relativeVel * timeToCollision;
    
        // 2. 计算VO锥的角度
        float coneAngle = Mathf.Atan2(combinedRadius, timeToCollision * maxSpeed);
        vo.coneAngle = coneAngle;
    
        // 3. 计算VO的边界方向
        Vector3 coneDirection = relativePos.normalized;
        vo.leftBoundary = Quaternion.AngleAxis(coneAngle * Mathf.Rad2Deg, Vector3.up) * coneDirection;
        vo.rightBoundary = Quaternion.AngleAxis(-coneAngle * Mathf.Rad2Deg, Vector3.up) * coneDirection;
    
        return vo;
    }
    
    
    // 生成候选速度空间
    private List<Vector3> GenerateCandidateVelocities()
    {
        const int velocitySampleCount = 180; // 半圆采样点数量
        List<Vector3> velocities = new List<Vector3>();
    
        // 添加首选速度作为首要候选
        velocities.Add(preferredVelocity);
    
        // 绕首选速度方向采样
        float angleStep = 180f / velocitySampleCount;
    
        for (int i = 0; i < velocitySampleCount; i++)
        {
            // 在当前首选速度周围生成候选速度
            Quaternion rotation = Quaternion.Euler(0, -90 + i * angleStep, 0);
            Vector3 candidate = rotation * preferredVelocity.normalized * maxSpeed;
        
            velocities.Add(candidate);
        
            // 添加一些随机扰动
            Vector3 perturbed = candidate + new Vector3(
                Random.Range(-0.1f, 0.1f) * maxSpeed,
                0,
                Random.Range(-0.1f, 0.1f) * maxSpeed
            ).normalized * maxSpeed;
        
            velocities.Add(perturbed);
        }
    
        return velocities;
    }
    
    // 筛选可行速度
    private List<Vector3> FilterFeasibleVelocities(
        List<Vector3> candidates, 
        List<VelocityObstacle> obstacles)
    {
        List<Vector3> feasibleVelocities = new List<Vector3>();
    
        foreach (Vector3 velocity in candidates)
        {
            bool isFeasible = true;
        
            // 检查该速度是否在任何速度障碍锥内
            foreach (VelocityObstacle vo in obstacles)
            {
                if (IsVelocityInsideObstacle(velocity, vo))
                {
                    isFeasible = false;
                    break;
                }
            }
        
            if (isFeasible)
            {
                feasibleVelocities.Add(velocity);
            }
        }
    
        return feasibleVelocities;
    }
    
    // 检查速度是否在障碍域内
    private bool IsVelocityInsideObstacle(Vector3 velocity, VelocityObstacle vo)
    {
        // 1. 计算速度方向的单位向量
        Vector3 velDir = velocity.normalized;
    
        // 2. 计算速度方向与VO左边界、右边界的角度
        Vector3 sign_left = Vector3.Cross(velDir, vo.leftBoundary);
        Vector3 sign_right = Vector3.Cross(velDir, vo.rightBoundary);
        if (sign_left.y * sign_right.y < 0)
            return true;
        else
            return false;
    }
    
    // 选择最优避障速度
    private Vector3 SelectBestVelocity(List<Vector3> feasibleVelocities)
    {
        Vector3 bestVelocity = feasibleVelocities[0];
        float minDeviation = float.MaxValue;
    
        // 1. 优先选择最接近首选速度的可行速度
        foreach (Vector3 velocity in feasibleVelocities)
        {
            float deviation = Vector3.Angle(velocity, preferredVelocity);
            
            if (deviation < minDeviation)
            {
                minDeviation = deviation;
                bestVelocity = velocity;
            }
        }
    
        return bestVelocity;
    }
    
    private void CalculatePreferredVelocity()
    {
        if (hasTarget)
        {
            // 首选速度直接朝向目标
            preferredVelocity = (target.position - transform.position).normalized * maxSpeed;
            preferredVelocity.y = 0;
        }
        else
        {
            // 如果没有目标，保持当前方向
            preferredVelocity = Vector3.zero;
        }
    }
    
    private void DrawVelocityArrows()
    {
        if (!drawVelocityArrows) return;
        
        // 绘制首选速度
        Debug.DrawRay(transform.position, preferredVelocity * 0.5f, Color.red);
        DrawArrow(transform.position, preferredVelocity * 0.5f, Color.red);
        
        // 绘制最终速度
        Debug.DrawRay(transform.position, currentVelocity * 0.5f, Color.blue);
        DrawArrow(transform.position, currentVelocity * 0.5f, Color.blue);
    }
    
    private void DrawArrow(Vector3 position, Vector3 direction, Color color)
    {
        float arrowHeadAngle = 20f;
        float arrowHeadLength = 0.25f;
        
        if (direction.magnitude > 0.01f)
        {
            Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0) * new Vector3(0, 0, 1);
            Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0) * new Vector3(0, 0, 1);
            Debug.DrawRay(position + direction, right * arrowHeadLength, color);
            Debug.DrawRay(position + direction, left * arrowHeadLength, color);
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Agent"))
            Profiler.Instance.collisionCount++;
    }
}
