using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class BasicAvoiding : MonoBehaviour
{
    [Header("移动参数")]
    public float moveSpeed = 10f;          // 移动速度
    public float directionChangeTime = 2f; // 方向变化间隔
    public float maxDistance = 10f;       // 移动边界
    
    [Header("射线检测")]
    public float rayDistance = 5f;          // 射线检测距离
    public LayerMask obstacleLayer;          // 障碍物层
    
    private Vector3 randomDirection;
    
    public bool showDebugRays = true;
    private bool showDirectionArrow = true;
    private Color directionArrowColor = Color.cyan;
    
    private Vector3 moveDirection;
    private float timer;
    

    void Start()
    {
        timer = directionChangeTime;
        GenerateRandomDirection();
    }

    // Update is called once per frame
    void Update()
    {
        timer -= Time.deltaTime;
        if (timer <= 0)
        {
            GenerateRandomDirection();
            timer = directionChangeTime;
        }

        BoundaryCheck();
        if(Profiler.Instance.avoiding)
            PerformRaycastDetection();
        
        moveDirection = Vector3.Lerp(moveDirection, randomDirection, Time.deltaTime * 4f);
        // 移动
        transform.position += randomDirection * moveSpeed * Time.deltaTime;
        // 旋转方向
        if (randomDirection != Vector3.zero)
        {
            Quaternion targetRotation = Quaternion.LookRotation(randomDirection);
            transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, 0.1f);
        }
    }
    

    private void GenerateRandomDirection()
    {
        // 生成3D空间中的随机方向
        randomDirection = new Vector3(
            Random.Range(-1f, 1f),
            0,
            Random.Range(-1f, 1f)
        ).normalized;
    }
    
    private void BoundaryCheck()
    {
        Vector3 currentPos = transform.position;
        
        if (Vector3.Distance(currentPos, Vector3.zero) > maxDistance)
        {
            Vector3 directionToOrigin = (Vector3.zero - currentPos).normalized;
            randomDirection = Vector3.Lerp(randomDirection, directionToOrigin, 0.5f).normalized;
        }
    }
    
    private void DrawDirectionArrow()
    {
        if (!showDirectionArrow || moveDirection == Vector3.zero)
            return;
        
        Vector3 start = transform.position + Vector3.up * 0.1f;
        Vector3 end = start + moveDirection * 2f; // 箭头长度
        float headAngle = 20f; // 箭头尖端角度
        float headLength = 0.4f; // 箭头尖端长度
        
        // 绘制主线
        Debug.DrawLine(start, end, directionArrowColor);
        
        // 计算并绘制两个箭头尖端
        Vector3 backDir = (moveDirection).normalized;
        Vector3 leftArrowTip = Quaternion.Euler(0, -headAngle, 0) * -backDir;
        Vector3 rightArrowTip = Quaternion.Euler(0, headAngle, 0) * -backDir;
        
        Debug.DrawLine(end, end + leftArrowTip * headLength, directionArrowColor);
        Debug.DrawLine(end, end + rightArrowTip * headLength, directionArrowColor);
    }

    private void PerformRaycastDetection()
    {
        Vector3 rayOrigin = transform.position + Vector3.up * 0.5f;
        Ray ray = new Ray(rayOrigin, transform.forward);
        
        if (Physics.Raycast(ray, out RaycastHit hit, rayDistance, obstacleLayer))
        {
            // 根据法线方向计算避障方向
            Vector3 avoidanceDirection = CalculateAvoidanceDirection(hit.normal);
            
            randomDirection = avoidanceDirection;
        }
    }
    
    private Vector3 CalculateAvoidanceDirection(Vector3 hitNormal)
    {
        // 计算法线的垂直方向（切线方向）
        Vector3 tangent = Vector3.Cross(hitNormal, Vector3.up).normalized;
        
        // 随机选择左右方向（避免所有代理行为一致）
        float directionChoice = Random.Range(0f, 1f) > 0.5f ? 1f : -1f;
        
        // 计算避障方向（法线切线方向 + 轻微远离法线方向）
        Vector3 avoidanceDirection = (tangent * directionChoice + hitNormal * 0.3f).normalized;
        
        return avoidanceDirection;
    }
    

    void OnCollisionEnter(Collision collision)
    {
        if(collision.gameObject.CompareTag("Player"))
            Profiler.Instance.collisionCount += 1;
    }
    
    void OnDrawGizmos()
    {
        if (showDebugRays)
        {
            // 在场景视图中始终绘制方向箭头
            if (Application.isPlaying && showDirectionArrow)
            {
                DrawDirectionArrow();
            }
        
            // 绘制边界
            Gizmos.color = Color.yellow;
            if(!Application.isPlaying)
                Gizmos.DrawWireSphere(Vector3.zero, maxDistance);   
        }
    }
}
