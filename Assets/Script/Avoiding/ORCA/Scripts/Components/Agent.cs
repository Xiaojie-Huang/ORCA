using Unity.Entities;
using Unity.Mathematics;

public struct Agent : IComponentData
{
    /// <summary>
    /// 代理ID，全局唯一
    /// </summary>
    public int id;
    
    /// <summary>
    /// 当前位置
    /// </summary>
    public float2 position;
    /// <summary>
    /// 期望速度，也就是v^opt
    /// </summary>
    public float2 prefVelocity;
    /// <summary>
    /// 当前速度
    /// </summary>
    public float2 velocity;
    /// <summary>
    /// 圆的半径
    /// </summary>
    public float radius;
    /// <summary>
    /// 避障责任权重
    /// </summary>
    public float weight;
    /// <summary>
    /// 最大速率
    /// </summary>
    public float maxSpeed;
    
    /// <summary>
    /// KDTree查找的最大邻接Agent数量
    /// </summary>
    public int maxNeighbors;
    /// <summary>
    /// KDTree查找邻接Agent时候的最大检测距离
    /// </summary>
    public float neighborDist;
    
    /// <summary>
    /// 提前避障的时间（可以大于帧间隔deltaTime，表示要提前避障）
    /// </summary>
    public float timeHorizon;
    /// <summary>
    /// 静态障碍物避障时间
    /// </summary>
    public float timeHorizonObst;
    
    /// <summary>
    /// 移动的目标位置，用来计算期望速度prefVelocity
    /// </summary>
    public float2 targetPos;
}