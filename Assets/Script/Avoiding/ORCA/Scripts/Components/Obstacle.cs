using Unity.Entities;
using Unity.Mathematics;

/// <summary>
/// 静态障碍的一个顶点
/// </summary>
public struct Obstacle
{
    /// <summary>
    /// 上一个顶点的索引
    /// </summary>
    public int nextIndex;
    /// <summary>
    /// 下一个顶点的索引
    /// </summary>
    public int prevIndex;
    /// <summary>
    /// 当前顶点指向下一个顶点的朝向
    /// </summary>
    public float2 direction;
    /// <summary>
    /// 顶点的位置
    /// </summary>
    public float2 point;
    /// <summary>
    /// 顶点标识ID
    /// </summary>
    public int id;
    /// <summary>
    /// 是否是凸多边形
    /// </summary>
    public bool convex;
}

/// <summary>
/// 通过IBaker烘焙出来的数据
/// </summary>
public struct ObstacleVertex : IBufferElementData
{
    public float2 position;
}