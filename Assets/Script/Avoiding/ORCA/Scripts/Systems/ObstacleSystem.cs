using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using ObstacleTreeNode = KDTree.ObstacleTreeNode;

public struct ObstacleCollection : IComponentData
{
    public NativeList<Obstacle> obstacles;
    public NativeList<ObstacleTreeNode> obstacleTree;
    public bool updateObstacleKDTree;
}

public partial struct ObstacleSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.EntityManager.AddComponentData(state.SystemHandle, new ObstacleCollection()
        {
            obstacles = new NativeList<Obstacle>(Allocator.Persistent),
            obstacleTree = new NativeList<ObstacleTreeNode>(Allocator.Persistent),
            updateObstacleKDTree = true
        });
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        ObstacleCollection obstacleCollection = state.EntityManager.GetComponentData<ObstacleCollection>(state.SystemHandle);
        // 只有需要更新，才会执行此OnUpdate函数
        if (!obstacleCollection.updateObstacleKDTree)
        {
            return;
        }
        obstacleCollection.obstacles.Clear();
        obstacleCollection.obstacleTree.Clear();
        
        // 使用引用，简化代码
        ref NativeList<Obstacle> obstaclesList = ref obstacleCollection.obstacles;

        foreach (DynamicBuffer<ObstacleVertex> obstaclePolygon in SystemAPI.Query<DynamicBuffer<ObstacleVertex>>())
        {
            if (obstaclePolygon.Length < 2)
            {
                continue;
            }
            obstacleCollection.updateObstacleKDTree = false;
            
            // 当前polygon在obstacleCollection.obstacles列表里的开始索引值
            int obstacleStateIndex = obstaclesList.Length;
            
            // 遍历多边形的每一个顶点，并生成Obstacle，加入obstaclesList列表
            for (int i = 0; i < obstaclePolygon.Length; ++i)
            {
                float2 currentPosition = obstaclePolygon[i].position;
                
                Obstacle obstacle = new Obstacle()
                {
                    point = currentPosition
                };

                if (i != 0)
                {
                    // obstaclesList.Length就是上一个Obstacle的索引
                    obstacle.prevIndex = obstaclesList.Length - 1;
                    // 更新上一个Obstacle的nextIndex
                    Obstacle prev = obstaclesList[obstacle.prevIndex];
                    prev.nextIndex = obstaclesList.Length;
                    obstaclesList[obstacle.prevIndex] = prev;
                }

                if (i == obstaclePolygon.Length - 1)
                {
                    // obstacleStateIndex就是obstaclePolygon[0]在obstaclesList列表里的索引
                    // 这个索引就是下一个Obstacle的索引
                    obstacle.nextIndex = obstacleStateIndex;
                    // 更新下一个Obstacle的prevIndex
                    Obstacle next = obstaclesList[obstacle.nextIndex];
                    next.prevIndex = obstaclesList.Length;
                    obstaclesList[obstacle.nextIndex] = next;
                }
                
                // 下一个顶点的position
                // 如果是polygon的最后一个顶点，那么下一个顶点就是第一个顶点；
                // 如果不是polygon的最后一个顶点，那么下一个顶点就是索引为i+1的顶点。
                float2 nextPosition = obstaclePolygon[i == obstaclePolygon.Length - 1 ? 0 : i + 1].position;
                // 使用下一个顶点的position更新obstacle的方向
                obstacle.direction = math.normalizesafe(nextPosition - currentPosition);

                if (obstaclePolygon.Length == 2)
                {
                    // 如果并不是多边形，而是一条线段：
                    // 线段也算凸多边形
                    obstacle.convex = true;
                }
                else
                {
                    // 上一个顶点的Position
                    // 如果是polygon的第一个顶点，那么上一个顶点就是最后一个顶点；
                    // 如果不是polygon的第一个顶点，那么上一个顶点就是索引为i-1的顶点。
                    float2 prevPosition = obstaclePolygon[i == 0 ? obstaclePolygon.Length - 1 : i - 1].position;
                    // 逆时针看，下一个顶点总是在【上一个顶点和当前顶点组成的线段】的左边，则说明是凸多边形
                    obstacle.convex = RVOMath.leftOf(prevPosition, currentPosition, nextPosition) >= 0.0f;
                }

                obstacle.id = obstaclesList.Length;
                obstaclesList.Add(obstacle);
            }
        }
        
        // 构建静态障碍物的KDTree
        NativeArray<int> obstaclesIndices = new NativeArray<int>(obstaclesList.Length, Allocator.Temp);
        for (int i = 0; i < obstaclesList.Length; ++i)
        {
            obstaclesIndices[i] = i;
        }
        KDTree.BuildObstacleTreeRecursive(ref obstaclesList, ref obstacleCollection.obstacleTree, in obstaclesIndices);
        obstaclesIndices.Dispose();
        
        state.EntityManager.SetComponentData(state.SystemHandle, obstacleCollection);
    }

    [BurstCompile]
    public void OnDestroy(ref SystemState state)
    {
        ObstacleCollection obstacleCollection = state.EntityManager.GetComponentData<ObstacleCollection>(state.SystemHandle);
        if (obstacleCollection.obstacles.IsCreated)
        {
            obstacleCollection.obstacles.Dispose();
            obstacleCollection.obstacleTree.Dispose();
        }
    }
}