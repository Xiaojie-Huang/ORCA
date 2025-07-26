using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

public struct KDTree
{
    /// <summary>
    /// KDTree的节点
    /// </summary>
    private struct AgentTreeNode
    {
        /// <summary>
        /// 代理的开始索引
        /// </summary>
        internal int begin;
        /// <summary>
        /// 代理的结束索引
        /// </summary>
        internal int end;
        /// <summary>
        /// 左叶子节点的AgentTreeNode索引
        /// </summary>
        internal int left;
        /// <summary>
        /// 右叶子节点的AgentTreeNode索引
        /// </summary>
        internal int right;

        #region KDTree节点范围

        internal float maxX;
        internal float maxY;
        internal float minX;
        internal float minY;

        #endregion
    }
    
    public struct ObstacleTreeNode
    {
        public int obstacleIndex;
        public int left;
        public int right;
    }
    
    /// <summary>
    /// 查找临近Agent时候的临近Agent信息
    /// </summary>
    public struct AgentNeighbor
    {
        /// <summary>
        /// 临近Agent组件
        /// </summary>
        public Agent agent;
        /// <summary>
        /// 临近Agent距离当前Agent的距离
        /// </summary>
        public float distanceSq;
    }

    public struct ObstacleNeighbor
    {
        public Obstacle obstacle;
        public float distanceSq;
    }
    
    /// <summary>
    /// KDTree叶子节点的最大尺寸
    /// </summary>
    private const int MAX_LEAF_SIZE = 10;
    
    /// <summary>
    /// 本帧的所有代理
    /// </summary>
    [ReadOnly] public NativeArray<Agent> agents;
    /// <summary>
    /// KDTree的所有节点
    /// </summary>
    [ReadOnly] private NativeArray<AgentTreeNode> agentTree;
    
    /// <summary>
    /// 所有静态障碍物
    /// </summary>
    [ReadOnly] public NativeList<Obstacle> obstacles;
    /// <summary>
    /// 静态障碍物的KDTree
    /// </summary>
    [ReadOnly] public NativeList<ObstacleTreeNode> obstacleTree;

    public void BuildAgentTree(ref NativeArray<Agent> agentsThisFrame)
    {
        agents = agentsThisFrame;
        int treeLength = 2 * agents.Length;
        agentTree = new NativeArray<AgentTreeNode>(treeLength, Allocator.TempJob);

        if (agents.Length != 0)
        {
            BuildAgentTreeRecursive(0, agents.Length, 0);
        }
    }

    public JobHandle Dispose(JobHandle jobHandle)
    {
        return agentTree.Dispose(jobHandle);
    }

    private void BuildAgentTreeRecursive(int begin, int end, int node)
    {
        AgentTreeNode treeNode = agentTree[node];
        treeNode.begin = begin;
        treeNode.end = end;
        treeNode.minX = treeNode.maxX = agents[begin].position.x;
        treeNode.minY = treeNode.maxY = agents[begin].position.y;
        for (int i = begin + 1; i < end; ++i)
        {
            treeNode.maxX = math.max(treeNode.maxX, agents[i].position.x);
            treeNode.minX = math.min(treeNode.minX, agents[i].position.x);
            treeNode.maxY = math.max(treeNode.maxY, agents[i].position.y);
            treeNode.minY = math.min(treeNode.minY, agents[i].position.y);
        }

        if (end - begin > MAX_LEAF_SIZE)
        {
            // 节点的宽度大于长度，就竖着划分；否则横着划分
            bool isVertical = treeNode.maxX - treeNode.minX > treeNode.maxY - treeNode.minY;
            // 取节点的中间值
            float splitValue = 0.5f * (isVertical ? treeNode.maxX + treeNode.minX : treeNode.maxY + treeNode.minY);

            int left = begin;
            int right = end;
            
            // 调整agents数组，找出所有比中间值小的Agent放到数组左边，找出所有比中间值大的Agent放到数组右边
            while (left < right)
            {
                // 数组左边应该要都小于中间值
                // 从数组左边开始找，找到大于等于中间值的Agent对应的索引
                while (left < right && (isVertical ? agents[left].position.x : agents[left].position.y) < splitValue)
                {
                    ++left;
                }
                
                // 数组右边应该要都大于中间值
                // 从数组右边开始找，找到小于中间值的Agent对应的索引
                while (right > left && (isVertical ? agents[right - 1].position.x : agents[right - 1].position.y) >= splitValue)
                {
                    --right;
                }
                
                // 将数组左边大于等于中间值的Agent，和数组右边小于中间值的Agent，进行交换
                // 保证左边的Agent一定小于中间值，右边的Agent一定大于等于中间值
                if (left < right)
                {
                    Agent tempAgent = agents[left];
                    agents[left] = agents[right - 1];
                    agents[right - 1] = tempAgent;
                    ++left;
                    --right;
                }
            }
            
            // KDTree当前节点左孩子管理的Agent数量
            int leftSize = left - begin;

            if (leftSize == 0)
            {
                ++leftSize;
                ++left;
            }

            treeNode.left = node + 1;
            treeNode.right = node + 2 * leftSize;
            agentTree[node] = treeNode;
            
            BuildAgentTreeRecursive(begin, left, treeNode.left);
            BuildAgentTreeRecursive(left, end, treeNode.right);
        }
        else
        {
            agentTree[node] = treeNode;
        }
    }
    
    public static int BuildObstacleTreeRecursive(ref NativeList<Obstacle> obstacles, ref NativeList<ObstacleTreeNode> obstacleTree, in NativeArray<int> obstaclesIndices)
    {
        if (obstaclesIndices.Length == 0)
        {
            return -1;
        }

        ObstacleTreeNode node = new ObstacleTreeNode();

        int optimalSplit = 0;
        // 最优索引optimalSplit的障碍物，左边的其他障碍物数量
        int minLeft = obstaclesIndices.Length;
        // 最优索引optimalSplit的障碍物，右边的其他障碍物数量
        int minRight = obstaclesIndices.Length;

        for (int i = 0; i < obstaclesIndices.Length; ++i)
        {
            int leftSize = 0;
            int rightSize = 0;

            int indexI = obstaclesIndices[i];
            
            // 障碍物I1
            Obstacle obstacleI1 = obstacles[indexI];
            // 障碍物I2
            Obstacle obstacleI2 = obstacles[obstacleI1.nextIndex];

            for (int j = 0; j < obstaclesIndices.Length; ++j)
            {
                if (i == j)
                {
                    continue;
                }

                int indexJ = obstaclesIndices[j];
                
                // 障碍物J1
                Obstacle obstacleJ1 = obstacles[indexJ];
                // 障碍物J2
                Obstacle obstacleJ2 = obstacles[obstacleJ1.nextIndex];
                
                // J1是否在线段I1-I2左边
                float j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                // J2是否在线段I1-I2左边
                float j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

                if (j1LeftOfI >= -math.EPSILON && j2LeftOfI >= -math.EPSILON)
                {
                    // J1和J2都在线段I1-I2左边
                    ++leftSize;
                }
                else if (j1LeftOfI <= math.EPSILON && j2LeftOfI <= math.EPSILON)
                {
                    // J1和J2都在线段I1-I2右边
                    ++rightSize;
                }
                else
                {
                    // J1和J2在线段I1-I2的两边
                    // （leftSize和rightSize都加，是因为这种情况要新增Obstacle）
                    ++leftSize;
                    ++rightSize;
                }
                // 如果线段I1-I2左边或者右边的障碍物数量，已经大于最小值，说明这条线段I1-I2不是最优，直接break，没必要继续遍历
                if (new FloatPair(math.max(leftSize, rightSize), math.min(leftSize, rightSize))
                    >=
                    new FloatPair(math.max(minLeft, minRight), math.min(minLeft, minRight)))
                {
                    break;
                }
            }
            
            // 如果线段I1-I2左边或者右边的障碍物数量，小于最小值，说明这条线段I1-I2更优，更能将所有障碍平均分到左右两边
            if (new FloatPair(math.max(leftSize, rightSize), math.min(leftSize, rightSize))
                <
                new FloatPair(math.max(minLeft, minRight), math.min(minLeft, minRight)))
            {
                minLeft = leftSize;
                minRight = rightSize;
                // 记录i为最优划分障碍物，用来当作KDTree划分左右子节点的参考
                optimalSplit = i;
            }
        }

        {
            // 左节点所有障碍物的数组
            NativeArray<int> leftObstacles = new NativeArray<int>(minLeft, Allocator.Temp);
            for (int n = 0; n < minLeft; ++n)
            {
                leftObstacles[n] = -1;
            }
            // 右节点所有障碍物的数组
            NativeArray<int> rightObstacles = new NativeArray<int>(minRight, Allocator.Temp);
            for (int n = 0; n < minRight; ++n)
            {
                rightObstacles[n] = -1;
            }
            
            // 左节点计数，当前往数组里添加到第几个障碍物
            int leftCounter = 0;
            // 右节点计数，当前往数组里添加到第几个障碍物
            int rightCounter = 0;
            // 当前用来区分左右节点的障碍物
            int i = optimalSplit;
            
            // 当前用来区分左右节点的障碍物在obstalces列表里的索引
            int indexI = obstaclesIndices[i];
            
            // 当前用来用来区分左右节点的线段的端点I1
            Obstacle obstacleI1 = obstacles[indexI];
            // 当前用来用来区分左右节点的线段的端点I2
            Obstacle obstacleI2 = obstacles[obstacleI1.nextIndex];

            for (int j = 0; j < obstaclesIndices.Length; ++j)
            {
                if (i == j)
                {
                    continue;
                }

                int indexJ1 = obstaclesIndices[j];
                
                // 遍历到的线段的端点J1
                Obstacle obstacleJ1 = obstacles[indexJ1];
                // 遍历到的线段的端点J2
                Obstacle obstacleJ2 = obstacles[obstacleJ1.nextIndex];
                
                // J1是否在线段I1-I2左边
                float j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                // J2是否在线段I1-I2左边
                float j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);
                
                if (j1LeftOfI >= -math.EPSILON && j2LeftOfI >= -math.EPSILON)
                {
                    // J1和J2都在线段I1-I2左边
                    leftObstacles[leftCounter++] = indexJ1;
                }
                else if (j1LeftOfI <= math.EPSILON && j2LeftOfI <= math.EPSILON)
                {
                    // J1和J2都在线段I1-I2右边
                    rightObstacles[rightCounter++] = indexJ1;
                }
                else
                {
                    // J1和J2在线段I1-I2的两边，则新建一个名为New的障碍物，重新组成J1-New线段和New-J2线段
                    
                    // 平移J1-J2为新的线段J1'-J2'，使J2'和I1重合，且J1-J2平行于J1'-J2'；
                    // t等于【J1到I1-I2的距离】除以【J1'到I1-I2的距离】
                    // 根据三角形相似原理，t就是新的障碍物在J1-J2上的插值比值
                    float t = RVOMath.det(obstacleI2.point - obstacleI1.point, obstacleJ1.point - obstacleI1.point) /
                              RVOMath.det(obstacleI2.point - obstacleI1.point, obstacleJ1.point - obstacleJ2.point);
                    // 在J1-J2上的插值出新的障碍物的位置
                    float2 splitPoint = obstacleJ1.point + t * (obstacleJ2.point - obstacleJ1.point);
                    
                    int indexJ2 = obstacleJ1.nextIndex;
                    int indexNew = obstacles.Length;
                    // 初始化名为New的新障碍物
                    Obstacle newObstacle = new Obstacle()
                    {
                        point = splitPoint,
                        prevIndex = indexJ1,
                        nextIndex = indexJ2,
                        convex = true,
                        direction = obstacleJ1.direction,
                        id = indexNew
                    };
                    obstacles.Add(newObstacle);
                    
                    // 重新赋值J1和J2的next和prev
                    obstacleJ1.nextIndex = indexNew;
                    obstacles[indexJ1] = obstacleJ1;

                    obstacleJ2.prevIndex = indexNew;
                    obstacles[indexJ2] = obstacleJ2;
                    
                    // 如果J1在I1-I2的左边，也就是说J2在I1-I2的右边
                    if (j1LeftOfI > 0.0f)
                    {
                        leftObstacles[leftCounter++] = indexJ1;
                        rightObstacles[rightCounter++] = indexNew;
                    }
                    // 如果J1在I1-I2的右边，也就是说J2在I1-I2的左边
                    else
                    {
                        rightObstacles[rightCounter++] = indexJ1;
                        leftObstacles[leftCounter++] = indexNew;
                    }
                }
            }
            
            // 返回当前KDTree节点的索引
            int nodeIndex = obstacleTree.Length;
            obstacleTree.Add(node);
            
            // 当前KDTree的节点的障碍物索引，就是区分左右节点的障碍物的索引
            node.obstacleIndex = indexI;
            // 递归生成左右子树
            node.left = BuildObstacleTreeRecursive(ref obstacles, ref obstacleTree, in leftObstacles);
            node.right = BuildObstacleTreeRecursive(ref obstacles, ref obstacleTree, in rightObstacles);

            obstacleTree[nodeIndex] = node;

            leftObstacles.Dispose();
            rightObstacles.Dispose();
           
            return nodeIndex;
        }
    }

    public void QueryAgentNeighborsRecursive(Agent agent, ref float rangeSq, int node, ref NativeList<AgentNeighbor> neighbors)
    {
        AgentTreeNode treeNode = agentTree[node];
        // 如果遍历到叶子节点了，那就把所有Agent都加到neighbors里吧
        if (treeNode.end - treeNode.begin <= MAX_LEAF_SIZE)
        {
            for (int i = treeNode.begin; i < treeNode.end; ++i)
            {
                InsertAgentNeighor(agent, agents[i], ref rangeSq, ref neighbors);
            }
        }
        else
        {
            // 计算agent的位置到左、右子节点包围框的距离，如果在包围框内则为0
            float distSqLeft = RVOMath.sqr(math.max(0.0f, agentTree[treeNode.left].minX - agent.position.x)) +
                               RVOMath.sqr(math.max(0.0f, agent.position.x - agentTree[treeNode.left].maxX)) +
                               RVOMath.sqr(math.max(0.0f, agentTree[treeNode.left].minY - agent.position.y)) +
                               RVOMath.sqr(math.max(0.0f, agent.position.y - agentTree[treeNode.left].maxY));
            float distSqRight = RVOMath.sqr(math.max(0.0f, agentTree[treeNode.right].minX - agent.position.x)) +
                                RVOMath.sqr(math.max(0.0f, agent.position.x - agentTree[treeNode.right].maxX)) +
                                RVOMath.sqr(math.max(0.0f, agentTree[treeNode.right].minY - agent.position.y)) +
                                RVOMath.sqr(math.max(0.0f, agent.position.y - agentTree[treeNode.right].maxY));
            
            // 如果agent离左边包围框更近
            if (distSqLeft < distSqRight)
            {
                // 如果左边离agent的距离小于rangeSq
                if (distSqLeft < rangeSq)
                {
                    QueryAgentNeighborsRecursive(agent, ref rangeSq, treeNode.left, ref neighbors);
                    // 如果右边离agent的距离也小于rangeSq
                    if (distSqRight < rangeSq)
                    {
                        QueryAgentNeighborsRecursive(agent, ref rangeSq, treeNode.right, ref neighbors);
                    }
                }
            }
            // 如果agent离右边包围框更近
            else
            {
                // 如果右边离agent的距离小于rangeSq
                if (distSqRight < rangeSq)
                {
                    QueryAgentNeighborsRecursive(agent, ref rangeSq, treeNode.right, ref neighbors);
                    
                    // 如果左边离agent的距离也小于rangeSq
                    if (distSqLeft < rangeSq)
                    {
                        QueryAgentNeighborsRecursive(agent, ref rangeSq, treeNode.left, ref neighbors);
                    }
                }
            }
        }
    }

    public void QueryObstacleNeighborsRecursive(Agent agent, float rangeSq, int node, ref NativeList<ObstacleNeighbor> obstacleNeighbors)
    {
        if (node >= 0 && node < obstacleTree.Length)
        {
            ObstacleTreeNode treeNode = obstacleTree[node];

            Obstacle obstacle1 = obstacles[treeNode.obstacleIndex];
            Obstacle obstacle2 = obstacles[obstacle1.nextIndex];
            
            float agentLeftOfLine = RVOMath.leftOf(obstacle1.point, obstacle2.point, agent.position);

            QueryObstacleNeighborsRecursive(agent, rangeSq, agentLeftOfLine >= 0.0f ? treeNode.left : treeNode.right, ref obstacleNeighbors);
            
            float distSqLine = RVOMath.sqr(agentLeftOfLine) / math.lengthsq(obstacle2.point - obstacle1.point);

            if (distSqLine < rangeSq)
            {
                if (agentLeftOfLine < 0.0f)
                {
                    InsertObstacleNeighbor(agent, treeNode.obstacleIndex, rangeSq, ref obstacleNeighbors);
                }
                
                QueryObstacleNeighborsRecursive(agent, rangeSq, agentLeftOfLine >= 0.0f ? treeNode.right : treeNode.left, ref obstacleNeighbors);
            }
        }
    }

    public void InsertAgentNeighor(Agent self, Agent other, ref float rangeSq, ref NativeList<AgentNeighbor> neighbors)
    {
        if (self.id != other.id)
        {
            // 两个Agent之间的距离
            float distSq = math.lengthsq(self.position - other.position);
            
            // 如果两个Agent之间的距离小于范围阈值
            if (distSq < rangeSq)
            {
                // 如果要查找的Neighbor数量还没有达到最大值，则直接加入neighbors列表
                if (neighbors.Length < self.maxNeighbors)
                {
                    neighbors.Add(new AgentNeighbor()
                    {
                        agent = other,
                        distanceSq = distSq
                    });
                }
                
                // 新的Neighber要插入的位置（倒着遍历）
                int i = neighbors.Length - 1;
                
                // 保证插入后的neighbors列表按distanceSq从小到大排序
                while (i != 0 && distSq < neighbors[i - 1].distanceSq)
                {
                    neighbors[i] = neighbors[i - 1];
                    --i;
                }
                neighbors[i] = new AgentNeighbor()
                {
                    agent = other,
                    distanceSq = distSq
                };
                
                // 如果neighbors列表已经满了，那么更新范围阈值rangeSq
                // 也就是说rangeSq其实并不需要那么大
                if (neighbors.Length == self.maxNeighbors)
                {
                    rangeSq = neighbors[neighbors.Length - 1].distanceSq;
                }
            }
        }
    }

    public void InsertObstacleNeighbor(Agent self, int obstacleIndex, float rangeSq, ref NativeList<ObstacleNeighbor> obstacleNeighbors)
    {
        Obstacle obstacle = obstacles[obstacleIndex];
        Obstacle nextObstacle = obstacles[obstacle.nextIndex];
        
        float distSq = RVOMath.distSqPointLineSegment(obstacle.point, nextObstacle.point, self.position);
        
        if (distSq < rangeSq)
        {
            obstacleNeighbors.Add(new ObstacleNeighbor()
            {
                obstacle = obstacle,
                distanceSq = distSq
            });

            int i = obstacleNeighbors.Length - 1;

            while (i != 0 && distSq < obstacleNeighbors[i - 1].distanceSq)
            {
                obstacleNeighbors[i] = obstacleNeighbors[i - 1];
                --i;
            }
            obstacleNeighbors[i] = new ObstacleNeighbor()
            {
                obstacle = obstacle,
                distanceSq = distSq
            };
        }
    }
}