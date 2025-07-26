using System;
using Unity.Mathematics;

/// <summary>
/// 用于比较两组Float值
/// </summary>
public struct FloatPair
{
    private readonly float major;
    private readonly float minor;

    internal FloatPair(float major, float minor)
    {
        this.major = major;
        this.minor = minor;
    }
    
    /// <summary>
    /// 第一组float值是否小于第二组float值。
    /// 如果两组的major不相等时，major的比较结果为准；
    /// 如果当两组的major想等时，比较两组的minor。
    /// </summary>
    /// <param name="pair1">第一组float值</param>
    /// <param name="pair2">第二组float值</param>
    /// <returns>是否第一组小于第二组</returns>
    public static bool operator <(FloatPair pair1, FloatPair pair2)
    {
        return pair1.major < pair2.major || !(pair2.major < pair1.major) && pair1.minor < pair2.minor;
    }
    
    /// <summary>
    /// 第一组float值是否小于等于第二组float值。
    /// 如果两组的major和minor都相等时，返回想等；
    /// 否则，同oeprator &lt;
    /// </summary>
    /// <param name="pair1">第一组float值</param>
    /// <param name="pair2">第二组float值</param>
    /// <returns>是否第一组小于等于第二组</returns>
    public static bool operator <=(FloatPair pair1, FloatPair pair2)
    {
        return (Math.Abs(pair1.major - pair2.major) < math.EPSILON &&
                Math.Abs(pair1.minor - pair2.minor) < math.EPSILON)
               || pair1 < pair2;
    }
    
    /// <summary>
    /// 第一组float值是否小于等于第二组float值。
    /// oeprator &lt; 取反;
    /// </summary>
    /// <param name="pair1">第一组float值</param>
    /// <param name="pair2">第二组float值</param>
    /// <returns>是否第一组大于第二组</returns>
    public static bool operator >(FloatPair pair1, FloatPair pair2)
    {
        return !(pair1 <= pair2);
    }
    
    /// <summary>
    /// 第一组float值是否小于等于第二组float值。
    /// oeprator &lt;= 取反;
    /// </summary>
    /// <param name="pair1">第一组float值</param>
    /// <param name="pair2">第二组float值</param>
    /// <returns>是否第一组大于等于第二组</returns>
    public static bool operator >=(FloatPair pair1, FloatPair pair2)
    {
        return !(pair1 < pair2);
    }
}