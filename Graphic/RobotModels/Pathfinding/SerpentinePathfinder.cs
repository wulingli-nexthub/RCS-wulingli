using System;
using System.Collections.Generic;

namespace GridDemo.RobotModels.Pathfinding
{
    /// <summary>
    /// 蛇形路径生成器：
    /// - 按整行蛇形扫描（左右来回），到行边界再纵向移动一格；
    /// - 垂直方向从 start.Y 向 goal.Y 推进；
    /// - 在包含目标的那一行，水平部分只走到 goal.X；
    /// - 所有格子必须满足 isWalkable。
    /// </summary>
    internal static class SerpentinePathfinder
    {
        public static List<GridPos> BuildPath(
            int width,
            int height,
            GridPos start,
            GridPos goal,
            Func<GridPos, bool> isWalkable)
        {
            var path = new List<GridPos>();

            if (!isWalkable(start) || !isWalkable(goal))
            {
                return path;
            }

            if (start.X == goal.X && start.Y == goal.Y)
            {
                path.Add(start);
                return path;
            }

            var cur = start;
            path.Add(cur);

            // 垂直方向：从 start.Y 朝 goal.Y 走（向下或向上）
            int verticalSign = goal.Y >= start.Y ? 1 : -1;

            // 当前行的蛇形方向：true=向右，false=向左
            bool goRight = true;

            int maxSteps = width * height * 4;
            int steps = 0;

            while (!(cur.X == goal.X && cur.Y == goal.Y) && steps < maxSteps)
            {
                steps++;

                int rowY = cur.Y;
                int rowEndX;

                // 若当前行是目标行，则只扫到 goal.X
                if (rowY == goal.Y)
                {
                    rowEndX = goal.X;
                }
                else
                {
                    // 非目标行：走到边界
                    rowEndX = goRight ? width - 1 : 0;
                }

                int dxSign = goRight ? 1 : -1;

                // 沿当前蛇形方向水平走到 rowEndX
                while (cur.X != rowEndX)
                {
                    var next = new GridPos(cur.X + dxSign, cur.Y);
                    if (!IsInBounds(next, width, height) || !isWalkable(next))
                    {
                        // 被障碍或边界打断，返回已经生成的部分
                        return path;
                    }

                    cur = next;
                    path.Add(cur);

                    if (cur.X == goal.X && cur.Y == goal.Y)
                    {
                        return path;
                    }
                }

                // 行已走完：如果已经是目标行，则结束
                if (cur.Y == goal.Y)
                {
                    // 此时 cur.X 已 == goal.X，上面的 while 负责保证
                    break;
                }

                // 纵向移动一格
                var verticalNext = new GridPos(cur.X, cur.Y + verticalSign);
                if (!IsInBounds(verticalNext, width, height) || !isWalkable(verticalNext))
                {
                    return path;
                }

                cur = verticalNext;
                path.Add(cur);
                if (cur.X == goal.X && cur.Y == goal.Y)
                {
                    break;
                }

                // 新行反转蛇形方向
                goRight = !goRight;
            }

            return path;
        }

        private static bool IsInBounds(GridPos p, int width, int height)
        {
            return p.X >= 0 && p.Y >= 0 && p.X < width && p.Y < height;
        }
    }
}