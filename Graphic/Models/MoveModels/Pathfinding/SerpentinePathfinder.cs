using System;
using System.Collections.Generic;

namespace GridDemo.RobotModels.Pathfinding
{
    /// <summary>
    /// 蛇形路径生成器（整行到边界再下移一格再整行）：
    /// - 从 start 出发，当前行先水平一直走到一侧边界；
    /// - 然后纵向走一格，再沿相反水平方向一直走到另一侧边界；
    /// - 如此循环，直到到达 goal 所在行，在目标行上只走到 goal.X 为止；
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

            if (width <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(width));
            }
            if (height <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(height));
            }
            if (isWalkable == null)
            {
                throw new ArgumentNullException(nameof(isWalkable));
            }

            if (!IsInBounds(start, width, height) || !IsInBounds(goal, width, height))
            {
                return path;
            }


            if (!isWalkable(start) || !isWalkable(goal))
            {
                return path;
            }

            if (start.Equals(goal))
            {
                path.Add(start);
                return path;
            }

            var cur = start;
            path.Add(cur);

            // 垂直方向：从 start.Y 朝 goal.Y 走（向下或向上）
            int verticalSign = goal.Y >= start.Y ? 1 : -1;

            // 蛇形水平方向：
            // 规则：为了直观，从 start 到 goal 某一侧开始：
            // - 若 start 更靠左或与 goal 同列，则第一行先向右扫到 width-1；
            // - 若 start 更靠右，则第一行先向左扫到 0。
            bool goRightFirst = start.X <= goal.X;
            bool goRight = goRightFirst;

            // 保险：避免逻辑错误导致死循环
            int maxSteps = width * height * 4;
            int steps = 0;

            while (!cur.Equals(goal) && steps < maxSteps)
            {
                steps++;

                int rowY = cur.Y;
                int rowEndX;

                // 若当前行是目标行，则水平只走到 goal.X；否则必须走到边界
                if (rowY == goal.Y)
                {
                    rowEndX = goal.X;
                }
                else
                {
                    rowEndX = goRight ? width - 1 : 0;
                }

                int dxSign = goRight ? 1 : -1;

                // 1) 沿当前方向水平走到 rowEndX
                while (cur.X != rowEndX)
                {
                    var next = new GridPos(cur.X + dxSign, cur.Y);
                    if (!IsInBounds(next, width, height) || !isWalkable(next))
                    {
                        // 被障碍或边界打断：返回已生成部分
                        return path;
                    }

                    cur = next;
                    path.Add(cur);

                    if (cur.Equals(goal))
                    {
                        return path;
                    }
                }

                // 当前行已经完成：
                // 如果已经是目标行（刚刚走到 goal.X），则结束
                if (cur.Y == goal.Y)
                {
                    break;
                }

                // 2) 纵向移动一格（向 goal.Y 方向）
                var verticalNext = new GridPos(cur.X, cur.Y + verticalSign);
                if (!IsInBounds(verticalNext, width, height) || !isWalkable(verticalNext))
                {
                    return path;
                }

                cur = verticalNext;
                path.Add(cur);

                if (cur.Equals(goal))
                {
                    break;
                }

                // 3) 新行反转蛇形方向
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