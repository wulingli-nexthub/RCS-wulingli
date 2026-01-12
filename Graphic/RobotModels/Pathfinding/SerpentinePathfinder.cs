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
        public static List<GridPos> BuildPath( // 构建从 start 到 goal 的蛇形路径（若中途不可走，则返回已生成部分）
            int width,
            int height,
            GridPos start,
            GridPos goal,
            Func<GridPos, bool> isWalkable)
        {
            var path = new List<GridPos>();           // 保存路径点序列（按移动顺序）

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
            { // 起点或终点不在网格内，无法生成有效路径，返回空路径
                return path;
            }


            if (!isWalkable(start) || !isWalkable(goal))
            { // 起点或终点不可走，无法生成有效路径，返回空路径
                return path;
            }

            if (start.Equals(goal))
            { // 起点即终点，路径仅包含起点
                path.Add(start);
                return path;
            }

            var cur = start;    // 当前所在位置
            path.Add(cur);        // 将起点加入路径

            // 垂直方向：从 start.Y 朝 goal.Y 走（向下或向上）
            int verticalSign = goal.Y >= start.Y ? 1 : -1;

            // 蛇形水平方向：
            // 规则：为了直观，从 start 到 goal 某一侧开始：
            // - 若 start 更靠左或与 goal 同列，则第一行先向右扫到 width-1；
            // - 若 start 更靠右，则第一行先向左扫到 0。
            bool goRightFirst = start.X <= goal.X;
            bool goRight = goRightFirst;

            // 保险：避免逻辑错误导致死循环
            int maxSteps = width * height * 4; // 最大迭代步数（上限保护，避免异常情况下无限循环）
            int steps = 0; // 已迭代步数计数器

            while (!cur.Equals(goal) && steps < maxSteps)
            { // 尚未到达目标且未超出最大步数
                steps++;  // 增加步数计数

                int rowY = cur.Y;       // 当前行的行号
                int rowEndX;          // 当前行水平扫描的目标X

                if (rowY == goal.Y)
                { // 若当前行是目标行，则水平只走到 goal.X；否则必须走到边界
                    rowEndX = goal.X; // 本行仅扫描到目标列即可
                }
                else
                { // 非目标行，水平走到边界
                    rowEndX = goRight ? width - 1 : 0; // 按当前方向扫到对应边界（最右或最左）
                }

                int dxSign = goRight ? 1 : -1;  // 当前水平移动方向：向右为 +1，向左为 -1

                // 1) 沿当前方向水平走到 rowEndX
                while (cur.X != rowEndX)
                { // 尚未到达本行水平终点（当前 X 未到达本行目标 X）
                    var next = new GridPos(cur.X + dxSign, cur.Y);   // 计算下一个水平位置
                    if (!IsInBounds(next, width, height) || !isWalkable(next))
                    { // 被障碍或边界打断：返回已生成部分
                        return path;
                    }

                    cur = next;  // 移动到下一个位置
                    path.Add(cur);   // 将新位置加入路径

                    if (cur.Equals(goal))
                    { //移动后到达目标，结束
                        return path;   // 直接返回完整路径
                    }
                }

                // 当前行已经完成：
                // 如果已经是目标行（刚刚走到 goal.X），则结束
                if (cur.Y == goal.Y)
                {
                    break;
                }

                // 2) 纵向移动一格（向 goal.Y 方向）
                var verticalNext = new GridPos(cur.X, cur.Y + verticalSign);   // 计算下一个垂直位置
                if (!IsInBounds(verticalNext, width, height) || !isWalkable(verticalNext))
                { // 被障碍或边界打断：返回已生成部分
                    return path;
                }

                cur = verticalNext;  // 移动到下一个位置（纵向移动）
                path.Add(cur);   // 将新位置加入路径

                if (cur.Equals(goal))
                { // 移动后到达目标，结束
                    break;
                }

                // 3) 新行反转蛇形方向
                goRight = !goRight;  // 每下移一行就反转水平扫描方向（形成蛇形）
            }

            return path;      // 返回生成的路径（成功到达或中途被阻断/保护上限退出）
        }

        private static bool IsInBounds(GridPos p, int width, int height)
        { // 检查位置是否在网格边界内
            return p.X >= 0 && p.Y >= 0 && p.X < width && p.Y < height;
        }
    }
}