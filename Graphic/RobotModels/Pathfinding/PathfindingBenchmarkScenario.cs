using System;

namespace GridDemo.RobotModels.Pathfinding
{
    internal sealed class PathfindingBenchmarkScenario
    {
        private readonly int _width;
        private readonly int _height;
        private readonly bool[,] _blocked;
        private readonly Random _random;
        private readonly EnumPathfindingAlgorithm _algorithm;

        public PathfindingBenchmarkScenario(int width, int height, int seed, double obstacleRate, EnumPathfindingAlgorithm algorithm)
        {
            if (width <= 0) throw new ArgumentOutOfRangeException(nameof(width));
            if (height <= 0) throw new ArgumentOutOfRangeException(nameof(height));
            if (obstacleRate < 0.0 || obstacleRate > 1.0) throw new ArgumentOutOfRangeException(nameof(obstacleRate));

            _width = width;
            _height = height;
            _algorithm = algorithm;
            _random = new Random(seed);

            _blocked = new bool[width, height];

            // 生成固定障碍（一次性），避免每次寻路都把时间花在造图上
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    _blocked[x, y] = _random.NextDouble() < obstacleRate;
                }
            }

            // 确保至少有可走点（防止全阻塞）
            _blocked[0, 0] = false;
            _blocked[width - 1, height - 1] = false;
        }

        public int RunOnce()
        {
            GridPos start = RandomWalkablePos();
            GridPos goal = RandomWalkablePos();

            var path = GridPathfinder.FindPath(
                width: _width,
                height: _height,
                start: start,
                goal: goal,
                isWalkable: IsWalkable,
                algorithm: _algorithm);

            return path.Count;
        }

        private bool IsWalkable(GridPos p)
        {
            if (p.X < 0 || p.Y < 0 || p.X >= _width || p.Y >= _height)
            {
                return false;
            }

            return !_blocked[p.X, p.Y];
        }

        private GridPos RandomWalkablePos()
        {
            // 简单循环找可走点（障碍率别设太极端即可）
            while (true)
            {
                int x = _random.Next(0, _width);
                int y = _random.Next(0, _height);

                if (!_blocked[x, y])
                {
                    return new GridPos(x, y);
                }
            }
        }
    }
}