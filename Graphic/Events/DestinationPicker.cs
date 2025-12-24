using GridDemo.RobotModels;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.WorldView;
using System;
using System.Windows.Forms;

namespace GridDemo.Events
{
    internal sealed class DestinationPicker
    {
        private readonly WorldTransform _transform;
        private readonly RobotAutoNavigator _navigator;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;
        private readonly double _cellSizeM;

        public DestinationPicker(
            WorldTransform transform,
            RobotAutoNavigator navigator,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM)
        {
            _transform = transform ?? throw new ArgumentNullException(nameof(transform));
            _navigator = navigator ?? throw new ArgumentNullException(nameof(navigator));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _cellSizeM = cellSizeM;
        }

        public bool TryPick(MouseEventArgs e)
        {
            if (e == null) throw new ArgumentNullException(nameof(e));

            if (e.Button != MouseButtons.Right)
            {
                return false;
            }

            var world = _transform.ScreenToWorld(e.X, e.Y);

            var size = GetGridSize();
            GridPos goal = WorldToGrid(world.X, world.Y, size.gridW, size.gridH);

            // 如果当前正在自动巡航：立刻重算路径
            _navigator.SetGoal(goal, rebuildIfEnabled: true);

            return true;
        }

        private (int gridW, int gridH) GetGridSize()
        {
            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();

            int gridW = Math.Max(1, (int)Math.Round(worldWidth / _cellSizeM));
            int gridH = Math.Max(1, (int)Math.Round(worldHeight / _cellSizeM));
            return (gridW, gridH);
        }

        private GridPos WorldToGrid(double wx, double wy, int gridW, int gridH)
        {
            int gx = (int)Math.Floor(wx / _cellSizeM);
            int gy = (int)Math.Floor(wy / _cellSizeM);

            if (gx < 0) gx = 0;
            if (gy < 0) gy = 0;
            if (gx >= gridW) gx = gridW - 1;
            if (gy >= gridH) gy = gridH - 1;

            return new GridPos(gx, gy);
        }
    }
}