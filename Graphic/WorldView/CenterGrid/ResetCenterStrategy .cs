using System;
using System.Windows.Forms;

namespace GridDemo.WorldView.CenterGrid
{
    /// <summary>
    /// 重置视图的网格居中策略：将视图恢复到“初始缩放比例”并重新计算居中偏移。
    /// 典型使用场景：用户点击“重置/回到初始视图”按钮。
    /// </summary>
    internal class ResetCenterStrategy : ICenterStrategy
    {
        private readonly Control _host;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly Func<double> _getInitialScale;
        private readonly Action<double> _setScale;

        private readonly Action<double> _setOffsetX;
        private readonly Action<double> _setOffsetY;

        private readonly Action<double, float, float> _updateWorldTransform;

        public ResetCenterStrategy(
            Control host,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            Func<double> getInitialScale,
            Action<double> setScale,
            Action<double> setOffsetX,
            Action<double> setOffsetY,
            Action<double, float, float> updateWorldTransform)
        {
            _host = host ?? throw new ArgumentNullException(nameof(host));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _getInitialScale = getInitialScale ?? throw new ArgumentNullException(nameof(getInitialScale));
            _setScale = setScale ?? throw new ArgumentNullException(nameof(setScale));
            _setOffsetX = setOffsetX ?? throw new ArgumentNullException(nameof(setOffsetX));
            _setOffsetY = setOffsetY ?? throw new ArgumentNullException(nameof(setOffsetY));
            _updateWorldTransform = updateWorldTransform ?? throw new ArgumentNullException(nameof(updateWorldTransform));
        }

        /// <summary>
        /// 执行一次重置居中：
        /// - 读取初始缩放 initScale；
        /// - 按 initScale 计算世界在屏幕上的像素尺寸；
        /// - 重新计算 offset，使网格在当前视口居中；
        /// - 更新 WorldTransform 并触发重绘。
        /// </summary>
        public void CenterGrid()
        {
            int clientWidth = _host.ClientSize.Width;
            int clientHeight = _host.ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();

            double initScale = _getInitialScale();    // 重置到“初始缩放”（由加载时记录的初始值决定）
            _setScale(initScale);

            double gridPixelWidth = worldWidth * initScale;          // 当前缩放下，世界在屏幕中的像素尺寸
            double gridPixelHeight = worldHeight * initScale;

            double newOffsetX = (clientWidth - gridPixelWidth) / 2.0;        // 基于当前视口尺寸，重新计算偏移，使世界居中显示
            double newOffsetY = (clientHeight - gridPixelHeight) / 2.0;

            _setOffsetX(newOffsetX);
            _setOffsetY(newOffsetY);

            _updateWorldTransform(initScale, (float)newOffsetX, (float)newOffsetY);             // 更新 WorldTransform

            _host.Invalidate();                             // 触发重绘
        }
    }
}
