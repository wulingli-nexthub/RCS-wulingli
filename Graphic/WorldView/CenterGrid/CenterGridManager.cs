namespace GridDemo.WorldView.CenterGrid
{
    /// <summary>
    /// 网格居中策略管理器：将“不同场景下的居中逻辑”按用途聚合在一起。
    /// 目的：Form 或渲染层只需要持有一个管理器，即可在加载/缩放/重置等时刻
    /// 选择对应的策略执行（策略本身的具体算法由各 ICenterStrategy 实现决定）。
    /// </summary>
    internal class CenterGridManager
    {
        public ICenterStrategy LoadCenter { get; }
        public ICenterStrategy ResizeCenter { get; }
        public ICenterStrategy ResetCenter { get; }

        public CenterGridManager(
            ICenterStrategy loadCenter,
            ICenterStrategy resizeCenter,
            ICenterStrategy resetCenter)
        {
            LoadCenter = loadCenter;
            ResizeCenter = resizeCenter;
            ResetCenter = resetCenter;
        }
    }
}
