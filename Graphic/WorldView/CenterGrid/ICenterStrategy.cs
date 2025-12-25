namespace GridDemo.WorldView.CenterGrid
{
    /// <summary>
    /// 网格居中策略接口（策略模式）。
    /// 用途：将“如何计算/设置视图居中（scale/offset/transform 更新）”的逻辑抽象出来，
    /// 以便在不同触发时机（首次加载、窗口缩放、重置视图等）使用不同实现。
    /// </summary>
    internal interface ICenterStrategy
    {
        /// <summary>
        /// 执行一次“居中网格/将视图对齐到合适位置”的操作。
        /// 具体实现通常会：
        /// - 计算合适的缩放比例（scale）；
        /// - 设置视图偏移（offsetX/offsetY）；
        /// - 并同步更新世界到屏幕的变换（例如调用 WorldTransform.Update）。
        /// </summary>
        void CenterGrid();
    }
}
