using System.Windows.Forms;

namespace GridDemo.Draws
{
    /// <summary>
    /// 启用双缓冲的 ListView，消除高频刷新时的闪烁问题。
    /// </summary>
    internal class DoubleBufferedListView : ListView
    {
        public DoubleBufferedListView()
        {
            // 启用双缓冲，减少绘制闪烁
            SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint, true);

            // 通过反射设置 protected 属性 DoubleBuffered
            DoubleBuffered = true;
        }
    }
}