namespace Graphic.WorldView.CenterGrid
{
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
