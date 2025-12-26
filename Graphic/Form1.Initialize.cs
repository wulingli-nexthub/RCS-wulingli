using GridDemo.Draws;
using GridDemo.Events;
using GridDemo.RobotRuns;
using GridDemo.WorldView;
using GridDemo.WorldView.CenterGrid;

namespace GridDemo
{
    public partial class Form1
    {
        private WorldTransform _worldTransform;
        private DrawGrid _drawGrid;
        private Robot _robot;
        private DrawRobot _drawRobot;
        private MouseWheel _mouseWheel;
        private MousePan _mousePan;
        private CenterGrid _centerGrid;

        private void Initialize()
        {
            _worldTransform = new WorldTransform(_scale, (float)_offsetX, (float)_offsetY);
            _drawGrid = new DrawGrid(_worldTransform, _worldWidthM, _worldHeightM);

            _drawRobot = new DrawRobot(
                _worldTransform,
                _robotLock,
                () => (_robotX, _robotY),
                () => _scale,
                () => _robot.OrientationAngle
            );

            _mouseWheel = new MouseWheel(
                this,
                getState: () => (_scale, _offsetX, _offsetY),
                setScale: s => _scale = s,
                setOffset: (ox, oy) =>
                {
                    _offsetX = ox;
                    _offsetY = oy;
                },
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            _mousePan = new MousePan(
                this,
                getState: () => (_offsetX, _offsetY, _scale),
                setOffset: (ox, oy) =>
                {
                    _offsetX = ox;
                    _offsetY = oy;
                },
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            _robot = new Robot(
                acc: _robotAcc,
                maxSpeed: _robotMaxSpeed,
                direction: EnumMoveDirection.Right
            );

            _centerGrid = new CenterGrid(
                host: skControl,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                setScale: s => _scale = s,
                setOffsetX: x => _offsetX = x,
                setOffsetY: y => _offsetY = y,
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );
        }
    }
}