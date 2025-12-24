using Graphic.Draws;
using Graphic.Events;
using Graphic.RobotModels;
using Graphic.RobotRuns;
using Graphic.WorldView;
using Graphic.WorldView.CenterGrid;

namespace Graphic
{
    public partial class Form1
    {
        private RobotManual _robotManual;
        private RobotAutoNavigator _robotAutoNavigator;
        private DrawPath _drawPath;

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

            var loadCenter = new LoadCenterStrategy(
                host: skControl,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                getScale: () => _scale,
                setScale: s => _scale = s,
                setOffsetX: x => _offsetX = x,
                setOffsetY: y => _offsetY = y,
                setInitialScale: s => _initialScale = s,
                setInitialOffsetX: x => _initialOffsetX = x,
                setInitialOffsetY: y => _initialOffsetY = y,
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            var resizeCenter = new ResizeCenterStrategy(
                host: skControl,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                getScale: () => _scale,
                setOffsetX: x => _offsetX = x,
                setOffsetY: y => _offsetY = y,
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            var resetCenter = new ResetCenterStrategy(
                host: skControl,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                getInitialScale: () => _initialScale,
                setScale: s => _scale = s,
                setOffsetX: x => _offsetX = x,
                setOffsetY: y => _offsetY = y,
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            _centerGridManager = new CenterGridManager(loadCenter, resizeCenter, resetCenter);

            // 先创建 move（需要 auto 回调，因此先创建 auto 实例或延迟回调）
            _robotAutoNavigator = new RobotAutoNavigator(
                robotLock: _robotLock,
                getRobotX: () => _robotX,
                getRobotY: () => _robotY,
                setRobotSpeed: v => _robotSpeed = v,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                cellSizeM: CellSizeM,
                robot: _robot
            );

            _drawPath = new DrawPath(
                _worldTransform,
                _robotLock,
                getPathPointsSnapshot: () => _robotAutoNavigator.GetPathWorldPointsSnapshot(),
                getPathIndexSnapshot: () => _robotAutoNavigator.GetPathIndexSnapshot()
            );

            _robotMove = new RobotMove(
                robotLock: _robotLock,
                getRobotX: () => _robotX,
                setRobotX: x => _robotX = x,
                getRobotY: () => _robotY,
                setRobotY: y => _robotY = y,
                getRobotSpeed: () => _robotSpeed,
                setRobotSpeed: v => _robotSpeed = v,
                robot: _robot,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                cellSizeM: CellSizeM,
                dt: _dt,
                getForwardAcc: () => _robotAcc,
                getAutoMotionState: () => _robotAutoNavigator.GetAutoMotionState(() => _robotAcc)
            );

            _robotManual = new RobotManual(
                robotLock: _robotLock,
                getForwardAcc: () => _robotAcc,
                setRobotSpeed: v => _robotSpeed = v,
                robot: _robot,
                turn: _robotMove.TurnController
            );

            _robotSimulator = new RobotSimulator(
                host: skControl,
                motion: _robotMove,
                dt: _dt
            );
            _robotSimulator._Thead_Start();

            _destinationPicker = new DestinationPicker(
                transform: _worldTransform,
                navigator: _robotAutoNavigator,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                cellSizeM: CellSizeM);
        }
    }
}