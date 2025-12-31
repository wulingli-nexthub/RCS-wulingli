//using Graphic.Maps;
//using GridDemo.Draws;
//using GridDemo.Events;
//using GridDemo.RobotModels;
//using GridDemo.RobotRuns;
//using GridDemo.WorldView;
//using GridDemo.WorldView.CenterGrid;

//namespace GridDemo
//{
//    public partial class Form1
//    {
//        private RobotManual _robotManual;
//        private RobotAutoNavigator _robotAutoNavigator;
//        private DrawPath _drawPath;
//        private WorldTransform _worldTransform;
//        private DrawGrid _drawGrid;
//        private DrawRobot _drawRobot;
//        private MouseWheel _mouseWheel;
//        private MousePan _mousePan;
//        private CenterGrid _centerGrid;
//        private RobotManager _robotManager;
//        private RobotMove _robotMove;
//        private RobotSimulator _robotSimulator;
//        private DestinationPicker _destinationPicker;
//        private ObstacleMap _obstacleMap;
//        private DrawObstacles _drawObstacles;

//        private void Initialize()
//        {
//            _worldTransform = new WorldTransform(_scale, (float)_offsetX, (float)_offsetY);
//            _drawGrid = new DrawGrid(_worldTransform, _worldWidthM, _worldHeightM);

//            _obstacleMap = new ObstacleMap(GridCount, GridCount);
//            _drawObstacles = new DrawObstacles(
//                _worldTransform,
//                getObstacleSnapshot: () => _obstacleMap.GetSnapshot(),
//                getCellSizeM: () => CellSizeM
//            );
//            _drawRobot = new DrawRobot(
//                _worldTransform,
//                _robotLock,
//                () => (_robotX, _robotY),
//                () => _scale,
//                () => _robotManager.OrientationAngle
//            );

//            _mouseWheel = new MouseWheel(
//                this,
//                getState: () => (_scale, _offsetX, _offsetY),
//                setScale: s => _scale = s,
//                setOffset: (ox, oy) =>
//                {
//                    _offsetX = ox;
//                    _offsetY = oy;
//                },
//                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
//            );

//            _mousePan = new MousePan(
//                this,
//                getState: () => (_offsetX, _offsetY, _scale),
//                setOffset: (ox, oy) =>
//                {
//                    _offsetX = ox;
//                    _offsetY = oy;
//                },
//                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
//            );

//            _robotManager = new RobotManager(
//                acc: _robotAcc,
//                maxSpeed: _robotMaxSpeed,
//                direction: EnumMoveDirection.Right
//            );

//            _centerGrid = new CenterGrid(
//                host: skControl,
//                getWorldWidthM: () => _worldWidthM,
//                getWorldHeightM: () => _worldHeightM,
//                setScale: s => _scale = s,
//                setOffsetX: x => _offsetX = x,
//                setOffsetY: y => _offsetY = y,
//                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
//            );

//            // 先创建 move（需要 auto 回调，因此先创建 auto 实例或延迟回调）
//            _robotAutoNavigator = new RobotAutoNavigator(
//                robotLock: _robotLock,
//                getRobotX: () => _robotX,
//                getRobotY: () => _robotY,
//                setRobotSpeed: v => _robotSpeed = v,
//                getWorldWidthM: () => _worldWidthM,
//                getWorldHeightM: () => _worldHeightM,
//                cellSizeM: CellSizeM,
//                robotManager: _robotManager
//            );
//            _robotAutoNavigator.SetIsWalkableProvider(p => !_obstacleMap.IsObstacle(p));

//            _robotManual = new RobotManual(
//                robotLock: _robotLock,
//                getCellSizeM: () => CellSizeM,
//                robotManager: _robotManager
//            );

//            _drawPath = new DrawPath(
//                _worldTransform,
//                getPathPointsSnapshot: () => _robotAutoNavigator.GetPathWorldPointsSnapshot()
//            );

//            _destinationPicker = new DestinationPicker(
//                transform: _worldTransform,
//                navigator: _robotAutoNavigator,
//                getWorldWidthM: () => _worldWidthM,
//                getWorldHeightM: () => _worldHeightM,
//                cellSizeM: CellSizeM);
//        }
//    }
//}