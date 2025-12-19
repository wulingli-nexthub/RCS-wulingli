using Graphic.Draws;
using Graphic.Events;
using Graphic.RobotRuns;
using Graphic.WorldView;
using Graphic.WorldView.CenterGrid;

namespace Graphic
{
    public partial class Form1
    {
        /// <summary>
        /// 初始化所有与绘图、交互相关的组件：
        /// WorldTransform / DrawGrid / DrawRobot / MouseWheel / MousePan / CenterGridManager
        /// </summary>
        private void Initialize()
        {
            // World 变换
            _worldTransform = new WorldTransform(_scale, (float)_offsetX, (float)_offsetY);

            // 网格
            _drawGrid = new DrawGrid(_worldTransform, _worldWidthM, _worldHeightM);

            // 机器人
            _drawRobot = new DrawRobot(
                _worldTransform,
                _robotLock,
                () => (_robotX, _robotY),
                () => _scale,
                () => _robot.OrientationAngle
            );

            // 鼠标滚轮缩放
            _mouseWheel = new MouseWheel(
                this,
                // 传入获取当前状态的委托
                getState: () => (_scale, _offsetX, _offsetY),
                // 设置缩放
                setScale: s => _scale = s,
                // 设置偏移
                setOffset: (ox, oy) =>
                {
                    _offsetX = ox;
                    _offsetY = oy;
                },
                // 更新 WorldTransform
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            // 鼠标拖动
            _mousePan = new MousePan(
                this,
                // 获取当前 offset 和 scale
                getState: () => (_offsetX, _offsetY, _scale),
                // 设置 offset
                setOffset: (ox, oy) =>
                {
                    _offsetX = ox;
                    _offsetY = oy;
                },
                // 更新 WorldTransform
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            // 机器人对象：这里使用当前加速度、最大速度，默认向右运动
            _robot = new Robot(
                acc: _robotAcc,
                maxSpeed: _robotMaxSpeed,
                direction: EnumMoveDirection.Right
            );

            // 居中策略：加载 / Resize / Reset
            var loadCenter = new LoadCenterStrategy(
                host: this,
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
                host: this,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                getScale: () => _scale,
                setOffsetX: x => _offsetX = x,
                setOffsetY: y => _offsetY = y,
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            var resetCenter = new ResetCenterStrategy(
                host: this,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                getInitialScale: () => _initialScale,
                setScale: s => _scale = s,
                setOffsetX: x => _offsetX = x,
                setOffsetY: y => _offsetY = y,
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            _centerGridManager = new CenterGridManager(loadCenter, resizeCenter, resetCenter);
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
                getForwardAcc: () => _robotAcc   // 新增这一项
            );

            // 创建模拟器，让它在线程里调用 RobotMove.Update()
            _robotSimulator = new RobotSimulator(
                host: this,      // 用 Form 作为承载控件
                motion: _robotMove,
                dt: _dt
            );
            _robotSimulator._Thead_Start();
        }
    }
}