using GridDemo.RobotModels.Pathfinding;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace GridDemo.Maps
{
    /// <summary>
    /// 地图文件读写服务：
    /// - 将障碍物地图序列化为 JSON 并写入文件；
    /// - 从 JSON 文件反序列化并还原障碍物。
    ///
    /// JSON 格式示例：
    /// {
    ///   "GridCount": 30,
    ///   "CellSizeM": 0.55,
    ///   "Obstacles": [[5,5],[5,6],[10,3]]
    /// }
    ///
    /// 使用手写轻量级 JSON 序列化/反序列化，不依赖第三方库，
    /// 兼容 .NET Framework 4.7.2 / C# 7.3。
    /// </summary>
    internal static class MapFileService
    {
        /// <summary>
        /// 从障碍物快照导出障碍物坐标列表，并保存为 JSON 文件。
        /// </summary>
        /// <param name="filePath">目标文件路径。</param>
        /// <param name="gridCount">网格行/列数。</param>
        /// <param name="cellSizeM">每格边长（米）。</param>
        /// <param name="obstacleSnapshot">障碍物快照 bool[x,y]。</param>
        public static void Save(string filePath, int gridCount, double cellSizeM, bool[,] obstacleSnapshot)
        {
            if (obstacleSnapshot == null)
                throw new ArgumentNullException(nameof(obstacleSnapshot));

            string json = SerializeToJson(gridCount, cellSizeM, obstacleSnapshot);
            File.WriteAllText(filePath, json, Encoding.UTF8);
        }

        /// <summary>
        /// 从 JSON 文件加载地图数据。
        /// </summary>
        /// <param name="filePath">JSON 文件路径。</param>
        /// <param name="gridCount">读取到的网格行/列数。</param>
        /// <param name="cellSizeM">读取到的每格边长（米）。</param>
        /// <param name="obstacles">读取到的障碍物坐标列表。</param>
        public static void Load(string filePath, out int gridCount, out double cellSizeM, out List<GridPos> obstacles)
        {
            if (!File.Exists(filePath))
                throw new FileNotFoundException("地图文件不存在：" + filePath);

            string json = File.ReadAllText(filePath, Encoding.UTF8);
            DeserializeFromJson(json, out gridCount, out cellSizeM, out obstacles);
        }

        #region JSON 序列化

        /// <summary>
        /// 将网格参数和障碍物快照序列化为格式化的 JSON 字符串。
        /// </summary>
        private static string SerializeToJson(int gridCount, double cellSizeM, bool[,] snapshot)
        {
            var sb = new StringBuilder();
            sb.AppendLine("{");
            sb.AppendLine("  \"GridCount\": " + gridCount + ",");
            sb.AppendLine("  \"CellSizeM\": " + cellSizeM.ToString(System.Globalization.CultureInfo.InvariantCulture) + ",");

            // 收集障碍物坐标
            sb.Append("  \"Obstacles\": [");

            int w = snapshot.GetLength(0);
            int h = snapshot.GetLength(1);
            bool first = true;

            for (int x = 0; x < w; x++)
            {
                for (int y = 0; y < h; y++)
                {
                    if (snapshot[x, y])
                    {
                        if (!first) sb.Append(",");
                        sb.Append("[" + x + "," + y + "]");
                        first = false;
                    }
                }
            }

            sb.AppendLine("]");
            sb.AppendLine("}");
            return sb.ToString();
        }

        #endregion

        #region JSON 反序列化

        /// <summary>
        /// 从 JSON 字符串反序列化网格参数和障碍物列表。
        /// </summary>
        private static void DeserializeFromJson(string json, out int gridCount, out double cellSizeM, out List<GridPos> obstacles)
        {
            gridCount = 0;
            cellSizeM = 0.0;
            obstacles = new List<GridPos>();

            int pos = 0;

            SkipWhitespace(json, ref pos);
            Expect(json, ref pos, '{');

            while (pos < json.Length)
            {
                SkipWhitespace(json, ref pos);
                if (pos >= json.Length) break;
                if (json[pos] == '}') break;
                if (json[pos] == ',') { pos++; continue; }

                string key = ReadString(json, ref pos);
                SkipWhitespace(json, ref pos);
                Expect(json, ref pos, ':');
                SkipWhitespace(json, ref pos);

                switch (key)
                {
                    case "GridCount":
                        gridCount = ReadInt(json, ref pos);
                        break;
                    case "CellSizeM":
                        cellSizeM = ReadDouble(json, ref pos);
                        break;
                    case "Obstacles":
                        obstacles = ReadObstacleArray(json, ref pos);
                        break;
                    default:
                        SkipValue(json, ref pos);
                        break;
                }
            }

            if (gridCount <= 0)
                throw new FormatException("地图文件 GridCount 必须大于 0");
            if (cellSizeM <= 0)
                throw new FormatException("地图文件 CellSizeM 必须大于 0");
        }

        private static void Expect(string json, ref int pos, char expected)
        {
            if (pos >= json.Length || json[pos] != expected)
                throw new FormatException("期望 '" + expected + "'，位置：" + pos);
            pos++;
        }

        private static string ReadString(string json, ref int pos)
        {
            SkipWhitespace(json, ref pos);
            if (pos >= json.Length || json[pos] != '"')
                throw new FormatException("期望字符串，位置：" + pos);

            pos++; // 跳过开头 '"'
            var sb = new StringBuilder();
            while (pos < json.Length)
            {
                char c = json[pos];
                if (c == '"') { pos++; return sb.ToString(); }
                if (c == '\\')
                {
                    pos++;
                    if (pos < json.Length) sb.Append(json[pos]);
                }
                else
                {
                    sb.Append(c);
                }
                pos++;
            }
            throw new FormatException("字符串未正确关闭");
        }

        private static int ReadInt(string json, ref int pos)
        {
            SkipWhitespace(json, ref pos);
            int start = pos;
            if (pos < json.Length && (json[pos] == '-' || json[pos] == '+')) pos++;
            while (pos < json.Length && json[pos] >= '0' && json[pos] <= '9') pos++;

            string s = json.Substring(start, pos - start);
            if (!int.TryParse(s, out int result))
                throw new FormatException("无法解析整数：'" + s + "'");
            return result;
        }

        private static double ReadDouble(string json, ref int pos)
        {
            SkipWhitespace(json, ref pos);
            int start = pos;
            // 允许 -/+ 开头，数字、小数点、科学计数法
            while (pos < json.Length)
            {
                char c = json[pos];
                if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E')
                    pos++;
                else
                    break;
            }

            string s = json.Substring(start, pos - start);
            if (!double.TryParse(s, System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out double result))
                throw new FormatException("无法解析浮点数：'" + s + "'");
            return result;
        }

        /// <summary> 读取障碍物数组：[[x,y],[x,y],...]。 </summary>
        private static List<GridPos> ReadObstacleArray(string json, ref int pos)
        {
            var list = new List<GridPos>();
            SkipWhitespace(json, ref pos);
            Expect(json, ref pos, '[');

            while (pos < json.Length)
            {
                SkipWhitespace(json, ref pos);
                if (pos >= json.Length) break;
                if (json[pos] == ']') { pos++; break; }
                if (json[pos] == ',') { pos++; continue; }

                // 读取 [x, y]
                Expect(json, ref pos, '[');
                int x = ReadInt(json, ref pos);
                SkipWhitespace(json, ref pos);
                if (pos < json.Length && json[pos] == ',') pos++;
                int y = ReadInt(json, ref pos);
                SkipWhitespace(json, ref pos);
                Expect(json, ref pos, ']');

                list.Add(new GridPos(x, y));
            }

            return list;
        }

        private static void SkipWhitespace(string json, ref int pos)
        {
            while (pos < json.Length && (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\r' || json[pos] == '\n'))
                pos++;
        }

        /// <summary> 跳过一个 JSON 值（用于忽略未知字段）。 </summary>
        private static void SkipValue(string json, ref int pos)
        {
            SkipWhitespace(json, ref pos);
            if (pos >= json.Length) return;

            char c = json[pos];
            if (c == '"') { ReadString(json, ref pos); }
            else if (c == '{') { SkipBlock(json, ref pos, '{', '}'); }
            else if (c == '[') { SkipBlock(json, ref pos, '[', ']'); }
            else
            {
                while (pos < json.Length && json[pos] != ',' && json[pos] != '}' && json[pos] != ']')
                    pos++;
            }
        }

        private static void SkipBlock(string json, ref int pos, char open, char close)
        {
            if (pos >= json.Length || json[pos] != open) return;
            int depth = 1;
            pos++;
            bool inStr = false;
            while (pos < json.Length && depth > 0)
            {
                char c = json[pos];
                if (inStr) { if (c == '\\') pos++; else if (c == '"') inStr = false; }
                else { if (c == '"') inStr = true; else if (c == open) depth++; else if (c == close) depth--; }
                pos++;
            }
        }

        #endregion
    }
}