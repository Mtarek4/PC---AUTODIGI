using System;
using System.Collections.Generic;
using System.IO;

namespace PointCloudWalls.Cad.AutoCAD
{
    /// <summary>
    /// Which dynamic block represents a door and which represents a window, and which drawing they come from - set
    /// once with PCWSETBLOCKS and remembered per Windows user, the same way Revit's own settings persist. This lives
    /// entirely on the AutoCAD side: dynamic blocks and their parameter names are a DWG-authoring concern, not
    /// something Revit has any way to know about.
    /// </summary>
    internal sealed class BlockConfig
    {
        public string SourceDwgPath = "";
        public string DoorBlockName = "";
        public string WindowBlockName = "";

        public static string FilePath =>
            Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "PointCloudWalls", "cad-blocks.ini");

        public bool IsComplete => SourceDwgPath.Length > 0 && File.Exists(SourceDwgPath) && DoorBlockName.Length > 0 && WindowBlockName.Length > 0;

        public static BlockConfig Load()
        {
            var c = new BlockConfig();
            try
            {
                if (!File.Exists(FilePath)) return c;
                foreach (var line in File.ReadAllLines(FilePath))
                {
                    int eq = line.IndexOf('=');
                    if (eq < 0) continue;
                    string k = line.Substring(0, eq), v = line.Substring(eq + 1);
                    if (k == "SourceDwgPath") c.SourceDwgPath = v;
                    else if (k == "DoorBlockName") c.DoorBlockName = v;
                    else if (k == "WindowBlockName") c.WindowBlockName = v;
                }
            }
            catch { }
            return c;
        }

        public void Save()
        {
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(FilePath));
                File.WriteAllLines(FilePath, new[]
                {
                    "SourceDwgPath=" + SourceDwgPath,
                    "DoorBlockName=" + DoorBlockName,
                    "WindowBlockName=" + WindowBlockName,
                });
            }
            catch { }
        }

        public string BlockNameFor(string kind) => kind == "door" ? DoorBlockName : kind == "window" ? WindowBlockName : "";
    }
}
