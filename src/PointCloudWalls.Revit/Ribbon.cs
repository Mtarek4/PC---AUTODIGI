using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Autodesk.Revit.UI;

namespace PointCloudWalls.Revit
{
    /// <summary>Creates the "XD" ribbon tab with all the tools (loads with Revit).</summary>
    public class XdApp : IExternalApplication
    {
        public const string TabName = "XD";

        public Result OnStartup(UIControlledApplication app)
        {
            try { Build(app); }
            catch (Exception ex)
            {
                // never stop Revit from starting because of a ribbon problem
                TaskDialog.Show("XD ribbon", "The XD tab could not be created:\n\n" + ex.Message);
            }
            return Result.Succeeded;
        }

        public Result OnShutdown(UIControlledApplication app) => Result.Succeeded;

        private static void Build(UIControlledApplication app)
        {
            try { app.CreateRibbonTab(TabName); } catch { /* the tab already exists */ }
            string dll = Assembly.GetExecutingAssembly().Location;

            // ---- Point Cloud Walls -------------------------------------------------------------------------------------------------
            RibbonPanel walls = Panel(app, "Point Cloud Walls");
            walls.AddItem(Button(dll, "XD_WallBoth", "Wall\nBoth Sides", "PointCloudWalls.Revit.PcWallCommand", "wall_both",
                "Model a wall: click a point on each face",
                "A setup popup asks for the level, height and wall type first. Then click one point on each face of the wall. The thickness, direction and length are measured from the point cloud, " +
                "and the wall ends are found automatically. Walls are trimmed and joined to the walls they meet."));
            walls.AddItem(Button(dll, "XD_WallCenter", "Wall\nCenterline", "PointCloudWalls.Revit.PcWallCenterlineCommand", "wall_center",
                "Model a wall: click along its center line",
                "A setup popup asks for the level, height and wall type first. Then click the START and the END of the wall along its middle, roughly. The direction and surfaces are found from the scan. " +
                "Works when a face is missing - click on the wall's body side of the visible dots."));
            walls.AddItem(Button(dll, "XD_WallDiagonal", "Wall\nDiagonal", "PointCloudWalls.Revit.PcWallDiagonalCommand", "wall_diag",
                "Model a wall: click across it, diagonally",
                "A setup popup asks for the level, height and wall type first. Then click the START on one side of the wall and the END on the other side. The wall body lies between your clicks, " +
                "so it works even when one face was not scanned, without guessing the side."));
            walls.AddItem(Button(dll, "XD_WallAuto", "Wall\nAuto", "PointCloudWalls.Revit.PcWallAutoCommand", "wall_auto",
                "Model every wall in this view automatically, without clicking",
                "Finds the walls in the point cloud of the open plan view (the whole view, or an area you drag) and models them the same way the click tools do: " +
                "measured thickness, wall type, ends, trim and join. You see how many walls were found before anything is created, and one Undo removes them all. " +
                "Short or unclear pieces are left for the click tools."));

            // ---- Point Cloud Columns ---------------------------------------------------------------------------------------------------
            RibbonPanel cols = Panel(app, "Point Cloud Columns");
            cols.AddItem(Button(dll, "XD_ColCenter", "Column\nCenter", "PointCloudWalls.Revit.PcColumnCenterCommand", "col_center",
                "Model a structural column: click near its center",
                "Click near the middle of the column. It is detected as round or rectangular automatically, from the point cloud around the click."));
            cols.AddItem(Button(dll, "XD_ColDiagonal", "Column\nDiagonal", "PointCloudWalls.Revit.PcColumnDiagonalCommand", "col_diagonal",
                "Model a rectangular column: click two opposite corners",
                "Click one corner, then the opposite corner. The exact size, position and rotation are measured from the point cloud."));
            cols.AddItem(Button(dll, "XD_ColSides", "Column\nSides", "PointCloudWalls.Revit.PcColumnSidesCommand", "col_sides",
                "Model a rectangular column: click two opposite faces",
                "Click a point on one face and the matching point on the opposite face, like the wall tool. The other pair of faces is found automatically."));

            // ---- Openings (doors/windows) ----------------------------------------------------------------------------------------------
            RibbonPanel openings = Panel(app, "Point Cloud Openings");
            openings.AddItem(Button(dll, "XD_DoorWindow", "Door /\nWindow", "PointCloudWalls.Revit.PcDoorWindowCommand", "door_window",
                "Model a door or window from a gap in a wall",
                "Click inside the opening in a modeled wall. The width, height and sill are measured from the point cloud, and whether it is a door " +
                "(reaches the floor) or a window (has a sill) is worked out automatically - the matching family is placed with that size."));

            // ---- CAD -----------------------------------------------------------------------------------------------------------------------
            RibbonPanel cad = Panel(app, "CAD");
            cad.AddItem(Button(dll, "XD_PublishCad", "Publish\nto CAD", "PointCloudWalls.Revit.PcPublishCadCommand", "publish_cad",
                "Send walls to the open AutoCAD drawing",
                "Publishes the selected walls (or all walls in the active view) to AutoCAD, live. Walls that touch are merged into one clean outline. " +
                "Publishing again updates only what changed. AutoCAD must be running with the PointCloudWalls CAD bridge."));

            // ---- Setup ------------------------------------------------------------------------------------------------------------------------
            RibbonPanel setup = Panel(app, "Setup");
            setup.AddStackedItems(
                Button(dll, "XD_Settings", "Settings", "PointCloudWalls.Revit.PcSettingsCommand", "settings",
                    "Detection, accuracy, wall ends and CAD options",
                    "All settings in plain language, with presets for clean and noisy scans."),
                Button(dll, "XD_WallSetup", "Wall Setup", "PointCloudWalls.Revit.PcWallSetupCommand", "wall_setup",
                    "Level, wall height and the wall type to duplicate from",
                    "The same popup the wall tools show when they start. Use it to change the level, the wall height (fixed, or up to a level) " +
                    "and the wall type that new wall types are duplicated from, and to switch the popup on or off."),
                Button(dll, "XD_ColSetup", "Column Setup", "PointCloudWalls.Revit.PcColumnSetupCommand", "col_setup",
                    "Level, column height and the column type to duplicate from",
                    "The same popup the column tools show when they start. Choose the level, the column height (fixed, or up to a level) " +
                    "and the column family/type that new column types are duplicated from, and switch the popup on or off."));
            // AddStackedItems only supports 2 or 3 items per call - Door/Window Setup goes in as its own regular button
            setup.AddItem(Button(dll, "XD_DoorWindowSetup", "Door/Window\nSetup", "PointCloudWalls.Revit.PcDoorWindowSetupCommand", "door_window_setup",
                "Which door and window family to duplicate from",
                "The same popup the Door/Window tool shows when it starts. Choose the door family and the window family that new types are " +
                "duplicated from, and switch the popup on or off. Width, height and sill height are never set here - they come from the scan."));

            RibbonPanel check = Panel(app, "Check");
            check.AddItem(Button(dll, "XD_Diagnostics", "Diagnostics", "PointCloudWalls.Revit.PcDiagCommand", "diagnostics",
                "Check that the point cloud can be read",
                "Click on a wall in the cloud: shows the clouds in the project, their coordinate system and how many points were found."));
        }

        private static RibbonPanel Panel(UIControlledApplication app, string name)
        {
            RibbonPanel existing = app.GetRibbonPanels(TabName).FirstOrDefault(p => p.Name == name);
            return existing ?? app.CreateRibbonPanel(TabName, name);
        }

        private static PushButtonData Button(string dll, string id, string text, string className, string icon, string tip, string description)
        {
            var d = new PushButtonData(id, text, dll, className);
            d.LargeImage = Icon(icon + "_32.png");
            d.Image = Icon(icon + "_16.png");
            d.ToolTip = tip;
            d.LongDescription = description;
            return d;
        }

        /// <summary>Icons are embedded in the DLL (Icons\*.png), so nothing extra has to be copied next to it.</summary>
        private static ImageSource Icon(string file)
        {
            using (Stream s = Assembly.GetExecutingAssembly().GetManifestResourceStream("PCW.Icons." + file))
            {
                if (s == null) return null;                          // a missing icon must never stop the button from appearing
                var decoder = new PngBitmapDecoder(s, BitmapCreateOptions.PreservePixelFormat, BitmapCacheOption.OnLoad);
                return decoder.Frames[0];
            }
        }
    }
}
