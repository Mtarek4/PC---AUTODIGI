# PointCloudWalls  (Revit 2024 + AutoCAD 2024 prototype)

src/PointCloudWalls.Core            wall-fitting math (netstandard2.0, meters, no Revit reference)
src/PointCloudWalls.Revit           Revit 2024 add-in, shown as the "XD" ribbon tab: Wall (both sides / centerline / diagonal / auto), Wall Setup,
                                     Column (center / diagonal / sides), Column Setup, Publish to CAD, Settings, Diagnostics
src/PointCloudWalls.Cad.Protocol    Revit <-> AutoCAD connector: messages, named pipe, sync logic (no third-party dependencies)
src/PointCloudWalls.Cad.AutoCAD     AutoCAD plugin (bundle) for 2025/2026 (.NET 8) or 2024 (.NET Framework 4.8): listens for Revit and draws the walls
tests/PointCloudWalls.Tests         synthetic scan scenarios + connector tests (optional, .NET 8)

## Build + install
Close Revit AND AutoCAD first.
1. Open PointCloudWalls.sln in Visual Studio 2022.
2. Right-click PointCloudWalls.Revit -> Build.   (Revit path: <RevitPath> in its .csproj)
3. Right-click PointCloudWalls.Cad.AutoCAD -> Build.   Default = AutoCAD 2025 (.NET 8).
   For AutoCAD 2024 change <AcadYear> to 2024 at the top of its .csproj. (AutoCAD folder is derived from the year.)
   Builds copy everything to:  %APPDATA%\Autodesk\Revit\Addins\2024   and   %APPDATA%\Autodesk\ApplicationPlugins\PointCloudWalls.CadBridge.bundle

## Live CAD session
1. Start AutoCAD and open a drawing - the bridge loads with it and prints "PointCloudWalls CAD bridge is listening".
   (AutoCAD commands: PCWSTATUS, PCWLISTEN, PCWSTOP, PCWCLEAR)
2. In Revit, XD tab -> CAD panel -> "Publish to CAD". Select walls first to publish only those (plus the walls they touch).
   Walls that touch or overlap are merged into ONE outline like Revit's joined walls (PC Settings > Publish to CAD > "Merge walls that touch").

## Ribbon icons
src/PointCloudWalls.Revit/Icons/*.png (32 px + 16 px) are embedded in the DLL. tools/make_icons.py redraws them (needs Python + Pillow).

## Wall setup popup
Each wall tool opens a popup for the base level, wall height (fixed or up to a level) and the wall type to duplicate from, unless turned off in PC Settings (or from the Wall Setup button, which opens the same popup without starting a tool). Choices are remembered between projects by name.

## Wall Auto (no clicks)
Point Cloud Walls panel > Wall Auto: finds the walls in the slice of the open plan view (the whole view, or a rectangle you drag) and builds each one through the same code as the click tools - measured thickness, wall type, ends, trim/join, quality note in Comments ("automatic mode"). It shows what it found before anything is created; one Undo removes the whole run. Nothing is modeled twice and nothing still missing is dropped: of a wall that is partly in the model (you clicked short of its end, or modeled one piece of it) every stretch that is not modeled yet is built, starting exactly where your piece stops - so it can be run again after changing the view's cut height. A wall whose own faces stop where it gets thicker (a pier or frame block at its end) is followed on to its end face.

How it decides (src/PointCloudWalls.Core/WallDetector.cs + AutoWallFinder.cs, Revit glue in src/PointCloudWalls.Revit/AutoWalls.cs):
- the building's wall directions, then every straight surface along them, then pairs of surfaces a wall-thickness apart with nothing between them;
- each pair is measured by WallFitter (FitDetected) exactly like a clicked wall;
- doors: a wall is modeled THROUGH its door openings, so the door can be hosted in it afterwards (PC Door) and cut its own opening. Two walls on one line with a door-sized gap between them (up to "Carry a wall across openings up to" in PC Settings) are closed up: same thickness and line = ONE wall; different thickness, or one of them scanned from one side only = one of them (the measured one, else the longer) is carried across the opening until it touches the other. A wall end that faces the side of a crossing wall across such a gap is carried on to that wall. In every case only when the 3D cloud shows the wall carrying on above the opening - the end of a wall at a corridor is not a door;
- a wall scanned from one side only is modeled when the 3D cloud shows the floor on one side of the surface and none on the other (body on the side without floor, thickness = the one-face thickness from PC Settings, marked "ASSUMED"), or when it carries on from a measured wall (same thickness);
- short, rough or unclear pieces and surfaces whose wall side is unknown are NOT modeled - they are counted in the summary and left for the click tools;
- only where the building really is: the slice is first cut down to where the FLOOR (or, behind furniture, the ceiling) was scanned (src/PointCloudWalls.Core/InteriorMap.cs). A mirror image of a room in window glass, and whatever stands outside, has walls as sharp as real ones but no floor of its own - without this they were modeled as rooms beyond the facade;
- rectangular columns: piers standing proud of a wall (and free-standing columns) are found from their short faces, measured by ColumnFitter.FitDiagonal from the two detected corners and built with the Column Setup choices ("automatic mode" in Comments). A wall runs on behind a column instead of stopping at it. A pier must face the room (floor in front of it) - the same outline facing away is a window bay. Round columns and columns whose sides are hidden are not found: use the Column tools for those.
PC Settings > "3c. Automatic walls" holds its limits (shortest wall, thinnest/thickest wall, opening width, one-sided walls on/off, columns on/off, smallest column).

Tips: a slice above the door heads and below the ceiling (cut plane ~2.3 m) is the cleanest - no door leaves, no furniture, walls run through above the doors. Walls at an unusual angle (not along the building's main directions) are left for the click tools.

Tests: `PCW_ONLY=auto dotnet run -c Release --project tests/PointCloudWalls.Tests` runs only the automatic-wall tests (a few seconds).

## Columns
Point Cloud Columns panel: Column Center (one click near the middle - detects round or rectangular automatically), Column Diagonal (two clicks at opposite corners, rectangular), Column Sides (two clicks on opposite faces, like the wall tool - the other pair of faces is found automatically). Column Setup (Setup panel) picks the level, height and the column family/type new sizes are duplicated from - needs a loaded family with Width & Depth parameters (rectangular) or a Diameter parameter (round); Revit's standard M_Rectangular Column / M_Round Column work.
