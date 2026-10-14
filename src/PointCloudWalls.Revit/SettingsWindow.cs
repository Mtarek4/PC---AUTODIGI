using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Interop;
using System.Windows.Media;

namespace PointCloudWalls.Revit
{
    /// <summary>
    /// Plain-language settings window. Built in code (no XAML) so it compiles like any other class.
    /// Controls edit a working copy; "Save" writes settings.ini, "Cancel" discards.
    /// </summary>
    internal sealed class SettingsWindow : Window
    {
        private readonly Settings _s;
        private readonly List<Action<Settings>> _refresh = new List<Action<Settings>>();   // settings -> controls
        private bool _loading;

        public SettingsWindow(Settings current)
        {
            _s = current.Clone();
            Title = "Point Cloud Walls - Settings";
            Width = 620; Height = 780; MinWidth = 500; MinHeight = 480;
            WindowStartupLocation = WindowStartupLocation.CenterScreen;
            try { new WindowInteropHelper(this).Owner = System.Diagnostics.Process.GetCurrentProcess().MainWindowHandle; } catch { }

            var root = new DockPanel();

            var buttons = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Right, Margin = new Thickness(12) };
            DockPanel.SetDock(buttons, Dock.Bottom);
            var reset = new Button { Content = "Reset to defaults", Padding = new Thickness(14, 5, 14, 5), Margin = new Thickness(0, 0, 24, 0) };
            var cancel = new Button { Content = "Cancel", Padding = new Thickness(18, 5, 18, 5), Margin = new Thickness(0, 0, 8, 0) };
            var save = new Button { Content = "Save", Padding = new Thickness(22, 5, 22, 5), IsDefault = true };
            buttons.Children.Add(reset); buttons.Children.Add(cancel); buttons.Children.Add(save);
            root.Children.Add(buttons);

            var scroll = new ScrollViewer { VerticalScrollBarVisibility = ScrollBarVisibility.Auto };
            var body = new StackPanel { Margin = new Thickness(18, 4, 18, 8) };
            scroll.Content = body;
            root.Children.Add(scroll);
            Content = root;

            Build(body);

            reset.Click += (s, e) => { _s.CopyFrom(new Settings()); Refresh(); };
            cancel.Click += (s, e) => Close();
            save.Click += (s, e) =>
            {
                try { _s.Save(); Close(); }
                catch (Exception ex) { MessageBox.Show("Could not save settings:\n" + ex.Message, "Point Cloud Walls"); }
            };
            Refresh();
        }

        private void Refresh()
        {
            _loading = true;
            foreach (var r in _refresh) r(_s);
            _loading = false;
        }

        // ------------------------------------------------------------------------------------------------
        private void Build(StackPanel body)
        {
            // ---- Quick start
            Section(body, "Quick start");
            Note(body, "Pick a starting point that matches your scan, then fine-tune below. Applies to walls AND columns. (Your wall/column height and output choices are kept.)");
            var presetRow = new DockPanel { Margin = new Thickness(0, 6, 0, 0) };
            var apply = new Button { Content = "Apply", Padding = new Thickness(16, 3, 16, 3), Margin = new Thickness(8, 0, 0, 0) };
            DockPanel.SetDock(apply, Dock.Right);
            var presets = new ComboBox();
            foreach (var n in Settings.PresetNames) presets.Items.Add(n);
            presets.SelectedIndex = 0;
            presetRow.Children.Add(apply); presetRow.Children.Add(presets);
            body.Children.Add(presetRow);
            apply.Click += (s, e) => { _s.ApplyPreset(Math.Max(0, presets.SelectedIndex)); Refresh(); };

            // ---- 1. reading the scan
            Section(body, "1.  How the scan is read");
            AddSlider(body, "Slice thickness",
                "The wall is found on a thin horizontal slice of the scan at the plan's cut height. Thinner gives cleaner lines but fewer points; thicker gives more points but door frames and furniture can blur the wall.",
                2, 40, 1, "cm", "0", st => st.SlabHalfM * 200, (st, v) => st.SlabHalfM = v / 200);

            var cut = AddCheck(body, "Slice at the height of this plan view's cut plane",
                "Recommended. Turn off to slice at a fixed height above the level instead.",
                st => double.IsNaN(st.SliceHeightAboveLevelM),
                (st, v) => st.SliceHeightAboveLevelM = v ? double.NaN : (double.IsNaN(st.SliceHeightAboveLevelM) ? 1.2 : st.SliceHeightAboveLevelM));
            var heightPanel = new StackPanel();
            AddSlider(heightPanel, "Slice height above the level", "Only used when the option above is off.",
                0.1, 3.5, 0.05, "m", "0.00",
                st => double.IsNaN(st.SliceHeightAboveLevelM) ? 1.2 : st.SliceHeightAboveLevelM,
                (st, v) => { if (!double.IsNaN(st.SliceHeightAboveLevelM)) st.SliceHeightAboveLevelM = v; });
            body.Children.Add(heightPanel);
            cut.Checked += (s, e) => heightPanel.IsEnabled = false;
            cut.Unchecked += (s, e) => heightPanel.IsEnabled = true;
            _refresh.Add(st => heightPanel.IsEnabled = !double.IsNaN(st.SliceHeightAboveLevelM));

            AddSlider(body, "Point detail",
                "How closely points are sampled from the scan. Smaller numbers use more points: slower, but more precise.",
                2, 30, 1, "mm", "0", st => st.SpacingM * 1000, (st, v) => st.SpacingM = v / 1000);

            // ---- 2. detection sensitivity - SHARED by every category (walls, columns, and anything added later)
            Section(body, "2.  Detection sensitivity  (applies to walls AND columns)");
            Note(body, "These two apply everywhere in the plugin - Wall tools, Column tools, and any category added later. Tune them once here.");
            AddSlider(body, "Scan roughness",
                "How noisy or rough the scanned surface is. Raise it for noisy scans, plaster or dusty surfaces; lower it for very clean scans. Too high can merge a wall or column with things right next to it.",
                2, 30, 1, "mm", "0", st => st.InlierTolM * 1000, (st, v) => st.InlierTolM = v / 1000);
            AddSlider(body, "Click forgiveness",
                "How far your clicks may be from the real surface, in any of the Wall or Column tools.",
                2, 20, 1, "cm", "0", st => st.SeedBandM * 100, (st, v) => st.SeedBandM = v / 100);

            // ---- 3. recognising walls (wall-only from here)
            Section(body, "3.  How walls are recognised");
            AddSlider(body, "Thinnest wall", "Anything thinner than this is rejected as \"not a wall\".",
                30, 300, 5, "mm", "0", st => st.MinThicknessM * 1000, (st, v) => st.MinThicknessM = v / 1000);
            AddSlider(body, "Thickest wall", "Anything thicker than this is rejected as \"not a wall\".",
                200, 1500, 10, "mm", "0", st => st.MaxThicknessM * 1000, (st, v) => st.MaxThicknessM = v / 1000);
            AddSlider(body, "Ignore gaps up to",
                "Scan shadows (behind furniture, pipes) shorter than this do not break a wall. Longer gaps end it: door openings, wide crossing walls.",
                5, 100, 5, "cm", "0", st => st.MaxGapM * 100, (st, v) => st.MaxGapM = v / 100);

            // ---- 2b. centerline mode
            Section(body, "3b.  Wall centerline and diagonal modes");
            Note(body, "\"PC Wall - centerline\": click the START and END along the wall's middle. \"PC Wall - diagonal\": click the START on one side of the wall and the END on the other side - the wall body is then between your clicks, so it works even when one side was not scanned.");
            AddSlider(body, "How exactly must the two clicks follow the center line?",
                "The wall's direction is found from the scan, so your clicks only need to be roughly on the wall. This is how far from the true center they may be.",
                5, 60, 1, "cm", "0", st => st.CenterlineTolM * 100, (st, v) => st.CenterlineTolM = v / 100);
            body.Children.Add(new TextBlock { Text = "Where do the wall ends go?  (centerline and diagonal modes)", FontWeight = FontWeights.SemiBold, Margin = new Thickness(0, 10, 0, 0) });
            Note(body, "Automatic follows the wall to its real ends wherever you clicked along it (it stops at door openings, end faces and other walls). " +
                       "\"Snap\" only looks near your clicks. \"Exactly\" builds the wall between your two clicks.");
            var endMode = new ComboBox { Margin = new Thickness(0, 2, 0, 0) };
            endMode.Items.Add("Find the wall's real ends automatically");
            endMode.Items.Add("Snap to the scan near my clicks");
            endMode.Items.Add("Exactly where I click");
            body.Children.Add(endMode);
            var snapPanel = new StackPanel();
            AddSlider(snapPanel, "Snap distance",
                "Only for \"Snap to the scan near my clicks\": a wall end snaps to a real feature in the scan (an end face, a crossing wall) within this distance of your click. With nothing nearby it stays where you clicked.",
                10, 100, 5, "cm", "0", st => st.EndSnapM * 100, (st, v) => st.EndSnapM = v / 100);
            body.Children.Add(snapPanel);
            // combo order: 0 automatic, 1 snap, 2 exact  <->  stored index: 1 automatic, 0 snap, 2 exact
            Func<int, int> toCombo = idx => idx == 1 ? 0 : idx == 0 ? 1 : 2;
            Func<int, int> fromCombo = ci => ci == 0 ? 1 : ci == 1 ? 0 : 2;
            endMode.SelectionChanged += (s, e) =>
            {
                if (_loading || endMode.SelectedIndex < 0) return;
                _s.EndModeIndex = fromCombo(endMode.SelectedIndex);
                snapPanel.IsEnabled = _s.EndModeIndex == 0;
            };
            _refresh.Add(st => { endMode.SelectedIndex = toCombo(st.EndModeIndex); snapPanel.IsEnabled = st.EndModeIndex == 0; });
            AddSlider(body, "Diagonal mode: how close must each click be to a wall surface?",
                "In \"PC Wall - diagonal\" you click the start on one side of the wall and the end on the other side. Each click must be this close to a scanned surface, or to where a missing surface should be.",
                3, 30, 1, "cm", "0", st => st.DiagonalTolM * 100, (st, v) => st.DiagonalTolM = v / 100);
            var oneFace = AddCheck(body, "Accept walls with only one scanned face",
                "The wall body is placed on the side of the visible face where you clicked. Such walls are marked \"thickness assumed\" and never graded GOOD.",
                st => st.AllowOneFace, (st, v) => st.AllowOneFace = v);
            var thickPanel = new StackPanel();
            AddSlider(thickPanel, "Thickness to use when only one face is visible", "Applied to walls found from a single face.",
                50, 600, 5, "mm", "0", st => st.AssumedThicknessM * 1000, (st, v) => st.AssumedThicknessM = v / 1000);
            body.Children.Add(thickPanel);
            oneFace.Checked += (s, e) => thickPanel.IsEnabled = true;
            oneFace.Unchecked += (s, e) => thickPanel.IsEnabled = false;
            _refresh.Add(st => thickPanel.IsEnabled = st.AllowOneFace);

            // ---- 3c. automatic walls
            Section(body, "3c.  Automatic walls  (Wall Auto)");
            Note(body, "\"Wall Auto\" finds every wall in the open plan view without clicks and measures each one with the same solver as the click tools. " +
                       "Nobody checks each wall while it is made, so it is stricter: short or unclear pieces are left for the click tools.");
            AddSlider(body, "Shortest wall to model automatically", "Shorter pieces are skipped.",
                20, 200, 5, "cm", "0", st => st.AutoMinLengthM * 100, (st, v) => st.AutoMinLengthM = v / 100);
            AddSlider(body, "Thinnest wall to model automatically",
                "Two surfaces closer together than this are treated as ONE doubled surface (two scans slightly out of register, a board or frame on the wall), not as a thin wall.",
                40, 200, 5, "mm", "0", st => st.AutoMinThicknessM * 1000, (st, v) => st.AutoMinThicknessM = v / 1000);
            AddSlider(body, "Thickest wall to model automatically",
                "Two surfaces further apart than this are never taken for the two faces of one wall - that is a corridor, a niche or two walls.",
                200, 1000, 10, "mm", "0", st => st.AutoMaxThicknessM * 1000, (st, v) => st.AutoMaxThicknessM = v / 1000);
            AddSlider(body, "Carry a wall across openings up to",
                "Door and window openings: when the wall is seen above the opening, the pieces on both sides become ONE wall, like clicking the wall's two far ends. 0 = never.",
                0, 300, 10, "cm", "0", st => st.AutoBridgeOpeningM * 100, (st, v) => st.AutoBridgeOpeningM = v / 100);
            var autoOne = AddCheck(body, "Also model walls scanned from one side only",
                "The wall body is put on the side of the surface where NO floor was scanned (the other side is the room). The thickness is the one set above for one-face walls and such walls are marked \"thickness assumed\" - unless the surface carries on from a measured wall, then it takes that wall's thickness. Surfaces whose side is unclear are never modeled.",
                st => st.AutoOneFace, (st, v) => st.AutoOneFace = v);
            var autoOnePanel = new StackPanel();
            AddSlider(autoOnePanel, "Shortest one-sided wall", "One-sided surfaces shorter than this are skipped.",
                50, 300, 10, "cm", "0", st => st.AutoOneFaceMinLengthM * 100, (st, v) => st.AutoOneFaceMinLengthM = v / 100);
            body.Children.Add(autoOnePanel);
            autoOne.Checked += (s, e) => autoOnePanel.IsEnabled = true;
            autoOne.Unchecked += (s, e) => autoOnePanel.IsEnabled = false;
            _refresh.Add(st => autoOnePanel.IsEnabled = st.AutoOneFace);

            var autoCol = AddCheck(body, "Also model the columns",
                "Rectangular columns standing free, and piers standing proud of a wall, are found with the walls and built with the Column Setup choices (level, height, family). " +
                "A wall then runs on behind a column instead of stopping at it. Round columns are not found automatically - use Column Center for those.",
                st => st.AutoColumns, (st, v) => st.AutoColumns = v);
            var autoColPanel = new StackPanel();
            AddSlider(autoColPanel, "Smallest column", "Smaller bumps on a wall (window mullions, pipes, frames) are not columns.",
                10, 60, 1, "cm", "0", st => st.AutoColMinSizeM * 100, (st, v) => st.AutoColMinSizeM = v / 100);
            body.Children.Add(autoColPanel);
            autoCol.Checked += (s, e) => autoColPanel.IsEnabled = true;
            autoCol.Unchecked += (s, e) => autoColPanel.IsEnabled = false;
            _refresh.Add(st => autoColPanel.IsEnabled = st.AutoColumns);

            // ---- 3. accuracy
            Section(body, "4.  Wall accuracy");
            AddCheck(body, "Fine alignment",
                "Rotates and shifts the wall by tiny amounts until both scanned surfaces line up as sharply as possible. Recommended.",
                st => st.RefineAngle, (st, v) => st.RefineAngle = v);
            var follow = AddCheck(body, "Measure along the whole wall",
                "Long walls are re-measured along their full length, so the direction stays accurate at the far end.",
                st => st.FollowWall, (st, v) => st.FollowWall = v);
            var lenPanel = new StackPanel();
            AddSlider(lenPanel, "Longest wall per pair of clicks", "A safety limit.",
                5, 100, 5, "m", "0", st => st.MaxWallLengthM, (st, v) => st.MaxWallLengthM = v);
            body.Children.Add(lenPanel);
            follow.Checked += (s, e) => lenPanel.IsEnabled = true;
            follow.Unchecked += (s, e) => lenPanel.IsEnabled = false;
            _refresh.Add(st => lenPanel.IsEnabled = st.FollowWall);

            AddCheck(body, "Find wall ends automatically",
                "Uses end faces, door jambs and crossing walls to place each wall end exactly where the scan shows it.",
                st => st.DetectEnds, (st, v) => st.DetectEnds = v);

            body.Children.Add(new TextBlock { Text = "When a wall meets another wall", FontWeight = FontWeights.SemiBold, Margin = new Thickness(0, 10, 0, 0) });
            Note(body, "Center line usually gives cleaner wall joins in Revit; face keeps the wall exactly as scanned.");
            var junction = new ComboBox { Margin = new Thickness(0, 2, 0, 0) };
            junction.Items.Add("Stop at the other wall's face");
            junction.Items.Add("Continue to the other wall's center line");
            body.Children.Add(junction);
            junction.SelectionChanged += (s, e) => { if (!_loading && junction.SelectedIndex >= 0) _s.JunctionToCenterline = junction.SelectedIndex == 1; };
            _refresh.Add(st => junction.SelectedIndex = st.JunctionToCenterline ? 1 : 0);

            // ---- 4. output
            Section(body, "5.  Walls: what is created in Revit");
            AddCheck(body, "Ask for level, height and wall type each time a wall tool starts",
                "Shows the Wall setup popup. Turn off to always use the last choices (change them any time with the Wall Setup button).",
                st => st.AskWallSetup, (st, v) => st.AskWallSetup = v);
            AddSlider(body, "Default wall height", "Used as the starting value in the Wall setup popup.",
                2, 8, 0.1, "m", "0.0", st => st.WallHeightM, (st, v) => st.WallHeightM = v);
            AddSlider(body, "Reuse existing types within",
                "Shared with columns. If a wall type (or column family) this close to the measured size already exists it is used. Otherwise a new one with the exact measured size is created. 0 = always exact.",
                0, 30, 1, "mm", "0", st => st.TypeSnapM * 1000, (st, v) => st.TypeSnapM = v / 1000);
            var join = AddCheck(body, "Join and trim walls automatically",
                "After each wall is created its ends are trimmed or extended to the center line of the walls they meet (T-junctions and corners), and Revit joins are applied so the junction cleans up. Only walls created by this tool are ever moved; your other walls are never changed.",
                st => st.AutoJoin, (st, v) => st.AutoJoin = v);
            var reachPanel = new StackPanel();
            AddSlider(reachPanel, "Join reach",
                "How far a wall end may be from another wall's surface and still be joined to it. Raise it if walls that should join are left with small gaps; lower it if walls join things they should not.",
                2, 50, 1, "cm", "0", st => st.JoinReachM * 100, (st, v) => st.JoinReachM = v / 100);
            body.Children.Add(reachPanel);
            join.Checked += (s, e) => reachPanel.IsEnabled = true;
            join.Unchecked += (s, e) => reachPanel.IsEnabled = false;
            _refresh.Add(st => reachPanel.IsEnabled = st.AutoJoin);

            AddCheck(body, "Draw check marks for walls",
                "Red = fitted wall surfaces, green = centerline, blue = where the wall starts and ends. Turn off once you trust the result.",
                st => st.DebugLines, (st, v) => st.DebugLines = v);
            AddCheck(body, "Save a quality note in the wall's Comments",
                "Writes Good / OK / Check plus the measured values, so walls that need a second look can be found and filtered.",
                st => st.QualityNote, (st, v) => st.QualityNote = v);

            // ---- 6. columns
            Section(body, "6.  Columns");
            Note(body, "Scan roughness and click forgiveness for columns are set together with walls, in section 2 above.");
            AddSlider(body, "Column Center: search radius",
                "How far around your click to look for a column, in \"PC Column - Center\". Set it a little larger than your biggest column.",
                0.3, 2.5, 0.1, "m", "0.00", st => st.ColSearchRadiusM, (st, v) => st.ColSearchRadiusM = v);
            AddSlider(body, "Smallest column", "Anything smaller than this is rejected as \"not a column\" (width/depth, or diameter).",
                50, 300, 5, "mm", "0", st => st.ColMinSizeM * 1000, (st, v) => st.ColMinSizeM = v / 1000);
            AddSlider(body, "Largest column", "Anything larger than this is rejected as \"not a column\" (width/depth, or diameter).",
                300, 2000, 10, "mm", "0", st => st.ColMaxSizeM * 1000, (st, v) => st.ColMaxSizeM = v / 1000);
            AddSlider(body, "How much of a round column must be visible",
                "A round column is only reported as round once at least this much of its circumference has scan points. Lower it for columns often scanned from only one side (e.g. standing against a wall); raise it to be more certain before calling something round.",
                40, 90, 5, "%", "0", st => st.ColRoundCoverage * 100, (st, v) => st.ColRoundCoverage = v / 100);

            var autoH = AddCheck(body, "Search nearby slice heights automatically",
                "Point cloud density often varies a lot with height on a column - dense near a slab or beam, sparse in between (for example, if the plan view's cut plane happens to land in a sparse band). " +
                "When this is on, each pick quickly checks a few heights near the usual slice height and uses whichever has the most data, instead of always using the one fixed height.",
                st => st.ColAutoHeight, (st, v) => st.ColAutoHeight = v);
            var heightSearchPanel = new StackPanel();
            AddSlider(heightSearchPanel, "Search range", "How far above and below the usual slice height to look.",
                20, 150, 5, "cm", "0", st => st.ColHeightSearchM * 100, (st, v) => st.ColHeightSearchM = v / 100);
            AddSlider(heightSearchPanel, "Search step", "How finely to step through that range. Smaller finds a good height more precisely but checks more heights each time.",
                5, 30, 5, "cm", "0", st => st.ColHeightStepM * 100, (st, v) => st.ColHeightStepM = v / 100);
            body.Children.Add(heightSearchPanel);
            autoH.Checked += (s, e) => heightSearchPanel.IsEnabled = true;
            autoH.Unchecked += (s, e) => heightSearchPanel.IsEnabled = false;
            _refresh.Add(st => heightSearchPanel.IsEnabled = st.ColAutoHeight);

            body.Children.Add(new TextBlock { Text = "Shapes to detect in Column Center", FontWeight = FontWeights.SemiBold, Margin = new Thickness(0, 10, 0, 0) });
            Note(body, "\"PC Column - Diagonal\" and \"PC Column - Sides\" always look for rectangular columns. This only affects \"PC Column - Center\", " +
                       "which normally works out the shape on its own - restrict it if you already know what you are clicking.");
            var colShape = new ComboBox { Margin = new Thickness(0, 2, 0, 0) };
            colShape.Items.Add("Either shape (recommended)");
            colShape.Items.Add("Rectangular only");
            colShape.Items.Add("Round only");
            body.Children.Add(colShape);
            colShape.SelectionChanged += (s, e) => { if (!_loading && colShape.SelectedIndex >= 0) _s.ColShapeMode = colShape.SelectedIndex; };
            _refresh.Add(st => colShape.SelectedIndex = Math.Max(0, Math.Min(2, st.ColShapeMode)));

            // ---- 5. publish to CAD
            // ---- doors and windows
            Section(body, "6b.  Doors and windows");
            Note(body, "Scan roughness and click forgiveness are set together with walls and columns, in section 2 above.");
            AddSlider(body, "Door/Window: search range", "How far each side of your click to look along the wall for the opening.",
                0.8, 3.0, 0.1, "m", "0.00", st => st.OpenSearchHalfWidthM, (st, v) => st.OpenSearchHalfWidthM = v);
            AddSlider(body, "Smallest opening width", "Anything narrower is rejected as \"not a real opening\".",
                200, 800, 10, "mm", "0", st => st.OpenMinWidthM * 1000, (st, v) => st.OpenMinWidthM = v / 1000);
            AddSlider(body, "Largest opening width", "Anything wider is rejected as \"not a real opening\" (e.g. an open storefront, not a single door or window).",
                1000, 5000, 50, "mm", "0", st => st.OpenMaxWidthM * 1000, (st, v) => st.OpenMaxWidthM = v / 1000);
            AddSlider(body, "Smallest opening height", "Anything shorter is rejected.",
                200, 800, 10, "mm", "0", st => st.OpenMinHeightM * 1000, (st, v) => st.OpenMinHeightM = v / 1000);
            AddSlider(body, "Largest opening height", "Anything taller is rejected.",
                1000, 5000, 50, "mm", "0", st => st.OpenMaxHeightM * 1000, (st, v) => st.OpenMaxHeightM = v / 1000);
            AddSlider(body, "Floor tolerance", "An opening whose bottom is within this of the wall's base counts as reaching the floor (a door); above it, a window with a sill.",
                20, 300, 10, "mm", "0", st => st.OpenFloorTolM * 1000, (st, v) => st.OpenFloorTolM = v / 1000);
            AddSlider(body, "Required jamb/header support", "How much of each side bordering the opening must have real scan points before it is trusted as a genuine opening, not noise.",
                20, 80, 5, "%", "0", st => st.OpenMinJambCoverage * 100, (st, v) => st.OpenMinJambCoverage = v / 100);
            AddSlider(body, "Bridge obstructions up to", "A person standing in a doorway (or something left leaning in it) can look like solid wall and split one real opening into two too-narrow ones. " +
                "Anything this wide or narrower is ignored, as long as it does not reach the top of the opening - a real jamb or pier is never affected by this.",
                0, 1500, 25, "mm", "0", st => st.OpenBridgeMaxWidthM * 1000, (st, v) => st.OpenBridgeMaxWidthM = v / 1000);

            Section(body, "7.  Publish to CAD (AutoCAD)");
            Note(body, "\"PC Publish to CAD\" sends the selected walls and columns (or all of both in the active view) to the AutoCAD that is open on this computer. " +
                       "Publishing again updates what changed and never touches anything you drew yourself in CAD.");
            body.Children.Add(new TextBlock { Text = "Drawing units", FontWeight = FontWeights.SemiBold, Margin = new Thickness(0, 10, 0, 0) });
            Note(body, "Match the units of your AutoCAD drawing (most Egyptian projects use millimetres).");
            var cadUnits = new ComboBox { Margin = new Thickness(0, 2, 0, 0) };
            cadUnits.Items.Add("Millimetres"); cadUnits.Items.Add("Centimetres"); cadUnits.Items.Add("Metres");
            body.Children.Add(cadUnits);
            cadUnits.SelectionChanged += (s, e) => { if (!_loading && cadUnits.SelectedIndex >= 0) _s.CadUnitsIndex = cadUnits.SelectedIndex; };
            _refresh.Add(st => cadUnits.SelectedIndex = st.CadUnitsIndex);

            body.Children.Add(new TextBlock { Text = "Coordinates", FontWeight = FontWeights.SemiBold, Margin = new Thickness(0, 10, 0, 0) });
            Note(body, "Revit origin: the drawing starts at the Revit internal origin. Shared coordinates: uses the project's survey / project base point.");
            var cadCoords = new ComboBox { Margin = new Thickness(0, 2, 0, 0) };
            cadCoords.Items.Add("Revit internal origin"); cadCoords.Items.Add("Shared coordinates");
            body.Children.Add(cadCoords);
            cadCoords.SelectionChanged += (s, e) => { if (!_loading && cadCoords.SelectedIndex >= 0) _s.CadCoordinatesIndex = cadCoords.SelectedIndex; };
            _refresh.Add(st => cadCoords.SelectedIndex = st.CadCoordinatesIndex);

            AddCheck(body, "Also publish columns",
                "Structural columns are sent to their own layer, each as its own outline - not merged with walls or with each other, matching how columns are normally drawn in CAD.",
                st => st.CadIncludeColumns, (st, v) => st.CadIncludeColumns = v);
            AddText(body, "Layer for columns", "", st => st.CadColumnLayer, (st, v) => { if (v.Length > 0) st.CadColumnLayer = v; });

            AddCheck(body, "Merge walls that touch into one outline",
                "Like Revit's joined walls: where walls meet (T, L, corners, crossings) the lines between them disappear and the walls become ONE clean outline. " +
                "A room closed by walls gets an outer and an inner outline. Turn off to draw every wall as its own rectangle.",
                st => st.CadMergeWalls, (st, v) => st.CadMergeWalls = v);
            AddText(body, "Layer for wall outlines", "The layer is created if it does not exist. An existing layer keeps its own colour.",
                st => st.CadOutlineLayer, (st, v) => { if (v.Length > 0) st.CadOutlineLayer = v; });
            AddCheck(body, "Also draw each wall's location line", "A line along the wall's location line, on its own layer.",
                st => st.CadCenterline, (st, v) => st.CadCenterline = v);
            AddText(body, "Layer for location lines", "", st => st.CadCenterLayer, (st, v) => { if (v.Length > 0) st.CadCenterLayer = v; });
            AddCheck(body, "Remove walls from CAD that were deleted in Revit",
                "Only when you publish everything in the view (nothing selected), and only entities this connector drew.",
                st => st.CadRemoveDeleted, (st, v) => st.CadRemoveDeleted = v);
            AddCheck(body, "Zoom to the walls after the first publish", "", st => st.CadZoom, (st, v) => st.CadZoom = v);

            Note(body, "\nChanges apply to your next click. Settings are stored in " + Settings.FilePath);
        }

        // ------------------------------------------------------------------------------------------------
        private static void Section(StackPanel p, string text)
        {
            p.Children.Add(new TextBlock { Text = text, FontSize = 15, FontWeight = FontWeights.Bold, Margin = new Thickness(0, 18, 0, 2) });
            p.Children.Add(new Separator());
        }

        private static void Note(StackPanel p, string text)
        {
            p.Children.Add(new TextBlock { Text = text, Foreground = Brushes.Gray, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(0, 0, 0, 2) });
        }

        private void AddSlider(StackPanel p, string title, string help, double min, double max, double step, string unit, string fmt,
                               Func<Settings, double> get, Action<Settings, double> set)
        {
            p.Children.Add(new TextBlock { Text = title, FontWeight = FontWeights.SemiBold, Margin = new Thickness(0, 10, 0, 0) });
            Note(p, help);
            var row = new DockPanel();
            var val = new TextBlock { Width = 90, TextAlignment = TextAlignment.Right, VerticalAlignment = VerticalAlignment.Center };
            DockPanel.SetDock(val, Dock.Right);
            var sl = new Slider { Minimum = min, Maximum = max, TickFrequency = step, IsSnapToTickEnabled = true, VerticalAlignment = VerticalAlignment.Center };
            row.Children.Add(val); row.Children.Add(sl);
            p.Children.Add(row);

            Action label = () => val.Text = sl.Value.ToString(fmt, CultureInfo.InvariantCulture) + " " + unit;
            sl.ValueChanged += (s, e) => { label(); if (!_loading) set(_s, sl.Value); };
            _refresh.Add(st =>
            {
                double v = get(st);
                sl.Value = Math.Max(min, Math.Min(max, v));
                label();
            });
        }

        private void AddText(StackPanel p, string title, string help, Func<Settings, string> get, Action<Settings, string> set)
        {
            p.Children.Add(new TextBlock { Text = title, FontWeight = FontWeights.SemiBold, Margin = new Thickness(0, 10, 0, 0) });
            if (help.Length > 0) Note(p, help);
            var tb = new TextBox { Margin = new Thickness(0, 2, 0, 0) };
            p.Children.Add(tb);
            tb.TextChanged += (s, e) => { if (!_loading) set(_s, tb.Text.Trim()); };
            _refresh.Add(st => tb.Text = get(st));
        }

        private CheckBox AddCheck(StackPanel p, string title, string help, Func<Settings, bool> get, Action<Settings, bool> set)
        {
            var cb = new CheckBox { Content = title, FontWeight = FontWeights.SemiBold, Margin = new Thickness(0, 10, 0, 0) };
            p.Children.Add(cb);
            Note(p, help);
            cb.Checked += (s, e) => { if (!_loading) set(_s, true); };
            cb.Unchecked += (s, e) => { if (!_loading) set(_s, false); };
            _refresh.Add(st => cb.IsChecked = get(st));
            return cb;
        }
    }
}
