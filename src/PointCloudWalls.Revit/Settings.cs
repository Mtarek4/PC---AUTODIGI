using System;
using System.Globalization;
using System.IO;
using PointCloudWalls.Core;

namespace PointCloudWalls.Revit
{
    /// <summary>
    /// All tunables. Stored in %APPDATA%\PointCloudWalls\settings.ini and edited from the "PC Settings" window
    /// (the file can also be edited by hand). Re-read before EVERY pick. All lengths in meters.
    /// </summary>
    internal sealed class Settings
    {
        // --- how the scan is read
        public double SlabHalfM = 0.10;                    // slice = cut height +/- this
        public double SliceHeightAboveLevelM = double.NaN; // NaN => use the view's cut plane
        public double SpacingM = 0.01;                     // requested point spacing from the cloud
        public int MaxPoints = 300000;
        // --- SHARED across every category: how rough the scan is, and how sloppy a click may be
        public double InlierTolM = 0.010;                  // scan roughness
        public double SeedBandM = 0.08;                    // how far a click may be from the real surface
        // --- wall-only from here
        public double MinThicknessM = 0.05;
        public double MaxThicknessM = 1.00;
        public double MaxGapM = 0.30;                      // gaps up to this are bridged, longer ones end the wall
        public double SearchRadiusM = 3.0;                 // first look-around radius
        // --- accuracy
        public bool RefineAngle = true;
        public bool FollowWall = true;
        public double MaxWallLengthM = 40.0;
        public bool DetectEnds = true;
        public bool JunctionToCenterline = false;
        public double StripMarginM = 0.20;
        // --- centerline picking mode
        public double CenterlineTolM = 0.25;               // how far the clicked centre line may be from the wall's true centre
        public double EndSnapM = 0.40;                     // wall ends snap to scan features within this distance of your click
        public int EndModeIndex = 1;                       // 0 = snap to the scan near my clicks, 1 = find the real ends automatically, 2 = exactly at my clicks
        public bool AllowOneFace = true;                   // accept walls where only one face was scanned
        public double DiagonalTolM = 0.10;                 // diagonal mode: how close each click must be to a wall surface
        public double AssumedThicknessM = 0.20;            // thickness used for one-face walls
        public double ExtendStepM = 3.0;
        // --- automatic walls (Wall Auto): the whole view at once, no clicks. Nobody checks each wall as it is made, so these are
        //     stricter than the click tools' own limits
        public double AutoMinLengthM = 0.40;               // shorter pieces are left for the click tools
        public double AutoMinThicknessM = 0.07;            // two surfaces closer than this are a doubled surface, not a wall
        public double AutoMaxThicknessM = 0.50;            // two surfaces further apart than this are two walls / a corridor
        public double AutoBridgeOpeningM = 1.20;           // a wall carries on across a door or window opening up to this wide
        public bool AutoOneFace = true;                    // also model walls scanned from one side only (thickness = AssumedThicknessM)
        public double AutoOneFaceMinLengthM = 1.0;
        public bool AutoColumns = true;                    // also find rectangular columns / piers (built with the Column Setup choices)
        public double AutoColMinSizeM = 0.20;              // smaller bumps on a wall (window mullions) are not columns
        // --- what is created
        public double WallHeightM = 3.0;                   // fixed wall height (default in the wall setup popup)
        // --- wall setup popup (remembered choices)
        public bool AskWallSetup = true;                   // show the popup every time a wall tool starts
        public string SetupLevelName = "";
        public string SetupTopLevelName = "";
        public string SetupTypeName = "";                  // the wall type new types are duplicated from
        public int SetupHeightMode = 0;                    // 0 = fixed height, 1 = up to a level
        public double SetupBaseOffsetM = 0, SetupTopOffsetM = 0;

        // --- column detection (scan roughness, click forgiveness and "reuse existing types within" are SHARED - see above;
        //     these three are the ones that only make sense per-category, since a sensible column search radius is a very
        //     different number from a sensible wall one, and "how round is round enough" has no wall equivalent at all)
        public double ColSearchRadiusM = 1.0;       // center-click mode: how far around the click to look
        public double ColMinSizeM = 0.10, ColMaxSizeM = 1.50;
        public double ColRoundCoverage = 0.65;              // how much of a round column's circumference must be seen to call it round
        public int ColShapeMode = 0;                        // 0 = detect either shape, 1 = rectangular only, 2 = round only
        public bool ColAutoHeight = true;                   // search nearby slice heights for the one with the most data, instead of only the one fixed height
        public double ColHeightSearchM = 0.6;               // how far above/below the usual slice height to search
        public double ColHeightStepM = 0.10;                // step between candidate heights while searching

        // --- door/window detection (scan roughness and click forgiveness are the SHARED ones above)
        public bool AskDoorWindowSetup = false;
        public string DoorTypeName = "", WindowTypeName = "";
        public string DoorWidthParamName = "", DoorHeightParamName = "";       // empty = auto-detect (Width/W, Height/H)
        public string WindowWidthParamName = "", WindowHeightParamName = "";
        public double OpenSearchHalfWidthM = 1.8;
        public double OpenMinWidthM = 0.4, OpenMaxWidthM = 3.0;
        public double OpenMinHeightM = 0.3, OpenMaxHeightM = 3.0;
        public double OpenFloorTolM = 0.10;                 // an opening bottom within this of the wall base counts as reaching the floor (door)
        public double OpenMinJambCoverage = 0.45;
        public double OpenBridgeMaxWidthM = 0.75;           // obstructions this wide or narrower (e.g. a person standing in the doorway) are bridged over if they don't reach the top of the search area
        // --- column setup popup (remembered choices; separate from the wall ones)
        public bool AskColumnSetup = true;
        public string ColLevelName = "", ColTopLevelName = "", ColTypeName = "";
        public string ColAltTypeName = "";               // remembered auto-switch target (Center mode) for whichever shape ColTypeName does NOT cover
        public int ColHeightMode = 0;
        public double ColHeightM = 3.0;
        public double ColBaseOffsetM = 0, ColTopOffsetM = 0;
        // --- SHARED across every category (walls, columns, and later beams etc.) --------------------------------------
        public double TypeSnapM = 0.010;                   // reuse an existing type/family within this size difference
        public bool DebugLines = true;                     // fitted faces (red), centerline (green), wall ends (blue)
        public bool QualityNote = true;                    // write the result into the wall's Comments
        // --- publish to CAD
        public int CadUnitsIndex = 0;                      // drawing units: 0 = mm, 1 = cm, 2 = m
        public int CadCoordinatesIndex = 0;                // 0 = Revit internal origin, 1 = shared coordinates
        public string CadOutlineLayer = "PCW-WALL";
        public string CadCenterLayer = "PCW-WALL-CENTER";
        public bool CadCenterline = true;                  // also draw each wall's location line
        public bool CadIncludeColumns = true;               // also publish structural columns
        public string CadColumnLayer = "PCW-COLUMN";
        public bool CadMergeWalls = true;                  // walls that touch or overlap are drawn as ONE clean outline (like Revit's joined walls)
        public bool CadRemoveDeleted = true;               // walls deleted in Revit are removed from CAD on the next full publish
        public bool CadZoom = true;                        // zoom to extents after the first publish
        public string CadPipeName = "";                    // empty = default (per Windows user)
        public bool AutoJoin = true;                       // trim ends to the walls they meet + apply Revit joins
        public double JoinReachM = 0.15;                   // how far a wall end may be from another wall and still be joined

        public static string FilePath => Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "PointCloudWalls", "settings.ini");

        public Settings Clone() => (Settings)MemberwiseClone();

        public ColumnFitOptions ToColumnOptions() => new ColumnFitOptions
        {
            SearchRadius = ColSearchRadiusM, InlierTol = InlierTolM, SeedBand = SeedBandM,
            MinSize = ColMinSizeM, MaxSize = ColMaxSizeM, RoundCoverage = ColRoundCoverage,
            AllowRectangular = ColShapeMode != 2, AllowRound = ColShapeMode != 1,
        };

        public WallFitOptions ToOptions() => new WallFitOptions
        {
            InlierTol = InlierTolM, SeedBand = SeedBandM, MaxGap = MaxGapM,
            MinThickness = MinThicknessM, MaxThickness = MaxThicknessM, SearchRadius = SearchRadiusM,
            RefineAngle = RefineAngle, FollowWall = FollowWall, MaxWallLength = MaxWallLengthM,
            DetectEnds = DetectEnds, JunctionToCenterline = JunctionToCenterline,
            StripMargin = StripMarginM, ExtendStep = ExtendStepM,
            CenterlineClickTol = CenterlineTolM, EndSnap = EndSnapM, Ends = (EndMode)Math.Max(0, Math.Min(2, EndModeIndex)), AllowOneFace = AllowOneFace, AssumedThickness = AssumedThicknessM, DiagonalClickTol = DiagonalTolM,
        };

        public WallDetectOptions ToDetectOptions() => new WallDetectOptions
        {
            MinThickness = AutoMinThicknessM, MaxThickness = Math.Min(AutoMaxThicknessM, MaxThicknessM), MinLength = AutoMinLengthM,
            InlierTol = InlierTolM, MaxGap = MaxGapM, BridgeOpening = AutoBridgeOpeningM,
            OneFaceWalls = AutoOneFace, OneFaceThickness = AssumedThicknessM, OneFaceMinLength = AutoOneFaceMinLengthM,
            Columns = AutoColumns, ColumnMinSize = AutoColMinSizeM, ColumnMaxSize = ColMaxSizeM,
        };

        // ---- presets -----------------------------------------------------------------------------
        public static readonly string[] PresetNames =
        {
            "Standard (recommended)",
            "Clean scan - maximum precision",
            "Noisy scan / rough surfaces",
        };

        public void ApplyPreset(int index)
        {
            var d = new Settings();
            // keep the user's output choices, reset the detection ones
            d.WallHeightM = WallHeightM; d.TypeSnapM = TypeSnapM; d.DebugLines = DebugLines; d.QualityNote = QualityNote; d.AutoJoin = AutoJoin; d.JoinReachM = JoinReachM; d.AssumedThicknessM = AssumedThicknessM; d.AllowOneFace = AllowOneFace; d.EndModeIndex = EndModeIndex; d.DiagonalTolM = DiagonalTolM;
            d.ColSearchRadiusM = ColSearchRadiusM; d.ColMinSizeM = ColMinSizeM; d.ColMaxSizeM = ColMaxSizeM;
            d.ColRoundCoverage = ColRoundCoverage; d.ColShapeMode = ColShapeMode; d.ColAutoHeight = ColAutoHeight; d.ColHeightSearchM = ColHeightSearchM; d.ColHeightStepM = ColHeightStepM;
            d.AskColumnSetup = AskColumnSetup; d.ColLevelName = ColLevelName; d.ColTopLevelName = ColTopLevelName; d.ColTypeName = ColTypeName;
            d.ColHeightMode = ColHeightMode; d.ColHeightM = ColHeightM; d.ColBaseOffsetM = ColBaseOffsetM; d.ColTopOffsetM = ColTopOffsetM; d.ColAltTypeName = ColAltTypeName;
            d.AskWallSetup = AskWallSetup; d.SetupLevelName = SetupLevelName; d.SetupTopLevelName = SetupTopLevelName; d.SetupTypeName = SetupTypeName;
            d.SetupHeightMode = SetupHeightMode; d.SetupBaseOffsetM = SetupBaseOffsetM; d.SetupTopOffsetM = SetupTopOffsetM;
            d.CadUnitsIndex = CadUnitsIndex; d.CadCoordinatesIndex = CadCoordinatesIndex; d.CadOutlineLayer = CadOutlineLayer; d.CadCenterLayer = CadCenterLayer;
            d.CadCenterline = CadCenterline; d.CadMergeWalls = CadMergeWalls; d.CadIncludeColumns = CadIncludeColumns; d.CadColumnLayer = CadColumnLayer; d.CadRemoveDeleted = CadRemoveDeleted; d.CadZoom = CadZoom; d.CadPipeName = CadPipeName;
            d.JunctionToCenterline = JunctionToCenterline;
            d.AskDoorWindowSetup = AskDoorWindowSetup; d.DoorTypeName = DoorTypeName; d.WindowTypeName = WindowTypeName;
            d.DoorWidthParamName = DoorWidthParamName; d.DoorHeightParamName = DoorHeightParamName;
            d.WindowWidthParamName = WindowWidthParamName; d.WindowHeightParamName = WindowHeightParamName;
            d.OpenSearchHalfWidthM = OpenSearchHalfWidthM; d.OpenMinWidthM = OpenMinWidthM; d.OpenMaxWidthM = OpenMaxWidthM;
            d.OpenMinHeightM = OpenMinHeightM; d.OpenMaxHeightM = OpenMaxHeightM; d.OpenFloorTolM = OpenFloorTolM; d.OpenMinJambCoverage = OpenMinJambCoverage;
            d.OpenBridgeMaxWidthM = OpenBridgeMaxWidthM;
            d.AutoMinLengthM = AutoMinLengthM; d.AutoMinThicknessM = AutoMinThicknessM; d.AutoMaxThicknessM = AutoMaxThicknessM;
            d.AutoBridgeOpeningM = AutoBridgeOpeningM; d.AutoOneFace = AutoOneFace; d.AutoOneFaceMinLengthM = AutoOneFaceMinLengthM;
            d.AutoColumns = AutoColumns; d.AutoColMinSizeM = AutoColMinSizeM;
            if (index == 1) { d.InlierTolM = 0.005; d.SeedBandM = 0.05; d.SlabHalfM = 0.05; d.SpacingM = 0.005; d.MaxGapM = 0.25; }
            if (index == 2) { d.InlierTolM = 0.020; d.SeedBandM = 0.10; d.SlabHalfM = 0.15; d.SpacingM = 0.02; d.MaxGapM = 0.50; }
            CopyFrom(d);
        }

        public void CopyFrom(Settings o)
        {
            SlabHalfM = o.SlabHalfM; SliceHeightAboveLevelM = o.SliceHeightAboveLevelM; SpacingM = o.SpacingM; MaxPoints = o.MaxPoints;
            InlierTolM = o.InlierTolM; SeedBandM = o.SeedBandM; MinThicknessM = o.MinThicknessM; MaxThicknessM = o.MaxThicknessM;
            MaxGapM = o.MaxGapM; SearchRadiusM = o.SearchRadiusM; RefineAngle = o.RefineAngle; FollowWall = o.FollowWall;
            MaxWallLengthM = o.MaxWallLengthM; DetectEnds = o.DetectEnds; JunctionToCenterline = o.JunctionToCenterline;
            StripMarginM = o.StripMarginM; ExtendStepM = o.ExtendStepM; WallHeightM = o.WallHeightM; TypeSnapM = o.TypeSnapM;
            DebugLines = o.DebugLines; QualityNote = o.QualityNote;
            ColSearchRadiusM = o.ColSearchRadiusM; ColMinSizeM = o.ColMinSizeM; ColMaxSizeM = o.ColMaxSizeM;
            ColRoundCoverage = o.ColRoundCoverage; ColShapeMode = o.ColShapeMode; ColAutoHeight = o.ColAutoHeight; ColHeightSearchM = o.ColHeightSearchM; ColHeightStepM = o.ColHeightStepM;
            AskDoorWindowSetup = o.AskDoorWindowSetup; DoorTypeName = o.DoorTypeName; WindowTypeName = o.WindowTypeName;
            DoorWidthParamName = o.DoorWidthParamName; DoorHeightParamName = o.DoorHeightParamName;
            WindowWidthParamName = o.WindowWidthParamName; WindowHeightParamName = o.WindowHeightParamName;
            OpenSearchHalfWidthM = o.OpenSearchHalfWidthM; OpenMinWidthM = o.OpenMinWidthM; OpenMaxWidthM = o.OpenMaxWidthM;
            OpenMinHeightM = o.OpenMinHeightM; OpenMaxHeightM = o.OpenMaxHeightM; OpenFloorTolM = o.OpenFloorTolM; OpenMinJambCoverage = o.OpenMinJambCoverage;
            OpenBridgeMaxWidthM = o.OpenBridgeMaxWidthM;
            AskColumnSetup = o.AskColumnSetup; ColLevelName = o.ColLevelName; ColTopLevelName = o.ColTopLevelName; ColTypeName = o.ColTypeName;
            ColHeightMode = o.ColHeightMode; ColHeightM = o.ColHeightM; ColBaseOffsetM = o.ColBaseOffsetM; ColTopOffsetM = o.ColTopOffsetM; ColAltTypeName = o.ColAltTypeName;
            AskWallSetup = o.AskWallSetup; SetupLevelName = o.SetupLevelName; SetupTopLevelName = o.SetupTopLevelName; SetupTypeName = o.SetupTypeName;
            SetupHeightMode = o.SetupHeightMode; SetupBaseOffsetM = o.SetupBaseOffsetM; SetupTopOffsetM = o.SetupTopOffsetM;
            CadUnitsIndex = o.CadUnitsIndex; CadCoordinatesIndex = o.CadCoordinatesIndex; CadOutlineLayer = o.CadOutlineLayer; CadCenterLayer = o.CadCenterLayer;
            CadCenterline = o.CadCenterline; CadMergeWalls = o.CadMergeWalls; CadIncludeColumns = o.CadIncludeColumns; CadColumnLayer = o.CadColumnLayer; CadRemoveDeleted = o.CadRemoveDeleted; CadZoom = o.CadZoom; CadPipeName = o.CadPipeName; AutoJoin = o.AutoJoin; JoinReachM = o.JoinReachM;
            CenterlineTolM = o.CenterlineTolM; EndSnapM = o.EndSnapM; EndModeIndex = o.EndModeIndex; AllowOneFace = o.AllowOneFace; AssumedThicknessM = o.AssumedThicknessM; DiagonalTolM = o.DiagonalTolM;
            AutoMinLengthM = o.AutoMinLengthM; AutoMinThicknessM = o.AutoMinThicknessM; AutoMaxThicknessM = o.AutoMaxThicknessM;
            AutoBridgeOpeningM = o.AutoBridgeOpeningM; AutoOneFace = o.AutoOneFace; AutoOneFaceMinLengthM = o.AutoOneFaceMinLengthM;
            AutoColumns = o.AutoColumns; AutoColMinSizeM = o.AutoColMinSizeM;
        }

        // ---- persistence ----------------------------------------------------------------------------
        static string F(double d) => double.IsNaN(d) ? "" : d.ToString("0.####", CultureInfo.InvariantCulture);
        static string B(bool b) => b ? "1" : "0";

        string ToText() => string.Join(Environment.NewLine, new[]
        {
            "# PointCloudWalls settings (meters). Use the 'PC Settings' window, or edit + save; applies on the next click.",
            "SlabHalfM=" + F(SlabHalfM),
            "SliceHeightAboveLevelM=" + F(SliceHeightAboveLevelM) + "   # empty = use the view's cut plane",
            "SpacingM=" + F(SpacingM),
            "MaxPoints=" + MaxPoints,
            "InlierTolM=" + F(InlierTolM),
            "SeedBandM=" + F(SeedBandM),
            "MinThicknessM=" + F(MinThicknessM),
            "MaxThicknessM=" + F(MaxThicknessM),
            "MaxGapM=" + F(MaxGapM),
            "SearchRadiusM=" + F(SearchRadiusM),
            "RefineAngle=" + B(RefineAngle),
            "FollowWall=" + B(FollowWall),
            "MaxWallLengthM=" + F(MaxWallLengthM),
            "DetectEnds=" + B(DetectEnds),
            "JunctionToCenterline=" + B(JunctionToCenterline),
            "StripMarginM=" + F(StripMarginM),
            "ExtendStepM=" + F(ExtendStepM),
            "WallHeightM=" + F(WallHeightM),
            "AskWallSetup=" + B(AskWallSetup),
            "SetupLevelName=" + Uri.EscapeDataString(SetupLevelName),
            "SetupTopLevelName=" + Uri.EscapeDataString(SetupTopLevelName),
            "SetupTypeName=" + Uri.EscapeDataString(SetupTypeName),
            "SetupHeightMode=" + SetupHeightMode,
            "SetupBaseOffsetM=" + F(SetupBaseOffsetM),
            "SetupTopOffsetM=" + F(SetupTopOffsetM),
            "ColSearchRadiusM=" + F(ColSearchRadiusM),
            "ColMinSizeM=" + F(ColMinSizeM),
            "ColMaxSizeM=" + F(ColMaxSizeM),
            "ColRoundCoverage=" + F(ColRoundCoverage),
            "ColShapeMode=" + ColShapeMode,
            "ColAutoHeight=" + B(ColAutoHeight),
            "ColHeightSearchM=" + F(ColHeightSearchM),
            "ColHeightStepM=" + F(ColHeightStepM),
            "AskDoorWindowSetup=" + B(AskDoorWindowSetup),
            "DoorTypeName=" + Uri.EscapeDataString(DoorTypeName),
            "WindowTypeName=" + Uri.EscapeDataString(WindowTypeName),
            "DoorWidthParamName=" + Uri.EscapeDataString(DoorWidthParamName),
            "DoorHeightParamName=" + Uri.EscapeDataString(DoorHeightParamName),
            "WindowWidthParamName=" + Uri.EscapeDataString(WindowWidthParamName),
            "WindowHeightParamName=" + Uri.EscapeDataString(WindowHeightParamName),
            "OpenSearchHalfWidthM=" + F(OpenSearchHalfWidthM),
            "OpenMinWidthM=" + F(OpenMinWidthM), "OpenMaxWidthM=" + F(OpenMaxWidthM),
            "OpenMinHeightM=" + F(OpenMinHeightM), "OpenMaxHeightM=" + F(OpenMaxHeightM),
            "OpenFloorTolM=" + F(OpenFloorTolM), "OpenMinJambCoverage=" + F(OpenMinJambCoverage), "OpenBridgeMaxWidthM=" + F(OpenBridgeMaxWidthM),
            "AskColumnSetup=" + B(AskColumnSetup),
            "ColLevelName=" + Uri.EscapeDataString(ColLevelName),
            "ColTopLevelName=" + Uri.EscapeDataString(ColTopLevelName),
            "ColTypeName=" + Uri.EscapeDataString(ColTypeName),
            "ColHeightMode=" + ColHeightMode,
            "ColHeightM=" + F(ColHeightM),
            "ColBaseOffsetM=" + F(ColBaseOffsetM),
            "ColTopOffsetM=" + F(ColTopOffsetM),
            "ColAltTypeName=" + Uri.EscapeDataString(ColAltTypeName),
            "TypeSnapM=" + F(TypeSnapM),
            "DebugLines=" + B(DebugLines),
            "QualityNote=" + B(QualityNote),
            "CadUnits=" + CadUnitsIndex,
            "CadCoordinates=" + CadCoordinatesIndex,
            "CadOutlineLayer=" + CadOutlineLayer,
            "CadCenterLayer=" + CadCenterLayer,
            "CadCenterline=" + B(CadCenterline),
            "CadMergeWalls=" + B(CadMergeWalls),
            "CadIncludeColumns=" + B(CadIncludeColumns),
            "CadColumnLayer=" + CadColumnLayer,
            "CadRemoveDeleted=" + B(CadRemoveDeleted),
            "CadZoom=" + B(CadZoom),
            "CadPipeName=" + CadPipeName + "   # empty = default",
            "CenterlineTolM=" + F(CenterlineTolM),
            "EndSnapM=" + F(EndSnapM),
            "EndMode=" + EndModeIndex,
            "AllowOneFace=" + B(AllowOneFace),
            "DiagonalTolM=" + F(DiagonalTolM),
            "AssumedThicknessM=" + F(AssumedThicknessM),
            "AutoJoin=" + B(AutoJoin),
            "JoinReachM=" + F(JoinReachM),
            "AutoMinLengthM=" + F(AutoMinLengthM),
            "AutoMinThicknessM=" + F(AutoMinThicknessM),
            "AutoMaxThicknessM=" + F(AutoMaxThicknessM),
            "AutoBridgeOpeningM=" + F(AutoBridgeOpeningM),
            "AutoOneFace=" + B(AutoOneFace),
            "AutoOneFaceMinLengthM=" + F(AutoOneFaceMinLengthM),
            "AutoColumns=" + B(AutoColumns),
            "AutoColMinSizeM=" + F(AutoColMinSizeM),
        });

        public void Save()
        {
            Directory.CreateDirectory(Path.GetDirectoryName(FilePath));
            File.WriteAllText(FilePath, ToText());
        }

        public static Settings Load()
        {
            var s = new Settings();
            try
            {
                if (!File.Exists(FilePath)) { s.Save(); return s; }
                foreach (var raw in File.ReadAllLines(FilePath))
                {
                    var line = raw;
                    int hash = line.IndexOf('#');
                    if (hash >= 0) line = line.Substring(0, hash);
                    line = line.Trim();
                    int eq = line.IndexOf('=');
                    if (eq < 0) continue;
                    string k = line.Substring(0, eq).Trim(), v = line.Substring(eq + 1).Trim();
                    double d; bool isD = double.TryParse(v, NumberStyles.Float, CultureInfo.InvariantCulture, out d);
                    bool b = v == "1" || v.Equals("true", StringComparison.OrdinalIgnoreCase);
                    switch (k)
                    {
                        case "SlabHalfM": if (isD) s.SlabHalfM = d; break;
                        case "SliceHeightAboveLevelM": s.SliceHeightAboveLevelM = isD ? d : double.NaN; break;
                        case "SpacingM": if (isD) s.SpacingM = d; break;
                        case "MaxPoints": if (isD) s.MaxPoints = (int)Math.Max(1000, Math.Min(900000, d)); break;
                        case "InlierTolM": if (isD) s.InlierTolM = d; break;
                        case "SeedBandM": if (isD) s.SeedBandM = d; break;
                        case "MinThicknessM": if (isD) s.MinThicknessM = d; break;
                        case "MaxThicknessM": if (isD) s.MaxThicknessM = d; break;
                        case "MaxGapM": if (isD) s.MaxGapM = d; break;
                        case "SearchRadiusM": if (isD) s.SearchRadiusM = d; break;
                        case "RefineAngle": s.RefineAngle = b; break;
                        case "FollowWall": s.FollowWall = b; break;
                        case "MaxWallLengthM": if (isD) s.MaxWallLengthM = d; break;
                        case "DetectEnds": s.DetectEnds = b; break;
                        case "JunctionToCenterline": s.JunctionToCenterline = b; break;
                        case "StripMarginM": if (isD) s.StripMarginM = d; break;
                        case "ExtendStepM": if (isD) s.ExtendStepM = d; break;
                        case "WallHeightM": if (isD) s.WallHeightM = d; break;
                        case "AskWallSetup": s.AskWallSetup = b; break;
                        case "SetupLevelName": s.SetupLevelName = Uri.UnescapeDataString(v); break;
                        case "SetupTopLevelName": s.SetupTopLevelName = Uri.UnescapeDataString(v); break;
                        case "SetupTypeName": s.SetupTypeName = Uri.UnescapeDataString(v); break;
                        case "SetupHeightMode": if (isD) s.SetupHeightMode = (int)d; break;
                        case "SetupBaseOffsetM": if (isD) s.SetupBaseOffsetM = d; break;
                        case "SetupTopOffsetM": if (isD) s.SetupTopOffsetM = d; break;
                        case "ColSearchRadiusM": if (isD) s.ColSearchRadiusM = d; break;
                        case "ColMinSizeM": if (isD) s.ColMinSizeM = d; break;
                        case "ColMaxSizeM": if (isD) s.ColMaxSizeM = d; break;
                        case "ColRoundCoverage": if (isD) s.ColRoundCoverage = Math.Max(0.2, Math.Min(0.95, d)); break;
                        case "ColShapeMode": if (isD) s.ColShapeMode = Math.Max(0, Math.Min(2, (int)d)); break;
                        case "ColAutoHeight": s.ColAutoHeight = b; break;
                        case "ColHeightSearchM": if (isD) s.ColHeightSearchM = d; break;
                        case "ColHeightStepM": if (isD) s.ColHeightStepM = d; break;
                        case "AskDoorWindowSetup": s.AskDoorWindowSetup = b; break;
                        case "DoorTypeName": s.DoorTypeName = Uri.UnescapeDataString(v); break;
                        case "WindowTypeName": s.WindowTypeName = Uri.UnescapeDataString(v); break;
                        case "DoorWidthParamName": s.DoorWidthParamName = Uri.UnescapeDataString(v); break;
                        case "DoorHeightParamName": s.DoorHeightParamName = Uri.UnescapeDataString(v); break;
                        case "WindowWidthParamName": s.WindowWidthParamName = Uri.UnescapeDataString(v); break;
                        case "WindowHeightParamName": s.WindowHeightParamName = Uri.UnescapeDataString(v); break;
                        case "OpenSearchHalfWidthM": if (isD) s.OpenSearchHalfWidthM = d; break;
                        case "OpenMinWidthM": if (isD) s.OpenMinWidthM = d; break;
                        case "OpenMaxWidthM": if (isD) s.OpenMaxWidthM = d; break;
                        case "OpenMinHeightM": if (isD) s.OpenMinHeightM = d; break;
                        case "OpenMaxHeightM": if (isD) s.OpenMaxHeightM = d; break;
                        case "OpenFloorTolM": if (isD) s.OpenFloorTolM = d; break;
                        case "OpenMinJambCoverage": if (isD) s.OpenMinJambCoverage = d; break;
                        case "OpenBridgeMaxWidthM": if (isD) s.OpenBridgeMaxWidthM = d; break;
                        case "AskColumnSetup": s.AskColumnSetup = b; break;
                        case "ColLevelName": s.ColLevelName = Uri.UnescapeDataString(v); break;
                        case "ColTopLevelName": s.ColTopLevelName = Uri.UnescapeDataString(v); break;
                        case "ColTypeName": s.ColTypeName = Uri.UnescapeDataString(v); break;
                        case "ColHeightMode": if (isD) s.ColHeightMode = (int)d; break;
                        case "ColHeightM": if (isD) s.ColHeightM = d; break;
                        case "ColBaseOffsetM": if (isD) s.ColBaseOffsetM = d; break;
                        case "ColTopOffsetM": if (isD) s.ColTopOffsetM = d; break;
                        case "ColAltTypeName": s.ColAltTypeName = Uri.UnescapeDataString(v); break;
                        case "TypeSnapM": if (isD) s.TypeSnapM = d; break;
                        case "DebugLines": s.DebugLines = b; break;
                        case "QualityNote": s.QualityNote = b; break;
                        case "CadUnits": if (isD) s.CadUnitsIndex = (int)d; break;
                        case "CadCoordinates": if (isD) s.CadCoordinatesIndex = (int)d; break;
                        case "CadOutlineLayer": if (v.Length > 0) s.CadOutlineLayer = v; break;
                        case "CadCenterLayer": if (v.Length > 0) s.CadCenterLayer = v; break;
                        case "CadCenterline": s.CadCenterline = b; break;
                        case "CadMergeWalls": s.CadMergeWalls = b; break;
                        case "CadIncludeColumns": s.CadIncludeColumns = b; break;
                        case "CadColumnLayer": if (v.Length > 0) s.CadColumnLayer = v; break;
                        case "CadRemoveDeleted": s.CadRemoveDeleted = b; break;
                        case "CadZoom": s.CadZoom = b; break;
                        case "CadPipeName": s.CadPipeName = v; break;
                        case "CenterlineTolM": if (isD) s.CenterlineTolM = d; break;
                        case "EndSnapM": if (isD) s.EndSnapM = d; break;
                        case "EndMode": if (isD) s.EndModeIndex = (int)d; break;
                        case "AllowOneFace": s.AllowOneFace = b; break;
                        case "DiagonalTolM": if (isD) s.DiagonalTolM = d; break;
                        case "AssumedThicknessM": if (isD) s.AssumedThicknessM = d; break;
                        case "AutoJoin": s.AutoJoin = b; break;
                        case "JoinReachM": if (isD) s.JoinReachM = d; break;
                        case "AutoMinLengthM": if (isD) s.AutoMinLengthM = d; break;
                        case "AutoMinThicknessM": if (isD) s.AutoMinThicknessM = d; break;
                        case "AutoMaxThicknessM": if (isD) s.AutoMaxThicknessM = d; break;
                        case "AutoBridgeOpeningM": if (isD) s.AutoBridgeOpeningM = d; break;
                        case "AutoOneFace": s.AutoOneFace = b; break;
                        case "AutoOneFaceMinLengthM": if (isD) s.AutoOneFaceMinLengthM = d; break;
                        case "AutoColumns": s.AutoColumns = b; break;
                        case "AutoColMinSizeM": if (isD) s.AutoColMinSizeM = d; break;
                    }
                }
            }
            catch { /* fall back to defaults */ }
            return s;
        }
    }
}
