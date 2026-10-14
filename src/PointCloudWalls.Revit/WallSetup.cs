using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Interop;
using System.Windows.Media;
using Autodesk.Revit.DB;

namespace PointCloudWalls.Revit
{
    /// <summary>What the next walls are built with: base level, height, and the wall type new types are duplicated from.</summary>
    internal enum SetupKind { Wall, Column }

    internal sealed class WallSetup
    {
        public SetupKind Kind = SetupKind.Wall;
        public ElementId BaseLevelId, TopLevelId, TemplateTypeId;
        public double BaseOffsetFt, HeightFt, TopOffsetFt;
        public int HeightMode;                            // 0 = fixed (unconnected) height, 1 = up to a level
        public string BaseLevelName = "", TopLevelName = "", TemplateName = "";

        public static List<Level> Levels(Document doc) =>
            new FilteredElementCollector(doc).OfClass(typeof(Level)).Cast<Level>().OrderBy(l => l.Elevation).ToList();

        public static List<WallType> BasicTypes(Document doc) =>
            new FilteredElementCollector(doc).OfClass(typeof(WallType)).Cast<WallType>()
                .Where(t => t.Kind == WallKind.Basic).OrderBy(t => t.Name, StringComparer.CurrentCultureIgnoreCase).ToList();

        public static List<FamilySymbol> ColumnTypes(Document doc) => ColumnBuilder.ColumnTypes(doc);

        /// <summary>Saved choices (by name, so they survive between projects). preferView: default the base level to the level of the active view.</summary>
        public static WallSetup FromSettings(Document doc, ViewPlan view, Settings s, bool preferView, SetupKind kind = SetupKind.Wall)
        {
            List<Level> levels = Levels(doc);
            string levelName = kind == SetupKind.Wall ? s.SetupLevelName : s.ColLevelName;
            string topName = kind == SetupKind.Wall ? s.SetupTopLevelName : s.ColTopLevelName;
            double heightM = kind == SetupKind.Wall ? s.WallHeightM : s.ColHeightM;
            double baseOffM = kind == SetupKind.Wall ? s.SetupBaseOffsetM : s.ColBaseOffsetM;
            double topOffM = kind == SetupKind.Wall ? s.SetupTopOffsetM : s.ColTopOffsetM;
            int heightMode = kind == SetupKind.Wall ? s.SetupHeightMode : s.ColHeightMode;

            Level saved = levels.FirstOrDefault(l => l.Name == levelName);
            Level baseLvl = preferView ? (view != null && view.GenLevel != null ? view.GenLevel : saved) : (saved ?? (view != null ? view.GenLevel : null));
            baseLvl = baseLvl ?? levels.FirstOrDefault();
            Level top = levels.FirstOrDefault(l => l.Name == topName && baseLvl != null && l.Elevation > baseLvl.Elevation)
                        ?? (baseLvl == null ? null : levels.FirstOrDefault(l => l.Elevation > baseLvl.Elevation));

            ElementId templateId = null; string templateName = "";
            if (kind == SetupKind.Wall)
            {
                List<WallType> types = BasicTypes(doc);
                WallType type = types.FirstOrDefault(t => t.Name == s.SetupTypeName)
                                ?? types.FirstOrDefault(t => t.Name.IndexOf("Generic", StringComparison.OrdinalIgnoreCase) >= 0) ?? types.FirstOrDefault();
                templateId = type?.Id; templateName = type?.Name ?? "";
            }
            else
            {
                List<FamilySymbol> types = ColumnTypes(doc);
                FamilySymbol type = types.FirstOrDefault(t => t.Family.Name + " : " + t.Name == s.ColTypeName)
                                    ?? types.FirstOrDefault();
                templateId = type?.Id; templateName = type != null ? type.Family.Name + " : " + type.Name : "";
            }

            return new WallSetup
            {
                Kind = kind,
                BaseLevelId = baseLvl?.Id, BaseLevelName = baseLvl?.Name ?? "",
                TopLevelId = top?.Id, TopLevelName = top?.Name ?? "",
                TemplateTypeId = templateId, TemplateName = templateName,
                BaseOffsetFt = baseOffM / Ctx.MPerFt, TopOffsetFt = topOffM / Ctx.MPerFt,
                HeightFt = heightM / Ctx.MPerFt,
                HeightMode = top == null ? 0 : heightMode,
            };
        }

        public void CopyTo(Settings s)
        {
            if (Kind == SetupKind.Wall)
            {
                s.SetupLevelName = BaseLevelName; s.SetupTopLevelName = TopLevelName; s.SetupTypeName = TemplateName;
                s.SetupHeightMode = HeightMode; s.WallHeightM = HeightFt * Ctx.MPerFt;
                s.SetupBaseOffsetM = BaseOffsetFt * Ctx.MPerFt; s.SetupTopOffsetM = TopOffsetFt * Ctx.MPerFt;
            }
            else
            {
                s.ColLevelName = BaseLevelName; s.ColTopLevelName = TopLevelName; s.ColTypeName = TemplateName;
                s.ColHeightMode = HeightMode; s.ColHeightM = HeightFt * Ctx.MPerFt;
                s.ColBaseOffsetM = BaseOffsetFt * Ctx.MPerFt; s.ColTopOffsetM = TopOffsetFt * Ctx.MPerFt;
            }
        }
    }

    /// <summary>The popup: level, wall height and the wall type new types are duplicated from. Code-only WPF, in the project's own units.</summary>
    internal sealed class WallSetupWindow : Window
    {
        private readonly Document _doc;
        private readonly Units _units;
        private readonly List<Level> _levels;
        private readonly List<WallType> _types;
        private readonly List<FamilySymbol> _colTypes;
        private readonly SetupKind _kind;

        private readonly ComboBox _baseLevel = new ComboBox(), _topLevel = new ComboBox(), _type = new ComboBox();
        private readonly TextBox _baseOffset = new TextBox(), _height = new TextBox(), _topOffset = new TextBox();
        private readonly RadioButton _fixed = new RadioButton(), _toLevel = new RadioButton();
        private readonly CheckBox _ask = new CheckBox();
        private readonly TextBlock _error = new TextBlock();

        public WallSetup Result { get; private set; }
        public bool AskEachTime { get; private set; }

        public WallSetupWindow(Document doc, WallSetup current, bool askEachTime, bool fromTool)
        {
            _doc = doc;
            _kind = current.Kind;
            _units = doc.GetUnits();
            _levels = WallSetup.Levels(doc);
            _types = _kind == SetupKind.Wall ? WallSetup.BasicTypes(doc) : new List<WallType>();
            _colTypes = _kind == SetupKind.Column ? WallSetup.ColumnTypes(doc) : new List<FamilySymbol>();
            string noun = _kind == SetupKind.Wall ? "wall" : "column";

            Title = _kind == SetupKind.Wall ? "Wall setup" : "Column setup";
            Width = 480; SizeToContent = SizeToContent.Height; MinWidth = 420;
            WindowStartupLocation = WindowStartupLocation.CenterScreen;
            ResizeMode = ResizeMode.NoResize;
            try { new WindowInteropHelper(this).Owner = System.Diagnostics.Process.GetCurrentProcess().MainWindowHandle; } catch { }

            var root = new StackPanel { Margin = new Thickness(18, 14, 18, 14) };
            Content = root;

            root.Children.Add(new TextBlock { Text = fromTool ? "How should the new " + noun + "s be built?" : Title, FontSize = 16, FontWeight = FontWeights.Bold });
            Note(root, "These choices are used for every " + noun + " the XD " + noun + " tools create. Values are in the project's units (" + UnitName() + ").");

            // ---- level
            Heading(root, "Level");
            foreach (Level l in _levels) _baseLevel.Items.Add(l.Name + "    (" + Show(l.Elevation) + ")");
            root.Children.Add(_baseLevel);
            root.Children.Add(Label("Base offset from the level"));
            root.Children.Add(_baseOffset);

            // ---- height
            Heading(root, "Wall height");
            _fixed.Content = "Fixed height"; _toLevel.Content = "Up to a level (top constraint)";
            _fixed.GroupName = "hm"; _toLevel.GroupName = "hm";
            root.Children.Add(_fixed);
            var fixedRow = new StackPanel { Margin = new Thickness(22, 2, 0, 6) };
            fixedRow.Children.Add(_height);
            root.Children.Add(fixedRow);
            root.Children.Add(_toLevel);
            var levelRow = new StackPanel { Margin = new Thickness(22, 2, 0, 0) };
            foreach (Level l in _levels) _topLevel.Items.Add(l.Name + "    (" + Show(l.Elevation) + ")");
            levelRow.Children.Add(_topLevel);
            levelRow.Children.Add(Label("Top offset from that level"));
            levelRow.Children.Add(_topOffset);
            root.Children.Add(levelRow);
            _fixed.Checked += (s, e) => { _height.IsEnabled = true; _topLevel.IsEnabled = false; _topOffset.IsEnabled = false; };
            _toLevel.Checked += (s, e) => { _height.IsEnabled = false; _topLevel.IsEnabled = true; _topOffset.IsEnabled = true; };

            // ---- type to duplicate from
            Heading(root, (_kind == SetupKind.Wall ? "Wall" : "Column") + " type to duplicate from");
            if (_kind == SetupKind.Wall)
                foreach (WallType t in _types) _type.Items.Add(t.Name + "    [" + Show(t.Width) + "]");
            else
                foreach (FamilySymbol t in _colTypes)
                {
                    string dim = ColumnBuilder.IsRound(t) ? "round" : ColumnBuilder.IsRectangular(t) ? "rectangular" : "other";
                    _type.Items.Add(t.Family.Name + " : " + t.Name + "    [" + dim + "]");
                }
            root.Children.Add(_type);
            Note(root, _kind == SetupKind.Wall
                ? "Each wall gets the thickness measured from the scan: the selected type is duplicated with that thickness (a new type is made only when no earlier copy is close enough)."
                : "Only STRUCTURAL column families are listed. Each column gets the size measured from the scan: the selected type is duplicated with that size (a new type is made only when no earlier copy is close enough). " +
                  "Rectangular columns need a family with Width/Depth (or b/h) parameters; round columns need one with a Diameter (or D/d) parameter - Revit's standard structural column families (e.g. Concrete-Rectangular-Column, Concrete-Round-Column) work. " +
                  "Column Center: if the scan turns out the OTHER shape from the family chosen here, and another matching family is loaded, that one is used automatically instead.");

            _ask.Content = "Ask me every time I start a wall tool";
            _ask.Margin = new Thickness(0, 12, 0, 0);
            root.Children.Add(_ask);

            _error.Foreground = Brushes.Firebrick; _error.TextWrapping = TextWrapping.Wrap; _error.Margin = new Thickness(0, 8, 0, 0);
            root.Children.Add(_error);

            var buttons = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Right, Margin = new Thickness(0, 14, 0, 0) };
            var ok = new Button { Content = fromTool ? "Start" : "Save", Padding = new Thickness(26, 5, 26, 5), IsDefault = true };
            var cancel = new Button { Content = "Cancel", Padding = new Thickness(18, 5, 18, 5), Margin = new Thickness(0, 0, 8, 0), IsCancel = true };
            buttons.Children.Add(cancel); buttons.Children.Add(ok);
            root.Children.Add(buttons);
            ok.Click += (s, e) => Accept();
            cancel.Click += (s, e) => { DialogResult = false; };

            // ---- current values
            _baseLevel.SelectedIndex = Math.Max(0, IndexOf(_levels, current.BaseLevelId));
            _topLevel.SelectedIndex = Math.Max(0, IndexOf(_levels, current.TopLevelId));
            _type.SelectedIndex = Math.Max(0, _kind == SetupKind.Wall
                ? _types.FindIndex(t => current.TemplateTypeId != null && t.Id.Equals(current.TemplateTypeId))
                : _colTypes.FindIndex(t => current.TemplateTypeId != null && t.Id.Equals(current.TemplateTypeId)));
            _baseOffset.Text = Edit(current.BaseOffsetFt);
            _height.Text = Edit(current.HeightFt);
            _topOffset.Text = Edit(current.TopOffsetFt);
            _ask.IsChecked = askEachTime;
            if (current.HeightMode == 1 && _levels.Count > 1) _toLevel.IsChecked = true; else _fixed.IsChecked = true;
        }

        private static int IndexOf(List<Level> list, ElementId id) => id == null ? -1 : list.FindIndex(l => l.Id.Equals(id));

        private void Accept()
        {
            _error.Text = "";
            if (_baseLevel.SelectedIndex < 0 || _type.SelectedIndex < 0) { _error.Text = "Choose a level and a wall type."; return; }
            Level baseLvl = _levels[_baseLevel.SelectedIndex];
            double baseOff, height = 0, topOff = 0;
            if (!TryParse(_baseOffset.Text, out baseOff)) { _error.Text = "The base offset is not a valid length."; return; }
            bool toLevel = _toLevel.IsChecked == true;
            Level top = null;
            if (toLevel)
            {
                if (_topLevel.SelectedIndex < 0) { _error.Text = "Choose the level the walls go up to."; return; }
                top = _levels[_topLevel.SelectedIndex];
                if (!TryParse(_topOffset.Text, out topOff)) { _error.Text = "The top offset is not a valid length."; return; }
                height = top.Elevation + topOff - baseLvl.Elevation - baseOff;
                if (height < 0.1) { _error.Text = "The top must be above the base: " + top.Name + " is not high enough above " + baseLvl.Name + "."; return; }
            }
            else
            {
                if (!TryParse(_height.Text, out height) || height < 0.1) { _error.Text = "Enter a wall height (at least 3 cm)."; return; }
            }

            Result = new WallSetup
            {
                BaseLevelId = baseLvl.Id, BaseLevelName = baseLvl.Name, BaseOffsetFt = baseOff,
                TopLevelId = top != null ? top.Id : (_topLevel.SelectedIndex >= 0 ? _levels[_topLevel.SelectedIndex].Id : null),
                TopLevelName = top != null ? top.Name : (_topLevel.SelectedIndex >= 0 ? _levels[_topLevel.SelectedIndex].Name : ""),
                TopOffsetFt = topOff,
                HeightMode = toLevel ? 1 : 0, HeightFt = height,
                Kind = _kind,
                TemplateTypeId = _kind == SetupKind.Wall ? _types[_type.SelectedIndex].Id : _colTypes[_type.SelectedIndex].Id,
                TemplateName = _kind == SetupKind.Wall ? _types[_type.SelectedIndex].Name : _colTypes[_type.SelectedIndex].Family.Name + " : " + _colTypes[_type.SelectedIndex].Name,
            };
            AskEachTime = _ask.IsChecked == true;
            DialogResult = true;
        }

        // ---- units: the project's own length unit, parsed and shown the way Revit does -----------------------------------------
        private string Show(double feet) => UnitFormatUtils.Format(_units, SpecTypeId.Length, feet, false);
        private string Edit(double feet) => UnitFormatUtils.Format(_units, SpecTypeId.Length, feet, true);
        private bool TryParse(string text, out double feet) => UnitFormatUtils.TryParse(_units, SpecTypeId.Length, text.Trim(), out feet);
        private string UnitName()
        {
            try { return LabelUtils.GetLabelForUnit(_units.GetFormatOptions(SpecTypeId.Length).GetUnitTypeId()); } catch { return "project length units"; }
        }

        // ---- small layout helpers ---------------------------------------------------------------------------------------------------
        private static void Heading(StackPanel p, string text) =>
            p.Children.Add(new TextBlock { Text = text, FontWeight = FontWeights.SemiBold, FontSize = 13, Margin = new Thickness(0, 14, 0, 4) });

        private static TextBlock Label(string text) => new TextBlock { Text = text, Foreground = Brushes.Gray, Margin = new Thickness(0, 6, 0, 2) };

        private static void Note(StackPanel p, string text) =>
            p.Children.Add(new TextBlock { Text = text, Foreground = Brushes.Gray, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(0, 2, 0, 0) });
    }
}
