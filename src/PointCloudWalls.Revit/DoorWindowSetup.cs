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
    /// <summary>Which door and window family new instances are duplicated from, and which of that family's own
    /// parameters count as Width and Height - much simpler than wall/column setup otherwise, since an opening's level,
    /// sill and head height all come straight from what was detected in the scan.</summary>
    internal sealed class DoorWindowSetup
    {
        public ElementId DoorTypeId, WindowTypeId;
        public string DoorTypeName = "", WindowTypeName = "";
        public string DoorWidthParamName = "", DoorHeightParamName = "";
        public string WindowWidthParamName = "", WindowHeightParamName = "";

        public static DoorWindowSetup FromSettings(Document doc, Settings s)
        {
            var doors = DoorWindowBuilder.Types(doc, OpeningCategory.Door);
            var wins = DoorWindowBuilder.Types(doc, OpeningCategory.Window);
            var door = doors.FirstOrDefault(t => t.Family.Name + " : " + t.Name == s.DoorTypeName) ?? doors.FirstOrDefault();
            var win = wins.FirstOrDefault(t => t.Family.Name + " : " + t.Name == s.WindowTypeName) ?? wins.FirstOrDefault();
            return new DoorWindowSetup
            {
                DoorTypeId = door?.Id, DoorTypeName = door != null ? door.Family.Name + " : " + door.Name : "",
                WindowTypeId = win?.Id, WindowTypeName = win != null ? win.Family.Name + " : " + win.Name : "",
                DoorWidthParamName = s.DoorWidthParamName, DoorHeightParamName = s.DoorHeightParamName,
                WindowWidthParamName = s.WindowWidthParamName, WindowHeightParamName = s.WindowHeightParamName,
            };
        }

        public void SaveToSettings(Settings s)
        {
            s.DoorTypeName = DoorTypeName; s.WindowTypeName = WindowTypeName;
            s.DoorWidthParamName = DoorWidthParamName; s.DoorHeightParamName = DoorHeightParamName;
            s.WindowWidthParamName = WindowWidthParamName; s.WindowHeightParamName = WindowHeightParamName;
            s.Save();
        }
    }

    internal sealed class DoorWindowSetupWindow : Window
    {
        private readonly List<FamilySymbol> _doors, _wins;
        private readonly ComboBox _door = new ComboBox(), _win = new ComboBox();
        private readonly ComboBox _doorW = new ComboBox(), _doorH = new ComboBox();
        private readonly ComboBox _winW = new ComboBox(), _winH = new ComboBox();
        private readonly CheckBox _ask = new CheckBox();
        private readonly TextBlock _error = new TextBlock();

        public DoorWindowSetup Result { get; private set; }
        public bool AskEachTime { get; private set; }

        public DoorWindowSetupWindow(Document doc, DoorWindowSetup current, bool askEachTime, bool fromTool)
        {
            _doors = DoorWindowBuilder.Types(doc, OpeningCategory.Door);
            _wins = DoorWindowBuilder.Types(doc, OpeningCategory.Window);

            Title = "Door / window setup";
            Width = 480; SizeToContent = SizeToContent.Height; MinWidth = 420;
            WindowStartupLocation = WindowStartupLocation.CenterScreen;
            ResizeMode = ResizeMode.NoResize;
            try { new WindowInteropHelper(this).Owner = System.Diagnostics.Process.GetCurrentProcess().MainWindowHandle; } catch { }

            var root = new StackPanel { Margin = new Thickness(18, 14, 18, 14) };
            Content = root;

            root.Children.Add(new TextBlock { Text = fromTool ? "How should new doors and windows be built?" : Title, FontSize = 16, FontWeight = FontWeights.Bold });
            Note(root, "\"PC Door/Window\" works out width, height and sill from the scan on its own, decides whether it is a " +
                       "door or a window, and duplicates the matching family below with that size.");

            Heading(root, "Door family to duplicate from");
            foreach (var t in _doors) _door.Items.Add(t.Family.Name + " : " + t.Name);
            root.Children.Add(_door);
            if (_doors.Count == 0) Note(root, "No door family is loaded in this project yet - load one (Insert > Load Family) before using the tool.");
            AddParamRow(root, "Width / Height parameter", _doorW, _doorH);

            Heading(root, "Window family to duplicate from");
            foreach (var t in _wins) _win.Items.Add(t.Family.Name + " : " + t.Name);
            root.Children.Add(_win);
            if (_wins.Count == 0) Note(root, "No window family is loaded in this project yet - load one (Insert > Load Family) before using the tool.");
            AddParamRow(root, "Width / Height parameter", _winW, _winH);

            Note(root, "Which of the family's own parameters actually control its width and height - normally found automatically " +
                       "(\"Width\"/\"Height\" or similar), but pick them here if your family uses different names.");
            Note(root, "If the scan detects the OTHER kind from what you clicked for (e.g. it turns out to be a window, not a door), " +
                       "the matching family above is used automatically - the same as columns auto-switching between round and rectangular.");

            _ask.Content = "Ask me every time I start the Door/Window tool";
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

            _door.SelectionChanged += (s, e) => RefreshParams(_door, _doors, _doorW, _doorH, current.DoorWidthParamName, current.DoorHeightParamName);
            _win.SelectionChanged += (s, e) => RefreshParams(_win, _wins, _winW, _winH, current.WindowWidthParamName, current.WindowHeightParamName);

            _door.SelectedIndex = Math.Max(0, _doors.FindIndex(t => current.DoorTypeId != null && t.Id.Equals(current.DoorTypeId)));
            _win.SelectedIndex = Math.Max(0, _wins.FindIndex(t => current.WindowTypeId != null && t.Id.Equals(current.WindowTypeId)));
            _ask.IsChecked = askEachTime;
            // the SelectionChanged handlers above only fire on a CHANGE - populate once up front for the initial selection
            RefreshParams(_door, _doors, _doorW, _doorH, current.DoorWidthParamName, current.DoorHeightParamName);
            RefreshParams(_win, _wins, _winW, _winH, current.WindowWidthParamName, current.WindowHeightParamName);
        }

        private static void AddParamRow(StackPanel root, string label, ComboBox width, ComboBox height)
        {
            var row = new System.Windows.Controls.Grid { Margin = new Thickness(0, 4, 0, 0) };
            row.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            row.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(10) });
            row.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            var wPanel = new StackPanel(); wPanel.Children.Add(new TextBlock { Text = "Width parameter", FontSize = 11, Foreground = Brushes.Gray }); wPanel.Children.Add(width);
            var hPanel = new StackPanel(); hPanel.Children.Add(new TextBlock { Text = "Height parameter", FontSize = 11, Foreground = Brushes.Gray }); hPanel.Children.Add(height);
            System.Windows.Controls.Grid.SetColumn(wPanel, 0); System.Windows.Controls.Grid.SetColumn(hPanel, 2);
            row.Children.Add(wPanel); row.Children.Add(hPanel);
            root.Children.Add(row);
        }

        /// <summary>Repopulates a family's Width/Height parameter dropdowns from whichever type is now selected, listing
        /// every numeric parameter that family actually has - "(auto-detect)" restores the usual Width/Height guessing.</summary>
        private static void RefreshParams(ComboBox typeBox, List<FamilySymbol> types, ComboBox widthBox, ComboBox heightBox, string wantWidth, string wantHeight)
        {
            widthBox.Items.Clear(); heightBox.Items.Clear();
            widthBox.Items.Add("(auto-detect)"); heightBox.Items.Add("(auto-detect)");
            if (typeBox.SelectedIndex >= 0 && typeBox.SelectedIndex < types.Count)
                foreach (var n in DoorWindowBuilder.NumericParamNames(types[typeBox.SelectedIndex]))
                {
                    widthBox.Items.Add(n); heightBox.Items.Add(n);
                }
            widthBox.SelectedIndex = Math.Max(0, IndexOfText(widthBox, wantWidth));
            heightBox.SelectedIndex = Math.Max(0, IndexOfText(heightBox, wantHeight));
        }

        private static int IndexOfText(ComboBox box, string text)
        {
            if (string.IsNullOrEmpty(text)) return 0;
            for (int i = 0; i < box.Items.Count; i++) if ((string)box.Items[i] == text) return i;
            return 0;
        }

        private static string SelectedParamName(ComboBox box) => box.SelectedIndex <= 0 ? "" : (string)box.SelectedItem;

        private void Accept()
        {
            _error.Text = "";
            if (_door.SelectedIndex < 0 || _win.SelectedIndex < 0) { _error.Text = "Choose a door family and a window family (load one first if the list is empty)."; return; }
            Result = new DoorWindowSetup
            {
                DoorTypeId = _doors[_door.SelectedIndex].Id, DoorTypeName = _doors[_door.SelectedIndex].Family.Name + " : " + _doors[_door.SelectedIndex].Name,
                WindowTypeId = _wins[_win.SelectedIndex].Id, WindowTypeName = _wins[_win.SelectedIndex].Family.Name + " : " + _wins[_win.SelectedIndex].Name,
                DoorWidthParamName = SelectedParamName(_doorW), DoorHeightParamName = SelectedParamName(_doorH),
                WindowWidthParamName = SelectedParamName(_winW), WindowHeightParamName = SelectedParamName(_winH),
            };
            AskEachTime = _ask.IsChecked == true;
            DialogResult = true;
        }

        private static void Heading(StackPanel p, string text) =>
            p.Children.Add(new TextBlock { Text = text, FontWeight = FontWeights.SemiBold, FontSize = 13, Margin = new Thickness(0, 14, 0, 4) });
        private static void Note(StackPanel p, string text) =>
            p.Children.Add(new TextBlock { Text = text, Foreground = Brushes.Gray, TextWrapping = TextWrapping.Wrap, Margin = new Thickness(0, 2, 0, 0) });
    }
}
