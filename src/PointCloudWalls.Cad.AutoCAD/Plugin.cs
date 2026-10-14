using System;
using System.Collections.Generic;
using System.Threading;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Runtime;
using PointCloudWalls.Cad;
using AcApp = Autodesk.AutoCAD.ApplicationServices.Application;

[assembly: ExtensionApplication(typeof(PointCloudWalls.Cad.AutoCAD.Plugin))]
[assembly: CommandClass(typeof(PointCloudWalls.Cad.AutoCAD.Commands))]

namespace PointCloudWalls.Cad.AutoCAD
{
    /// <summary>Loads with AutoCAD and keeps a private local channel open for Revit's "Publish to CAD".</summary>
    public sealed class Plugin : IExtensionApplication
    {
        private static CadServer _server;
        private static System.Threading.SynchronizationContext _ui;      // (AutoCAD has its own SynchronizationContext type - be explicit)
        internal static string PipeName = CadPipe.DefaultName;
        internal static int Published;

        public void Initialize()
        {
            _ui = System.Threading.SynchronizationContext.Current;          // AutoCAD's main (UI) thread
            StartListening(CadPipe.DefaultName);
            Say("\nPointCloudWalls CAD bridge is listening for Revit (PCWSTATUS for details, PCWSTOP to stop).\n");
        }

        public void Terminate() { StopListening(); }

        internal static bool IsListening => _server != null && _server.Running;
        internal static int Handled => _server == null ? 0 : _server.Handled;
        internal static string LastError => _server == null ? "" : _server.LastError;

        internal static void StartListening(string pipeName)
        {
            if (IsListening) return;
            PipeName = pipeName;
            _server = new CadServer(pipeName, Handle);
            _server.Start();
        }

        internal static void StopListening()
        {
            _server?.Stop();
            _server = null;
        }

        internal static void Say(string text)
        {
            try { AcApp.DocumentManager.MdiActiveDocument?.Editor.WriteMessage(text); } catch { }
        }

        // ------------------------------------------------------------------ runs on the pipe thread
        private static string Handle(string request)
        {
            string type = CadMessage.TypeOf(request);
            if (type == CadMessage.Ping)
                return OnUi(() => new PublishResult { Ok = true, Drawing = ActiveName() }).ToJson();
            if (type == CadMessage.Publish)
            {
                PublishBatch batch = PublishBatch.FromJson(request);
                return OnUi(() => Apply(batch)).ToJson();
            }
            return new PublishResult { Ok = false, Message = "Unknown message type '" + type + "'" }.ToJson();
        }

        private static string ActiveName()
        {
            Document d = AcApp.DocumentManager.MdiActiveDocument;
            return d == null ? "" : System.IO.Path.GetFileName(d.Name);
        }

        /// <summary>Run <paramref name="f"/> on AutoCAD's main thread and wait for it (the drawing must only be touched there).</summary>
        private static T OnUi<T>(Func<T> f)
        {
            if (_ui == null) return f();
            T result = default(T); System.Exception error = null;
            using (var done = new ManualResetEventSlim())
            {
                _ui.Post(_ =>
                {
                    try { result = f(); }
                    catch (System.Exception ex) { error = ex; }
                    finally { done.Set(); }
                }, null);
                if (!done.Wait(30000)) throw new TimeoutException("AutoCAD did not respond within 30 s - is a dialog open or a long command running?");
            }
            if (error != null) throw error;
            return result;
        }

        // ------------------------------------------------------------------ runs on the AutoCAD main thread
        private static PublishResult Apply(PublishBatch batch)
        {
            Document doc = AcApp.DocumentManager.MdiActiveDocument;
            if (doc == null) return new PublishResult { Ok = false, Message = "No drawing is open in AutoCAD." };

            PublishResult result;
            bool zoom;
            try
            {
                using (doc.LockDocument())
                using (Transaction tr = doc.Database.TransactionManager.StartTransaction())
                {
                    var drawing = new AcadDrawing(doc, tr);
                    result = CadSyncEngine.Apply(batch, drawing);
                    tr.Commit();
                    zoom = drawing.ZoomRequested;
                }
            }
            catch (Autodesk.AutoCAD.Runtime.Exception ex)
            {
                return new PublishResult { Ok = false, Message = "AutoCAD is busy (a command is waiting for input?). Press Esc in AutoCAD and publish again. (" + ex.ErrorStatus + ")" };
            }

            Published++;
            if (zoom) { try { doc.SendStringToExecute("_.ZOOM _E ", true, false, false); } catch { } }
            try
            {
                doc.Editor.WriteMessage("\nPCW: " + batch.Elements.Count + " wall(s) from Revit '" + batch.Source + "' - created " + result.Created +
                                        ", updated " + result.Updated + ", unchanged " + result.Unchanged + ", removed " + result.Deleted + ".\n");
                doc.Editor.UpdateScreen();
            }
            catch { }
            return result;
        }
    }

    public sealed class Commands
    {
        [CommandMethod("PCWLISTEN")]
        public void Listen()
        {
            Plugin.StartListening(CadPipe.DefaultName);
            Plugin.Say("\nPCW: listening for Revit on '" + Plugin.PipeName + "'.\n");
        }

        [CommandMethod("PCWSTOP")]
        public void Stop()
        {
            Plugin.StopListening();
            Plugin.Say("\nPCW: stopped listening.\n");
        }

        [CommandMethod("PCWSTATUS")]
        public void Status()
        {
            Plugin.Say("\nPCW CAD bridge: " + (Plugin.IsListening ? "LISTENING" : "not listening") + " on '" + Plugin.PipeName + "'" +
                       "\n  requests handled: " + Plugin.Handled + "   publishes applied: " + Plugin.Published +
                       (Plugin.LastError.Length > 0 ? "\n  last error: " + Plugin.LastError : "") + "\n");
        }

        /// <summary>Which block represents a door and which represents a window, and which drawing they live in.</summary>
        [CommandMethod("PCWSETBLOCKS")]
        public void SetBlocks()
        {
            Document doc = AcApp.DocumentManager.MdiActiveDocument;
            if (doc == null) return;
            Editor ed = doc.Editor;

            var ofd = new Autodesk.AutoCAD.Windows.OpenFileDialog("Select the drawing containing your door/window blocks",
                null, "dwg", "PCWSetBlocksSource", Autodesk.AutoCAD.Windows.OpenFileDialog.OpenFileDialogFlags.NoUrls);
            System.Windows.Forms.DialogResult dr;
            try { dr = ofd.ShowDialog(); }
            catch { ed.WriteMessage("\nPCW: could not open the file picker - is this AutoCAD (not AutoCAD LT)?\n"); return; }
            if (dr != System.Windows.Forms.DialogResult.OK) return;
            string path = ofd.Filename;

            List<string> names;
            try { names = ReadBlockNames(path); }
            catch (System.Exception ex) { ed.WriteMessage("\nPCW: could not read that drawing (" + ex.Message + ").\n"); return; }
            if (names.Count == 0) { ed.WriteMessage("\nPCW: that drawing has no blocks defined.\n"); return; }
            ed.WriteMessage("\nPCW: blocks found in that drawing: " + string.Join(", ", names) + "\n");

            string doorName = PromptBlockName(ed, "door", names);
            if (doorName == null) return;
            string winName = PromptBlockName(ed, "window", names);
            if (winName == null) return;

            var cfg = new BlockConfig { SourceDwgPath = path, DoorBlockName = doorName, WindowBlockName = winName };
            cfg.Save();
            ed.WriteMessage("\nPCW: door = '" + doorName + "', window = '" + winName + "', from '" + path + "'. Saved.\n");
        }

        private static List<string> ReadBlockNames(string path)
        {
            var names = new List<string>();
            using (var db = new Database(false, true))
            {
                db.ReadDwgFile(path, FileOpenMode.OpenForReadAndAllShare, true, null);
                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var bt = (BlockTable)tr.GetObject(db.BlockTableId, OpenMode.ForRead);
                    foreach (ObjectId id in bt)
                    {
                        var btr = (BlockTableRecord)tr.GetObject(id, OpenMode.ForRead);
                        if (!btr.IsLayout && !btr.IsAnonymous && !btr.Name.StartsWith("*")) names.Add(btr.Name);
                    }
                    tr.Commit();
                }
            }
            return names;
        }

        private static string PromptBlockName(Editor ed, string kind, List<string> known)
        {
            var opt = new PromptStringOptions("\nName of the " + kind + " block") { AllowSpaces = true };
            PromptResult pr = ed.GetString(opt);
            if (pr.Status != PromptStatus.OK || string.IsNullOrWhiteSpace(pr.StringResult)) return null;
            string name = pr.StringResult.Trim();
            if (!known.Contains(name)) ed.WriteMessage("\nPCW: '" + name + "' was not in the list above - saved anyway in case it is a dynamic block variant name.\n");
            return name;
        }

        /// <summary>Erase everything the connector drew (never anything else).</summary>
        [CommandMethod("PCWCLEAR")]
        public void Clear()
        {
            Document doc = AcApp.DocumentManager.MdiActiveDocument;
            if (doc == null) return;
            var opt = new PromptKeywordOptions("\nErase ALL walls published from Revit in this drawing? [Yes/No]", "Yes No") { AllowNone = false };
            opt.Keywords.Default = "No";
            PromptResult pr = doc.Editor.GetKeywords(opt);
            if (pr.Status != PromptStatus.OK || pr.StringResult != "Yes") return;
            int n;
            using (doc.LockDocument())
            using (Transaction tr = doc.Database.TransactionManager.StartTransaction())
            {
                n = new AcadDrawing(doc, tr).EraseAllConnectorEntities();
                tr.Commit();
            }
            doc.Editor.WriteMessage("\nPCW: erased " + n + " entities.\n");
        }
    }
}
