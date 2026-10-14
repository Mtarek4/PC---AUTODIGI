using System;
using System.Collections.Generic;
using System.IO;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.Colors;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;

namespace PointCloudWalls.Cad.AutoCAD
{
    /// <summary>
    /// The real AutoCAD drawing behind <see cref="ICadDrawing"/>. Everything the connector draws carries XDATA under
    /// the application name PCW_REVIT: scope, Revit UniqueId, role, hash. Entities without that XDATA are never touched.
    /// </summary>
    internal sealed class AcadDrawing : ICadDrawing
    {
        public const string AppName = "PCW_REVIT";

        private readonly Document _doc;
        private readonly Database _db;
        private readonly Transaction _tr;
        private BlockTableRecord _ms;

        public bool ZoomRequested { get; private set; }
        public string Name => Path.GetFileName(_doc.Name);

        public AcadDrawing(Document doc, Transaction tr)
        {
            _doc = doc; _db = doc.Database; _tr = tr;
            EnsureRegApp();
        }

        private BlockTableRecord ModelSpace(bool write)
        {
            if (_ms == null) _ms = (BlockTableRecord)_tr.GetObject(SymbolUtilityServices.GetBlockModelSpaceId(_db), OpenMode.ForRead);
            if (write && !_ms.IsWriteEnabled) _ms.UpgradeOpen();
            return _ms;
        }

        private void EnsureRegApp()
        {
            var rat = (RegAppTable)_tr.GetObject(_db.RegAppTableId, OpenMode.ForRead);
            if (rat.Has(AppName)) return;
            rat.UpgradeOpen();
            var rec = new RegAppTableRecord { Name = AppName };
            rat.Add(rec);
            _tr.AddNewlyCreatedDBObject(rec, true);
        }

        public void EnsureLayer(string name, short colorIndex)
        {
            var lt = (LayerTable)_tr.GetObject(_db.LayerTableId, OpenMode.ForRead);
            if (lt.Has(name)) return;                       // an existing layer keeps the user's own colour / settings
            lt.UpgradeOpen();
            var rec = new LayerTableRecord { Name = name, Color = Color.FromColorIndex(ColorMethod.ByAci, colorIndex) };
            lt.Add(rec);
            _tr.AddNewlyCreatedDBObject(rec, true);
        }

        private static ResultBuffer MakeXData(string scope, string revitId, string role, string hash) =>
            new ResultBuffer(
                new TypedValue((int)DxfCode.ExtendedDataRegAppName, AppName),
                new TypedValue((int)DxfCode.ExtendedDataAsciiString, scope),
                new TypedValue((int)DxfCode.ExtendedDataAsciiString, revitId),
                new TypedValue((int)DxfCode.ExtendedDataAsciiString, role),
                new TypedValue((int)DxfCode.ExtendedDataAsciiString, hash));

        public List<CadEntityRef> FindByScope(string scope)
        {
            var list = new List<CadEntityRef>();
            foreach (ObjectId id in ModelSpace(false))
            {
                var ent = _tr.GetObject(id, OpenMode.ForRead, false) as Entity;
                if (ent == null) continue;
                using (ResultBuffer rb = ent.GetXDataForApplication(AppName))
                {
                    if (rb == null) continue;
                    TypedValue[] tv = rb.AsArray();
                    if (tv.Length < 5) continue;
                    if ((string)tv[1].Value != scope) continue;
                    list.Add(new CadEntityRef { Handle = ent.Handle.ToString(), RevitId = (string)tv[2].Value, Role = (string)tv[3].Value, Hash = (string)tv[4].Value });
                }
            }
            return list;
        }

        public string Create(string scope, string revitId, string role, string hash, string layer, double elevation, double[] xy, bool closed)
        {
            var pl = new Polyline(xy.Length / 2);
            for (int i = 0; i < xy.Length / 2; i++) pl.AddVertexAt(i, new Point2d(xy[2 * i], xy[2 * i + 1]), 0, 0, 0);
            pl.Closed = closed;
            pl.Layer = layer;
            pl.Elevation = elevation;
            pl.XData = MakeXData(scope, revitId, role, hash);
            ModelSpace(true).AppendEntity(pl);
            _tr.AddNewlyCreatedDBObject(pl, true);
            return pl.Handle.ToString();
        }

        private ObjectId IdOf(string handle)
        {
            ObjectId id;
            if (!_db.TryGetObjectId(new Handle(Convert.ToInt64(handle, 16)), out id)) throw new InvalidOperationException("Entity " + handle + " no longer exists");
            return id;
        }

        public void Update(string handle, string hash, string layer, double elevation, double[] xy, bool closed)
        {
            var pl = (Polyline)_tr.GetObject(IdOf(handle), OpenMode.ForWrite);
            int nNew = xy.Length / 2;
            // move the existing vertices, then add or remove the difference: the entity (and its handle) is kept
            for (int i = 0; i < Math.Min(pl.NumberOfVertices, nNew); i++) pl.SetPointAt(i, new Point2d(xy[2 * i], xy[2 * i + 1]));
            for (int i = pl.NumberOfVertices; i < nNew; i++) pl.AddVertexAt(i, new Point2d(xy[2 * i], xy[2 * i + 1]), 0, 0, 0);
            while (pl.NumberOfVertices > nNew && pl.NumberOfVertices > 2) pl.RemoveVertexAt(pl.NumberOfVertices - 1);
            pl.Closed = closed;
            pl.Layer = layer;
            pl.Elevation = elevation;
            // keep scope / id / role, replace the hash
            using (ResultBuffer old = pl.GetXDataForApplication(AppName))
            {
                TypedValue[] tv = old.AsArray();
                pl.XData = MakeXData((string)tv[1].Value, (string)tv[2].Value, (string)tv[3].Value, hash);
            }
        }

        public string CreateBlock(string scope, string revitId, string role, string hash, string layer, double elevation,
                                   double insertX, double insertY, double rotation, string blockKind, double width, double height)
        {
            ObjectId btrId = ResolveBlockDefinition(blockKind);
            if (btrId == ObjectId.Null)
                throw new InvalidOperationException("No " + blockKind + " block is set up - run PCWSETBLOCKS first.");

            var br = new BlockReference(new Point3d(insertX, insertY, elevation), btrId) { Rotation = rotation, Layer = layer };
            ModelSpace(true).AppendEntity(br);
            _tr.AddNewlyCreatedDBObject(br, true);
            br.XData = MakeXData(scope, revitId, role, hash);
            TrySetDynamicSize(br, width, height);
            return br.Handle.ToString();
        }

        public void UpdateBlock(string handle, string hash, string layer, double elevation, double insertX, double insertY, double rotation, double width, double height)
        {
            var br = (BlockReference)_tr.GetObject(IdOf(handle), OpenMode.ForWrite);
            br.Position = new Point3d(insertX, insertY, elevation);
            br.Rotation = rotation;
            br.Layer = layer;
            TrySetDynamicSize(br, width, height);
            using (ResultBuffer old = br.GetXDataForApplication(AppName))
            {
                TypedValue[] tv = old.AsArray();
                br.XData = MakeXData((string)tv[1].Value, (string)tv[2].Value, (string)tv[3].Value, hash);
            }
        }

        /// <summary>The block definition for "door" or "window", importing it from the configured source drawing into
        /// this one the first time it is needed. Returns ObjectId.Null if no block is configured for that kind.</summary>
        private ObjectId ResolveBlockDefinition(string blockKind)
        {
            var cfg = BlockConfig.Load();
            string blockName = cfg.BlockNameFor(blockKind);
            if (string.IsNullOrEmpty(blockName)) return ObjectId.Null;

            var bt = (BlockTable)_tr.GetObject(_db.BlockTableId, OpenMode.ForRead);
            if (bt.Has(blockName)) return bt[blockName];

            if (string.IsNullOrEmpty(cfg.SourceDwgPath) || !File.Exists(cfg.SourceDwgPath))
                throw new InvalidOperationException("The " + blockKind + " block '" + blockName + "' is not in this drawing and its source file is missing - run PCWSETBLOCKS again.");

            using (var srcDb = new Database(false, true))
            {
                srcDb.ReadDwgFile(cfg.SourceDwgPath, FileOpenMode.OpenForReadAndAllShare, true, null);
                var idMap = new IdMapping();
                _db.WblockCloneObjects(new ObjectIdCollection(new[] { SrcBlockId(srcDb, blockName) }),
                                        _db.BlockTableId, idMap, DuplicateRecordCloning.Replace, false);
            }
            bt.UpgradeOpen();
            bt = (BlockTable)_tr.GetObject(_db.BlockTableId, OpenMode.ForRead);
            if (!bt.Has(blockName)) throw new InvalidOperationException("Block '" + blockName + "' could not be imported from '" + cfg.SourceDwgPath + "'.");
            return bt[blockName];
        }

        private static ObjectId SrcBlockId(Database srcDb, string blockName)
        {
            using (var tr = srcDb.TransactionManager.StartTransaction())
            {
                var bt = (BlockTable)tr.GetObject(srcDb.BlockTableId, OpenMode.ForRead);
                if (!bt.Has(blockName)) throw new InvalidOperationException("Block '" + blockName + "' was not found in the source drawing.");
                ObjectId id = bt[blockName];
                tr.Commit();
                return id;
            }
        }

        /// <summary>Best-effort: if this is a dynamic block and it exposes a property under a common width/height name,
        /// set it to the detected size. Block authors name these differently, so this can never be guaranteed to match -
        /// if nothing matches, the block is still placed correctly, just at its default size.</summary>
        private static readonly string[] WidthNames = { "Width", "Door Width", "Window Width", "W" };
        private static readonly string[] HeightNames = { "Height", "Door Height", "Window Height", "H" };
        private static void TrySetDynamicSize(BlockReference br, double width, double height)
        {
            if (!br.IsDynamicBlock) return;
            try
            {
                DynamicBlockReferencePropertyCollection props = br.DynamicBlockReferencePropertyCollection;
                foreach (DynamicBlockReferenceProperty p in props)
                {
                    if (p.ReadOnly) continue;
                    if (Array.IndexOf(WidthNames, p.PropertyName) >= 0) { try { p.Value = width; } catch { } }
                    else if (Array.IndexOf(HeightNames, p.PropertyName) >= 0) { try { p.Value = height; } catch { } }
                }
            }
            catch { }
        }

        public void Delete(string handle)
        {
            var ent = (Entity)_tr.GetObject(IdOf(handle), OpenMode.ForWrite);
            ent.Erase();
        }

        public void ZoomExtents() { ZoomRequested = true; }

        /// <summary>Erase everything this connector ever drew (all scopes). Used by PCWCLEAR.</summary>
        public int EraseAllConnectorEntities()
        {
            int n = 0;
            foreach (ObjectId id in ModelSpace(false))
            {
                var ent = _tr.GetObject(id, OpenMode.ForRead, false) as Entity;
                if (ent == null) continue;
                using (ResultBuffer rb = ent.GetXDataForApplication(AppName))
                {
                    if (rb == null) continue;
                }
                ent.UpgradeOpen();
                ent.Erase();
                n++;
            }
            return n;
        }
    }
}
