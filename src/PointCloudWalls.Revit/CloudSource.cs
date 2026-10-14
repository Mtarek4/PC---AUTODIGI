using System;
using System.Collections.Generic;
using Autodesk.Revit.DB;
using Autodesk.Revit.DB.PointClouds;

namespace PointCloudWalls.Revit
{
    /// <summary>
    /// Reads points from one PointCloudInstance and always hands back points in REVIT model coordinates.
    /// The region asked for is an ORIENTED box (so a long diagonal wall does not pull in a huge axis-aligned box).
    ///
    /// Two things in the Revit docs are ambiguous, so we AUTO-DETECT them on first use by asking for a known region
    /// and checking where the returned points land:
    ///   1. is the filter (and the returned points) in Revit coordinates or in the cloud's local coordinates?
    ///   2. do the multi-plane filter normals point OUT of the region or INTO it?
    /// </summary>
    internal sealed class CloudSource
    {
        public readonly PointCloudInstance Pc;
        private readonly Transform _toRevit;
        private readonly Transform _toLocal;
        private bool _calibrated;
        private bool _filterInRevit = true, _pointsInRevit = true, _normalsOutward = true;

        public CloudSource(PointCloudInstance pc)
        {
            Pc = pc;
            _toRevit = pc.GetTransform();
            _toLocal = _toRevit.Inverse;
        }

        public bool IsIdentity => _toRevit.IsIdentity;
        public bool Calibrated => _calibrated;
        public Transform ToRevitTransform => _toRevit;
        public string ModeText => !_calibrated ? "not calibrated (no usable points returned yet)"
            : "filter in " + (_filterInRevit ? "Revit" : "cloud-local") + " coords, points returned in " +
              (_pointsInRevit ? "Revit" : "cloud-local") + " coords, plane normals " + (_normalsOutward ? "OUTWARD" : "INWARD");

        /// <summary>Axis-aligned convenience (used by the diagnostics): box [minR, maxR] in Revit feet.</summary>
        public List<XYZ> QueryBox(XYZ minR, XYZ maxR, double spacingFt, int maxPts)
        {
            var c = new XYZ((minR.X + maxR.X) / 2, (minR.Y + maxR.Y) / 2, 0);
            return Query(c, new XYZ(1, 0, 0), (maxR.X - minR.X) / 2, (maxR.Y - minR.Y) / 2, minR.Z, maxR.Z, spacingFt, maxPts);
        }

        /// <summary>
        /// Points inside an oriented box: centre (XY, feet), unit axis (XY), half length along the axis, half width across,
        /// between zMin and zMax. Result is in Revit coordinates.
        /// </summary>
        public List<XYZ> Query(XYZ center, XYZ axis, double halfLen, double halfWid, double zMin, double zMax, double spacingFt, int maxPts)
        {
            XYZ ax = new XYZ(axis.X, axis.Y, 0).Normalize();
            if (!_calibrated)
            {
                foreach (bool filterRevit in new[] { true, false })
                    foreach (bool outward in new[] { true, false })
                    {
                        List<XYZ> raw = Raw(filterRevit, outward, center, ax, halfLen, halfWid, zMin, zMax, spacingFt, maxPts);
                        if (raw.Count == 0) continue;
                        double fracRevit = FractionInside(raw, center, ax, halfLen, halfWid, zMin, zMax, false);
                        double fracLocal = FractionInside(raw, center, ax, halfLen, halfWid, zMin, zMax, true);
                        if (fracRevit < 0.9 && fracLocal < 0.9) continue;      // wrong normals / wrong space
                        _filterInRevit = filterRevit;
                        _normalsOutward = outward;
                        _pointsInRevit = fracRevit >= 0.9;                      // identity transform => "Revit"
                        _calibrated = true;
                        return ToRevit(raw);
                    }
                return new List<XYZ>();
            }
            return ToRevit(Raw(_filterInRevit, _normalsOutward, center, ax, halfLen, halfWid, zMin, zMax, spacingFt, maxPts));
        }

        private List<XYZ> Raw(bool filterRevit, bool outward, XYZ center, XYZ ax, double halfLen, double halfWid,
                              double zMin, double zMax, double spacingFt, int maxPts)
        {
            XYZ pp = new XYZ(-ax.Y, ax.X, 0);
            XYZ zUp = new XYZ(0, 0, 1);
            double s = outward ? 1.0 : -1.0;
            var planes = new List<Plane>
            {
                MakePlane(filterRevit, ax * s,     center + ax * halfLen),
                MakePlane(filterRevit, ax * -s,    center - ax * halfLen),
                MakePlane(filterRevit, pp * s,     center + pp * halfWid),
                MakePlane(filterRevit, pp * -s,    center - pp * halfWid),
                MakePlane(filterRevit, zUp * s,    new XYZ(center.X, center.Y, zMax)),
                MakePlane(filterRevit, zUp * -s,   new XYZ(center.X, center.Y, zMin)),
            };
            PointCloudFilter filter = PointCloudFilterFactory.CreateMultiPlaneFilter(planes);
            PointCollection col = Pc.GetPoints(filter, spacingFt, maxPts);
            var list = new List<XYZ>();
            foreach (CloudPoint cp in col) list.Add(new XYZ(cp.X, cp.Y, cp.Z));
            return list;
        }

        private Plane MakePlane(bool inRevit, XYZ normal, XYZ origin)
        {
            if (inRevit) return Plane.CreateByNormalAndOrigin(normal, origin);
            return Plane.CreateByNormalAndOrigin(_toLocal.OfVector(normal), _toLocal.OfPoint(origin));
        }

        private List<XYZ> ToRevit(List<XYZ> raw)
        {
            if (_pointsInRevit) return raw;
            var res = new List<XYZ>(raw.Count);
            foreach (var p in raw) res.Add(_toRevit.OfPoint(p));
            return res;
        }

        /// <summary>Share of points that fall inside the requested oriented box (small margin), assuming Revit or local coords.</summary>
        private double FractionInside(List<XYZ> pts, XYZ center, XYZ ax, double halfLen, double halfWid, double zMin, double zMax, bool pointsAreLocal)
        {
            const double m = 0.2;   // feet
            int inside = 0;
            foreach (var raw in pts)
            {
                XYZ p = pointsAreLocal ? _toRevit.OfPoint(raw) : raw;
                double dx = p.X - center.X, dy = p.Y - center.Y;
                double along = dx * ax.X + dy * ax.Y, across = -dx * ax.Y + dy * ax.X;
                if (Math.Abs(along) <= halfLen + m && Math.Abs(across) <= halfWid + m && p.Z >= zMin - m && p.Z <= zMax + m) inside++;
            }
            return (double)inside / pts.Count;
        }
    }
}
