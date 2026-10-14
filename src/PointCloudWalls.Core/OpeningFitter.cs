using System;
using System.Collections.Generic;
using System.Linq;

namespace PointCloudWalls.Core
{
    public enum OpeningKind { Door, Window }

    public sealed class OpeningFitOptions
    {
        public double CellSize = 0.05;              // occupancy grid cell (m)
        public double InlierTol = 0.010;            // scan roughness (shared with walls/columns)
        public double SearchHalfWidth = 1.8;        // how far each side of the click to look, along the wall (m)
        public double MinWidth = 0.4, MaxWidth = 3.0;
        public double MinHeight = 0.3, MaxHeight = 3.0;
        public double FloorTolerance = 0.10;        // an opening bottom within this of the wall base counts as "reaches the floor" (door)
        public double MinJambCoverage = 0.45;       // required fraction of each side's own length that has real, bordering points
        public double BridgeMaxWidth = 0.75;        // obstructions up to this wide (e.g. a person standing in the doorway) are bridged over if they don't reach the top of the search area
        public double LeafFloorGap = 0.012;         // a column with no real data within this of the true floor has its own lowest cluster of points excluded from the occupancy grid rather than treated as part of a jamb - a real jamb/pier is fixed structure and always reaches the floor to within ordinary scan noise; something that doesn't is floating above it by design, almost always a door leaf caught ajar in the scan (which always clears the floor by a real physical margin, typically 10-20mm+, so it can swing) - kept tight (tighter than a typical leaf's clearance, looser than plain scan noise at the already-density-thresholded floor line) so a genuine jamb is never mistaken for one
        public int RandomSeed = 12345;
        /// <summary>Meters, off by default. When set (together with a per-point `depths` argument to Fit - the
        /// distance of each point from the wall's own face, perpendicular to it), any point farther than this from
        /// the face is excluded from the occupancy grid before the flood fill runs. Real wall/jamb material always
        /// sits within a couple cm of the wall's own face; a door leaf swung open to any real angle does not - it
        /// reads every bit as dense and solid as real wall by simple point-density measures (this is NOT the same
        /// signal LeafFloorGap looks for: a wide-open leaf is not floor-gapped, it's just off-plane), so without this
        /// it gets mistaken for the far jamb and the flood fill stops on the leaf's own face instead of the true
        /// opening. Set this to the wall's own half-thickness plus a small tolerance (e.g. 60-80mm) for the real
        /// fix; leave it null to skip the check entirely (e.g. for synthetic tests with no depth data).</summary>
        public double? MaxWallDepth = null;
        /// <summary>Optional diagnostic sink - when set, Fit reports the internal decisions it made (search bounds, the
        /// per-column floor-vs-leaf density readings, the final flood-fill extents) as plain text lines. Never touches
        /// the fit itself; purely for troubleshooting a specific real scan after the fact.</summary>
        public Action<string> Log;

        public OpeningFitOptions Clone() => (OpeningFitOptions)MemberwiseClone();
    }

    public sealed class OpeningFitResult
    {
        public bool Success;
        public string Message = "";
        public OpeningKind Kind;
        public double AlongCenter;      // position along the wall from whatever origin the caller used (meters)
        public double Width, Height;
        public double SillHeight;       // height of the opening's bottom above the wall's base (0 for a door)
        public double Spread;
        public double Coverage;
        public string Grade = "";

        public override string ToString() =>
            !Success ? "FAIL: " + Message :
            $"OK  {Kind}  {Width * 1000:F0}x{Height * 1000:F0}mm  sill={SillHeight * 1000:F0}mm  spread={Spread * 1000:F1}mm  cov={Coverage * 100:F0}%  [{Grade}]";
    }

    /// <summary>
    /// Finds a door or window opening in a wall from a single click roughly inside it. Works in the wall's own 2D
    /// "unrolled" surface coordinates - X = distance along the wall, Y = height above the wall's base - built by the
    /// caller from the wall's real geometry and the point cloud, so this class has no Revit dependency.
    ///
    /// Unlike a wall or column (finding a SHAPE the data forms), an opening is a HOLE bounded by real data: the method
    /// is a flood fill through the empty region around the click, refined against the real jamb/header points that
    /// border it, classified as a door or window by whether the hole reaches the wall's base.
    /// </summary>
    public static class OpeningFitter
    {
        /// <summary>
        /// Exact Poisson CDF P(X &lt;= k) for X ~ Poisson(lambda), by direct term-by-term summation. k is always a
        /// small observed count in this file's use (a handful of points in a thin floor-clearance band), so this is
        /// numerically fine and, unlike a normal/mean-3*sigma approximation, never goes negative or misbehaves for
        /// small lambda.
        /// </summary>
        static double PoissonCdf(int k, double lambda)
        {
            if (lambda <= 0) return 1.0;
            if (k < 0) return 0.0;
            double term = Math.Exp(-lambda);
            double cdf = term;
            for (int i = 1; i <= k; i++) { term *= lambda / i; cdf += term; }
            return cdf;
        }

        /// <summary>
        /// A robust estimate of where the wall's SCANNED surface actually starts vertically - its real physical floor
        /// level - which can differ from a modeled wall element's own Base Constraint by a few centimetres (a common,
        /// unremarkable BIM modeling choice, e.g. extending a wall a little below finish floor to tie into a slab).
        /// Door/window classification needs the REAL floor, not the model's, or a genuine floor-reaching door can be
        /// misread as a window (or vice versa) purely because of that offset. Uses a low percentile of the points' own
        /// heights near the base, since real wall surface data clusters tightly at its true bottom - a few stray points
        /// below that (dust, debris, a skirting board) barely move a percentile.
        /// </summary>
        public static double EstimateFloorOffset(IReadOnlyList<P2> wallPoints, double binSize = 0.02)
        {
            var ys = wallPoints.Select(p => p.Y).Where(y => y > -1.0 && y < 1.5).ToList();
            if (ys.Count < 10) return 0;
            // A handful of noisy outlier points below the true floor (ordinary scan scatter) will always exist somewhere
            // in a large sample - taking the single lowest point (or a low percentile of individual points) is an
            // extreme-value statistic and is biased low by exactly that noise. Instead, points are binned by height and
            // the LOWEST BIN WITH SUBSTANTIAL DENSITY is used - a bin needs a real share of the peak bin's count before
            // it counts as "the wall really starts here", which a few stray points can't fake.
            var counts = new Dictionary<int, int>();
            foreach (var y in ys) { int b = (int)Math.Floor(y / binSize); counts[b] = counts.TryGetValue(b, out int c) ? c + 1 : 1; }
            int peak = counts.Values.Max();
            int threshold = Math.Max(3, (int)(peak * 0.15));
            int minBin = counts.Where(kv => kv.Value >= threshold).Min(kv => kv.Key);
            return minBin * binSize;
        }

        public static OpeningFitResult Fit(IReadOnlyList<P2> wallPoints, P2 click, double wallBase, double wallTop, OpeningFitOptions o = null,
            IReadOnlyList<double> depths = null)
        {
            o = o ?? new OpeningFitOptions();
            OpeningFitResult Fail(string m) => new OpeningFitResult { Success = false, Message = m };

            double alo = click.X - o.SearchHalfWidth, ahi = click.X + o.SearchHalfWidth;
            double margin = Math.Max(0.15, 3 * o.CellSize);
            // NO margin below the wall base: a door's threshold genuinely has no wall material there, so the flood fill
            // must treat the wall's own base as the natural stopping point (and success condition for "reaches the
            // floor"), not as empty space it can escape through into a "ran off the search area" rejection.
            double hlo = wallBase, hhi = wallTop + margin;
            List<P2> pts; double[] ptDepth = null;
            bool haveDepths = depths != null && depths.Count == wallPoints.Count;
            if (haveDepths)
            {
                // Built by hand, in lockstep with wallPoints, rather than two independent LINQ .Where() calls - that
                // would risk the points list and the depths list silently drifting out of alignment with each other.
                pts = new List<P2>();
                var dep = new List<double>();
                for (int i = 0; i < wallPoints.Count; i++)
                {
                    var p = wallPoints[i];
                    if (p.X >= alo - margin && p.X <= ahi + margin && p.Y >= hlo && p.Y <= hhi) { pts.Add(p); dep.Add(depths[i]); }
                }
                ptDepth = dep.ToArray();
            }
            else
            {
                pts = wallPoints.Where(p => p.X >= alo - margin && p.X <= ahi + margin && p.Y >= hlo && p.Y <= hhi).ToList();
            }
            o.Log?.Invoke($"click=({click.X:F3},{click.Y:F3})  wallBase={wallBase:F3} wallTop={wallTop:F3}  search x=[{alo:F3},{ahi:F3}] y=[{hlo:F3},{hhi:F3}]  pts={pts.Count}  LeafFloorGap={o.LeafFloorGap:F3}  MaxWallDepth={(o.MaxWallDepth.HasValue ? o.MaxWallDepth.Value.ToString("F3") : "off")}");
            if (pts.Count < 20) return Fail("Too few points on the wall surface near the click.");

            // ---- occupancy grid --------------------------------------------------------------------------------------------
            double cell = o.CellSize;
            int nx = (int)Math.Ceiling((ahi - alo) / cell) + 1, ny = (int)Math.Ceiling((hhi - hlo) / cell) + 1;
            if ((long)nx * ny > 4_000_000) return Fail("Search area too large for the current cell size.");
            int CellX(double x) => Math.Max(0, Math.Min(nx - 1, (int)((x - alo) / cell)));
            int CellY(double y) => Math.Max(0, Math.Min(ny - 1, (int)((y - hlo) / cell)));

            // A door leaf caught ajar in the scan reads, in this flattened profile, as a solid, often near-full-height
            // blob sitting INSIDE the true opening rather than against either real jamb - and because its own small
            // headroom clearance under the head is frequently a few mm to a couple of cm (well under one grid cell),
            // it can end up merged with the real header once the grid is built, making it indistinguishable from a
            // genuine floor-to-ceiling structural member by shape alone. What always tells them apart is the floor: a
            // real jamb, pier or header support is fixed structure and reaches the true floor to within ordinary scan
            // noise; a leaf never does - it always clears the floor by a real physical gap (typically 10-20mm+) so it
            // can swing. That test is done here, directly against the raw points (not the coming neighbour-smoothed
            // occupancy grid, which is too coarse to resolve a gap that small) and over a GROUP of a few columns at a
            // time, not one 5cm column alone - a single column's own point count near a specific height is too sparse
            // to judge reliably (the along-wall precision lost by grouping does not matter here: the boundary is
            // re-snapped to the real, unfiltered points afterwards anyway).
            //
            // A group can straddle the real boundary between solid wall and the opening - part real jamb, part open
            // floor-to-header space - and on such a group, comparing its floor band against the PEAK of its own whole
            // histogram is unreliable: the jamb's share of that group's width is small, so its floor-band count can
            // look sparse purely because the group itself is mostly empty air, not because the jamb doesn't reach the
            // floor. Comparing against a fixed global peak also fails on this group for the opposite reason: a group
            // that is genuinely part of the opening with no leaf, whose only material is real header well above the
            // floor, would have that header register as "the peak" and get its perfectly real material misread as
            // "not touching the floor". Both problems disappear by comparing the floor band's density against the
            // density measured a little further up, IN THE SAME GROUP: real wall material is roughly as dense just
            // above the floor as right at it, however much of the group's width happens to be real wall; a floor gap
            // (leaf or otherwise) is not - whatever sits above a genuine gap (leaf body, or just empty air below a
            // header that starts higher up) reads at a very different density than the near-empty band at the floor.
            var ptsForOcc = pts;
            {
                // A fixed, non-overlapping band can straddle a leaf's own real edge (catching only part of it, with
                // the rest folded into the neighbouring band along with real jamb material past it) - a partition
                // boundary landing a few centimetres into the leaf is enough to leave a fragment of it unstripped,
                // which then merges into the real jamb at the grid level exactly as before. A per-COLUMN sliding
                // window - centred on every 5cm column, stepping one column at a time rather than partitioning -
                // removes that alignment dependency: every column gets its own judgement from the material actually
                // around it, so nothing about the leaf's real extent depends on where an arbitrary band edge falls.
                int halfW = 2;   // window = 5 columns (~0.25m) centred on each column, same width as before
                var byCol = new List<double>[nx];
                foreach (var p in pts)
                {
                    int c = CellX(p.X);
                    (byCol[c] ?? (byCol[c] = new List<double>())).Add(p.Y);
                }
                var colCutoff = new double[nx];
                for (int c = 0; c < nx; c++) colCutoff[c] = double.NaN;
                double gap = o.LeafFloorGap;
                double refBand = Math.Max(0.15, 6 * gap);   // a band right above the floor band, used only to gauge this window's own local density
                const double binH = 0.01;
                for (int c = 0; c < nx; c++)
                {
                    int c0 = Math.Max(0, c - halfW), c1 = Math.Min(nx - 1, c + halfW);
                    List<double> ys = null;
                    for (int k = c0; k <= c1; k++) if (byCol[k] != null) (ys ?? (ys = new List<double>())).AddRange(byCol[k]);
                    if (ys == null || ys.Count < 10) continue;
                    int floorCnt = ys.Count(y => y <= wallBase + gap);
                    int refCnt = ys.Count(y => y > wallBase + gap && y <= wallBase + gap + refBand);
                    if (refCnt < 5) { o.Log?.Invoke($"  col {c} x={alo + c * cell:F3}: n={ys.Count} floorCnt={floorCnt} refCnt={refCnt} -> skip (too little material just above the floor band)"); continue; }
                    double refDensityPerM = refCnt / refBand;
                    double expectedFloorCnt = refDensityPerM * gap;
                    if (expectedFloorCnt < 1.0) { o.Log?.Invoke($"  col {c} x={alo + c * cell:F3}: n={ys.Count} floorCnt={floorCnt} refCnt={refCnt} expectedFloorCnt={expectedFloorCnt:F2} -> skip (too little local density)"); continue; }
                    double ratio = floorCnt / expectedFloorCnt;
                    // step55 fix, corrected: a flat ratio alone is a weak test at low counts - at expectedFloorCnt
                    // around 5-15 (routine on real, honestly-floor-touching wall once the query band was widened to
                    // catch a swung leaf elsewhere), ordinary Poisson noise alone can knock floorCnt below 30% of
                    // that expectation purely by chance, misreading solid wall as a floor gap and opening a false
                    // void right through it (confirmed with synthetic data this session: a false ~350mm gap opened
                    // in wall that had no floor gap at all). The first attempt at a fix used a linear mean-3*sigma
                    // lower bound, but that approximation goes negative for expectedFloorCnt below ~9 - exactly the
                    // regime a real leaf's floor clearance (a small, genuine gap) lands in - which silently disabled
                    // gap detection altogether and broke DoorLeafTests.cs outright (150/150 -> 0/150). A direct
                    // Poisson tail probability (exact, not a normal approximation) stays well-behaved at small
                    // counts: it asks "if this column really were touching the floor at the reference density, how
                    // likely is a floorCnt this low by chance alone?" - genuine, physically-continuous gaps (leaf
                    // clearance) score far below the threshold, while single-column noise on real touching wall
                    // does not.
                    double pFloorTooLow = PoissonCdf(floorCnt, expectedFloorCnt);
                    if (ratio >= 0.30 || pFloorTooLow > 0.08) { o.Log?.Invoke($"  col {c} x={alo + c * cell:F3}: n={ys.Count} floorCnt={floorCnt} refCnt={refCnt} ratio={ratio:F2} expectedFloorCnt={expectedFloorCnt:F1} p={pFloorTooLow:F4} -> touches floor, leave alone"); continue; }
                    o.Log?.Invoke($"  col {c} x={alo + c * cell:F3}: n={ys.Count} floorCnt={floorCnt} refCnt={refCnt} ratio={ratio:F2} -> FLOOR GAP candidate");
                    // This window's floor band is markedly sparser than the material sitting just above it - a real
                    // gap. Find where that material (leaf, if this is one) tops out, using the same
                    // adaptive-density histogram technique EstimateFloorOffset uses for the whole wall, so the cutoff
                    // sits right above the leaf's own body and below the real header.
                    var byBin = new Dictionary<int, int>();
                    int loBin = int.MaxValue, hiBin = int.MinValue;
                    foreach (var y in ys)
                    {
                        int b = (int)Math.Floor((y - hlo) / binH);
                        byBin[b] = byBin.TryGetValue(b, out int cnt) ? cnt + 1 : 1;
                        if (b < loBin) loBin = b; if (b > hiBin) hiBin = b;
                    }
                    int peak = byBin.Values.Max();
                    int thresh = Math.Max(3, (int)(peak * 0.20));
                    bool Dense(int b) => byBin.TryGetValue(b, out int cnt) && cnt >= thresh;
                    int loDenseBin = loBin;
                    for (int b = loBin; b <= hiBin; b++) if (Dense(b)) { loDenseBin = b; break; }
                    int clusterTopBin = loDenseBin, emptyRun = 0; bool gapFound = false;
                    for (int b = loDenseBin; b <= hiBin; b++)
                    {
                        if (Dense(b)) { clusterTopBin = b; emptyRun = 0; }
                        else if (++emptyRun >= 2) { gapFound = true; break; }   // a real gap above the cluster - stop, whatever is above is a separate feature
                    }
                    // If the material never actually breaks - it runs unbroken to the top of this window's own data -
                    // there is nothing here to distinguish from a real header: a genuine floating leaf always has a
                    // real gap above it before the header (that's its headroom clearance).
                    if (!gapFound) { o.Log?.Invoke($"    -> runs unbroken to the top of this window's data, no real gap found - treated as real structure, not stripped"); continue; }
                    colCutoff[c] = hlo + (clusterTopBin + 1) * binH;
                    o.Log?.Invoke($"    -> cutoff={colCutoff[c]:F3} (points at/below this height excluded for this column)");
                }
                bool anyColCutoff = colCutoff.Any(v => !double.IsNaN(v));
                bool useDepthFilter = ptDepth != null && o.MaxWallDepth.HasValue;
                if (anyColCutoff || useDepthFilter)
                {
                    var kept = new List<P2>(pts.Count);
                    int excludedByDepth = 0;
                    for (int i = 0; i < pts.Count; i++)
                    {
                        var p = pts[i];
                        int c = CellX(p.X);
                        double cu = colCutoff[c];
                        bool floorOk = double.IsNaN(cu) || p.Y > cu;
                        bool depthOk = !useDepthFilter || Math.Abs(ptDepth[i]) <= o.MaxWallDepth.Value;
                        if (!depthOk) excludedByDepth++;
                        if (floorOk && depthOk) kept.Add(p);
                    }
                    ptsForOcc = kept;
                    // A door leaf swung open to any real angle is not floor-gapped (see MaxWallDepth's own comment) -
                    // this is a second, independent exclusion pass, keyed on how far off the wall's own face a point
                    // sits rather than whether it reaches the floor, so it catches exactly the case the floor-gap
                    // test above cannot.
                    if (useDepthFilter) o.Log?.Invoke($"depth pre-filter: excluded {excludedByDepth} point(s) farther than {o.MaxWallDepth:F3}m off the wall's own face (not real wall - an open door leaf, furniture, etc.)");
                }
                o.Log?.Invoke($"leaf pre-filter: {ptsForOcc.Count}/{pts.Count} points kept for the occupancy grid ({pts.Count - ptsForOcc.Count} excluded)");
            }

            var counts = new int[nx, ny];
            foreach (var p in ptsForOcc) counts[CellX(p.X), CellY(p.Y)]++;

            // A single tiny cell's raw point count is too noisy to trust on its own: even genuinely solid wall material,
            // sampled at a realistic density, randomly leaves plenty of individual 5cm cells with 0 or 1 points just by
            // chance (ordinary Poisson variation) - reading those as "empty" would carve false gaps out of a solid wall.
            // Occupancy is judged over a small neighbourhood sum instead (a summed-area table, so each lookup stays O(1)
            // however large the grid), which is robust to that per-cell noise while still resolving a genuine opening
            // sharply, since a real gap is empty across many neighbouring cells at once, not just one.
            var sat = new int[nx + 1, ny + 1];
            for (int x = 0; x < nx; x++)
            {
                int run = 0;
                for (int y = 0; y < ny; y++) { run += counts[x, y]; sat[x + 1, y + 1] = sat[x, y + 1] + run; }
            }
            int NeighborSum(int x, int y, int r)
            {
                int x0 = Math.Max(0, x - r), x1 = Math.Min(nx, x + r + 1), y0 = Math.Max(0, y - r), y1 = Math.Min(ny, y + r + 1);
                return sat[x1, y1] - sat[x0, y1] - sat[x1, y0] + sat[x0, y0];
            }
            double density = pts.Count / Math.Max(1.0, (ahi - alo) * (hhi - hlo));   // points per m^2 in the search area
            int radius = density < 400 ? 2 : 1;                                       // widen the neighbourhood a bit when data is sparse
            double neighborhoodArea = (2 * radius + 1) * (2 * radius + 1) * cell * cell;
            int minOccupied = Math.Max(3, (int)(density * neighborhoodArea * 0.25));  // ~1/4 of the expected count - forgiving, not blind
            bool Occ(int x, int y) => x >= 0 && x < nx && y >= 0 && y < ny && NeighborSum(x, y, radius) >= minOccupied;

            // A person standing in the doorway (or a stray piece of furniture) shows up as dense, solid-looking material
            // right in the middle of an otherwise real opening - to the flood fill below, that looks exactly like a real
            // jamb, splitting one genuine opening into two narrow fragments that each fail the size check. A real jamb or
            // pier is (a) many cells wide and (b) reaches every height the fill would otherwise cross; a person is neither -
            // they are narrow, and someone under 2 m tall does not reach the header of a standard door. So before flood-
            // filling, any OCCUPIED region is "bridged over" (treated as empty for connectivity purposes only) if it is both
            // narrower than a plausible person and does not reach the top of the search area - a real jamb spanning the full
            // height, however narrow, is never bridged.
            bool[,] occGrid = new bool[nx, ny];
            for (int x = 0; x < nx; x++) for (int y = 0; y < ny; y++) occGrid[x, y] = Occ(x, y);
            bool[,] bridged = BridgeObstructions(occGrid, nx, ny, (int)Math.Ceiling(o.BridgeMaxWidth / cell));
            bool OccForFill(int x, int y) => x >= 0 && x < nx && y >= 0 && y < ny && bridged[x, y];

            // ---- flood fill from the click through the EMPTY region ------------------------------------------------------------
            int cx = CellX(click.X), cy = CellY(click.Y);
            if (OccForFill(cx, cy))
            {
                // the click landed on a surface point (sloppy click, or right at a jamb) - hunt for the nearest empty cell
                bool found = false;
                for (int r = 1; r <= 6 && !found; r++)
                    for (int dx = -r; dx <= r && !found; dx++)
                        for (int dy = -r; dy <= r && !found; dy++)
                        {
                            int x = cx + dx, y = cy + dy;
                            if (x < 0 || x >= nx || y < 0 || y >= ny || OccForFill(x, y)) continue;
                            cx = x; cy = y; found = true;
                        }
                if (!found) return Fail("No gap found near the click - the wall looks solid there.");
            }

            var visited = new bool[nx, ny];
            var stack = new Stack<(int x, int y)>();
            stack.Push((cx, cy)); visited[cx, cy] = true;
            int minXs = cx, maxXs = cx, minYs = cy, maxYs = cy;
            bool touchedBoundary = false;
            int guard = 0;
            while (stack.Count > 0)
            {
                var (x, y) = stack.Pop();
                if (guard++ > 3_000_000) return Fail("The gap near the click does not look like a bounded opening.");
                minXs = Math.Min(minXs, x); maxXs = Math.Max(maxXs, x);
                minYs = Math.Min(minYs, y); maxYs = Math.Max(maxYs, y);
                if (x == 0 || x == nx - 1 || y == ny - 1) touchedBoundary = true;         // ran off the search area
                foreach (var (nx2, ny2) in new[] { (x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1) })
                {
                    if (nx2 < 0 || nx2 >= nx || ny2 < 0 || ny2 >= ny || visited[nx2, ny2] || OccForFill(nx2, ny2)) continue;
                    visited[nx2, ny2] = true;
                    stack.Push((nx2, ny2));
                }
            }
            if (touchedBoundary) return Fail("The gap near the click runs past the search area - click nearer the middle of the opening, or widen the search range.");

            double aLo = alo + minXs * cell, aHi = alo + (maxXs + 1) * cell;
            double hLo = hlo + minYs * cell, hHi = hlo + (maxYs + 1) * cell;
            o.Log?.Invoke($"flood fill (grid, pre-refine): x=[{aLo:F3},{aHi:F3}] width={((aHi - aLo) * 1000):F0}mm  y=[{hLo:F3},{hHi:F3}] height={((hHi - hLo) * 1000):F0}mm");
            bool reachesFloor = hLo <= wallBase + o.FloorTolerance;
            if (reachesFloor) hLo = wallBase;

            // ---- refine the edges against the real bordering points (median of a tight band, like every other tool here) ------
            // The flood-fill's initial edge position comes from a grid cell boundary, so it can be off by close to a whole
            // cell width - a tolerance band sized only for genuine scan noise (a few mm) can be too narrow to reach the
            // true edge from there and gets permanently stuck short of it. The first pass uses a band wide enough to
            // reliably recapture the true edge from that initial uncertainty; later passes tighten it for precision,
            // the same coarse-to-fine pattern used for walls and columns.
            double edgeTol = Math.Max(o.InlierTol * 3, 0.012);
            double binSize = Math.Max(3 * cell, 0.10);
            // A door jamb's data is not a thin band the way a wall face or column outline is - it is the ENTIRE REST
            // of the wall surface, which densely fills any window straddling the boundary. A median over such a window
            // is pulled toward the bulk of that material, not the actual transition - so instead, the along-range is
            // split into small bins and, in EACH bin, only the single point CLOSEST to the current boundary guess (on
            // the solid side) is kept; the median of those closest-per-bin points then tracks the transition itself,
            // regardless of how much more material sits further back from it.
            double? BoundaryNear(bool vertical, double at, double lo, double hi, bool outward)
            {
                var closest = new Dictionary<int, (double coord, double dist)>();
                // Bug found this session: this used to iterate the raw, unfiltered `pts` - so even when the flood
                // fill correctly treated a swung leaf (or floor-gap material) as empty space via the depth/floor
                // filters baked into ptsForOcc, this sub-cell refinement step had no idea those points were
                // excluded and could get pulled right back toward them, since it just looks for the nearest point
                // to the current boundary guess with no concept of "this was already ruled out". Refinement must
                // agree with what the flood fill actually treated as solid, so it uses the same filtered set.
                foreach (var p in ptsForOcc)
                {
                    double coord = vertical ? p.X : p.Y, along = vertical ? p.Y : p.X;
                    if (along < lo - binSize || along > hi + binSize) continue;
                    double d = outward ? at - coord : coord - at;            // >=0 means on the solid (outside-the-gap) side
                    if (d < -edgeTol * 0.5) continue;                        // allow a little noise past the guess, not deep into the gap
                    int bin = (int)Math.Floor((along - lo) / binSize);
                    double dist = Math.Abs(coord - at);
                    if (!closest.TryGetValue(bin, out var cur) || dist < cur.dist) closest[bin] = (coord, dist);
                }
                return closest.Count < 4 ? (double?)null : Stats.Median(closest.Values.Select(v => v.coord).ToList());
            }
            for (int pass = 0; pass < 4; pass++)
            {
                double nLo = BoundaryNear(true, aLo, hLo, hHi, true) ?? aLo;
                double nHi = BoundaryNear(true, aHi, hLo, hHi, false) ?? aHi;
                double nTop = BoundaryNear(false, hHi, aLo, aHi, false) ?? hHi;
                double nBot = reachesFloor ? wallBase : (BoundaryNear(false, hLo, aLo, aHi, true) ?? hLo);
                aLo = nLo; aHi = nHi; hHi = nTop; hLo = nBot;
            }

            double width = aHi - aLo, height = hHi - hLo;
            o.Log?.Invoke($"after edge refinement: x=[{aLo:F3},{aHi:F3}] width={width * 1000:F0}mm  y=[{hLo:F3},{hHi:F3}] height={height * 1000:F0}mm  reachesFloor={reachesFloor}");

            // ---- hinge-line extrapolation ---------------------------------------------------------------------------
            // The depth filter above stops a leaf from being mistaken for wall almost everywhere along its face, but
            // right at its own hinge it can't: the hinge is mounted ON the jamb, so the leaf is genuinely AT the wall
            // plane there, and the material right around it is a real mix of leaf and (often partly occluded) jamb.
            // But that is exactly what makes the hinge findable a different way: a rigid flat leaf's depth off the
            // wall grows ~linearly with distance from its hinge (confirmed against two real doors this session - see
            // step55's diagnostics), so fitting a line through the EXCLUDED, out-of-plane material just past a found
            // edge and extrapolating it back to the wall's own face gives a direct, independent estimate of where the
            // hinge - and so the true jamb - actually is. This only ever extends an edge outward, never in, and only
            // when the trend is real (an honest linear fit, not noise): a wrong or missing extrapolation just leaves
            // the edge exactly where the flood fill already put it.
            bool useDepthFilterForHinge = ptDepth != null && o.MaxWallDepth.HasValue;
            if (useDepthFilterForHinge)
            {
                double midLo = hLo + Math.Max(0.05, 0.15 * height), midHi = hLo + Math.Min(height, 2.2) * 0.85;
                double WallFaceDepthNear(double at, bool outward)
                {
                    var ds = new List<double>();
                    for (int i = 0; i < pts.Count; i++)
                    {
                        var p = pts[i];
                        if (p.Y < midLo || p.Y > midHi) continue;
                        double d = outward ? at - p.X : p.X - at;
                        if (d < -edgeTol * 0.5 || d > 0.25) continue;
                        if (Math.Abs(ptDepth[i]) > o.MaxWallDepth.Value) continue;
                        ds.Add(ptDepth[i]);
                    }
                    return ds.Count >= 8 ? Stats.Median(ds) : 0;
                }
                double ExtrapolateHinge(double edge, bool outward, string side)
                {
                    double wallDepth = WallFaceDepthNear(edge, outward);
                    double dir2 = outward ? 1 : -1;
                    const double binW = 0.03, maxReach = 0.9;
                    // Deliberately looser than o.MaxWallDepth: right where the flood fill actually stopped, a
                    // shallow-angle leaf's near-hinge stretch has ALREADY dropped back within the strict occupancy
                    // threshold (that is exactly why the fill stopped there) - so there is barely any STRICTLY
                    // excluded material left beyond the edge to trace on a shallow leaf, even though the ramp is
                    // still clearly there in the data. Any point that measurably deviates from the local wall-face
                    // depth (not just the ones the grid filter rejected outright) is real signal for tracing the
                    // ramp back to its source.
                    double traceThresh = Math.Max(0.03, 3 * o.InlierTol);
                    var byBin = new SortedDictionary<int, List<double>>();
                    for (int i = 0; i < pts.Count; i++)
                    {
                        var p = pts[i];
                        if (p.Y < midLo || p.Y > midHi) continue;
                        if (Math.Abs(ptDepth[i] - wallDepth) < traceThresh) continue;   // indistinguishable from real wall depth - not ramp signal
                        double signedDist = dir2 * (p.X - edge);
                        if (signedDist < 0 || signedDist > maxReach) continue;
                        int bi = (int)Math.Floor(signedDist / binW);
                        (byBin.TryGetValue(bi, out var l) ? l : (byBin[bi] = new List<double>())).Add(ptDepth[i]);
                    }
                    // A real leaf's own points at a given distance from the hinge all sit at close to the same
                    // depth (ordinary scan noise only, a couple cm at most) - a bin whose depths are spread over
                    // many tens of cm is not that: it's a mix of the leaf with something else entirely at that
                    // along-wall position, most often a second surface glimpsed THROUGH the open doorway (a far
                    // wall behind it, visible only through the gap, at a depth that has nothing to do with the
                    // leaf's own geometry). Confirmed against a real scan this session: a ~1000-point cluster
                    // sitting at one x position with depths spanning 0.14m to 0.97m, which wrecked the regression
                    // (r2 dropped to 0.05) purely because it was averaged in as if it were ramp signal. Reject any
                    // bin whose spread (interquartile range) is too wide to be one coherent surface.
                    double Iqr(List<double> v) { var s = v.OrderBy(x => x).ToList(); int n = s.Count; return s[(int)(0.75 * (n - 1))] - s[(int)(0.25 * (n - 1))]; }
                    var xs = new List<double>(); var ds2 = new List<double>();
                    int rejectedContaminated = 0;
                    foreach (var kv in byBin)
                    {
                        if (kv.Value.Count < 15) continue;
                        if (Iqr(kv.Value) > 0.08) { rejectedContaminated++; continue; }
                        xs.Add(edge + dir2 * (kv.Key + 0.5) * binW); ds2.Add(Stats.Median(kv.Value));
                    }
                    if (rejectedContaminated > 0) o.Log?.Invoke($"  hinge trace ({side}): rejected {rejectedContaminated} bin(s) as contaminated (depth spread too wide to be one surface - likely something seen through the open doorway, not the leaf)");
                    if (xs.Count < 4) { o.Log?.Invoke($"  hinge trace ({side}): too little clean ramp material nearby - edge left as-is"); return edge; }
                    // simple least-squares line depth = m*x + b
                    double mx = xs.Average(), my = ds2.Average();
                    double sxy = 0, sxx = 0;
                    for (int i = 0; i < xs.Count; i++) { sxy += (xs[i] - mx) * (ds2[i] - my); sxx += (xs[i] - mx) * (xs[i] - mx); }
                    if (sxx < 1e-9) { o.Log?.Invoke($"  hinge trace ({side}): no along-wall spread in the excluded material - edge left as-is"); return edge; }
                    double m = sxy / sxx, b = my - m * mx;
                    // fit quality: how well the line actually explains the binned depths (a wandering/noisy cluster,
                    // not a real hinge sweep, must not be trusted to extrapolate from)
                    double ssTot = ds2.Sum(v => (v - my) * (v - my));
                    double ssRes = 0; for (int i = 0; i < xs.Count; i++) { double pr = m * xs[i] + b; ssRes += (ds2[i] - pr) * (ds2[i] - pr); }
                    double r2 = ssTot > 1e-9 ? 1 - ssRes / ssTot : 0;
                    if (Math.Abs(m) < 0.15 || r2 < 0.75) { o.Log?.Invoke($"  hinge trace ({side}): slope={m:F2} r2={r2:F2} - not a clean enough hinge sweep, edge left as-is"); return edge; }
                    double x0 = (wallDepth - b) / m;
                    double extension = dir2 * (x0 - edge);
                    if (extension < 0.02 || extension > maxReach) { o.Log?.Invoke($"  hinge trace ({side}): extrapolated hinge {extension * 1000:F0}mm out - outside plausible range, edge left as-is"); return edge; }
                    o.Log?.Invoke($"  hinge trace ({side}): slope={m:F2} r2={r2:F2} wallFaceDepth={wallDepth:F3} -> hinge/jamb at x={x0:F3} (extends edge by {extension * 1000:F0}mm)");
                    return x0;
                }
                // Last resort for a leaf swung open near 90 degrees: right there the hinge trace above has nothing
                // to extrapolate from (the leaf is nearly edge-on to the wall, so its depth doesn't sweep with
                // along-wall position - the regression that hinge tracing relies on gets slope~0, r2~0, and bails).
                // A floor-level density "footprint" cue was tried here first and removed for being too fragile
                // (see below). Per the user's own diagnosis of the failure: the excluded (out-of-plane) material
                // right at the edge in this case is not the leaf's face at all - it's the actual door frame/jamb
                // reveal surface, which is only as wide, along the wall, as the wall itself is thick (order
                // 100-250mm in the real doors measured this session). A door leaf's face, by contrast, reads as a
                // much wider surface (order 700mm+) when it's lying flat against or near the wall. So: measure the
                // along-wall WIDTH of the excluded material's own cluster that actually touches the current edge,
                // and use THAT - not any floor statistic - to tell a frame surface from a leaf. Only runs on a side
                // where the hinge trace above found nothing to extrapolate (never overrides a successful hinge fit).
                double WidthClassifyExtend(double edge, bool outward, string side)
                {
                    double dir2 = outward ? 1 : -1;
                    const double reach = 0.5;        // how far out to look for excluded material at all
                    const double touchTol = 0.03;     // a cluster must start within this of the edge to count as "touching" it
                    const double gapBreak = 0.05;     // along-wall gap that splits the excluded material into separate clusters
                    const double leafWidthMin = 0.55; // at/above this width the touching cluster reads as the leaf's own face, not a frame - leave alone
                    // Calibrated against the three real doors this session with a confirmed true width (895mm x2,
                    // 892mm): a full extend-to-the-cluster's-far-edge push overshot by 100-130mm (that cluster
                    // includes some of the leaf's own near-hinge material, not just the frame reveal), while a
                    // 70mm cap landed within 2-43mm of the true width on all three - including the one door where
                    // both edges triggered this at once. Kept modest on purpose: better to still fall a bit short
                    // than to repeat the earlier floor-footprint's habit of occasionally overshooting badly.
                    const double capReach = 0.07;
                    var items = new List<(double dist, double x)>();
                    for (int i = 0; i < pts.Count; i++)
                    {
                        var p = pts[i];
                        if (p.Y < midLo || p.Y > midHi) continue;
                        if (Math.Abs(ptDepth[i]) <= o.MaxWallDepth.Value) continue; // only the excluded (occluded/far) material
                        double signedDist = dir2 * (p.X - edge);
                        if (signedDist < -touchTol || signedDist > reach) continue;
                        items.Add((signedDist, p.X));
                    }
                    if (items.Count < 20) { o.Log?.Invoke($"  width scan ({side}): too little excluded material near the edge - edge left as-is"); return edge; }
                    items.Sort((a, b) => a.dist.CompareTo(b.dist));
                    double runStart = items[0].dist, runEnd = items[0].dist;
                    int runCount = 1;
                    for (int i = 1; i < items.Count; i++)
                    {
                        if (items[i].dist - runEnd > gapBreak) break;
                        runEnd = items[i].dist;
                        runCount++;
                    }
                    if (runCount < 20) { o.Log?.Invoke($"  width scan ({side}): no cluster of excluded material actually touches the edge - edge left as-is"); return edge; }
                    double clusterWidth = runEnd - Math.Max(runStart, 0);
                    if (clusterWidth < 0.03) { o.Log?.Invoke($"  width scan ({side}): touching cluster is only {clusterWidth * 1000:F0}mm wide - too thin to be anything, edge left as-is"); return edge; }
                    if (clusterWidth >= leafWidthMin)
                    {
                        o.Log?.Invoke($"  width scan ({side}): excluded material touching the edge is {clusterWidth * 1000:F0}mm wide - too wide to be a door frame/jamb surface (reads as the leaf itself), edge left as-is");
                        return edge;
                    }
                    double push = Math.Min(capReach, clusterWidth);
                    double newEdge = edge + dir2 * push;
                    o.Log?.Invoke($"  width scan ({side}): excluded material touching the edge is {clusterWidth * 1000:F0}mm wide - narrow enough to be a door frame/jamb surface, not the leaf, extending edge by {push * 1000:F0}mm to x={newEdge:F3}");
                    return newEdge;
                }
                double extLo = ExtrapolateHinge(aLo, false, "left");
                if (extLo == aLo) extLo = WidthClassifyExtend(aLo, false, "left");
                double extHi = ExtrapolateHinge(aHi, true, "right");
                if (extHi == aHi) extHi = WidthClassifyExtend(aHi, true, "right");
                if (extLo < aLo) aLo = extLo;
                if (extHi > aHi) aHi = extHi;
                width = aHi - aLo;
                o.Log?.Invoke($"after hinge-line extrapolation: x=[{aLo:F3},{aHi:F3}] width={width * 1000:F0}mm");
            }
            // ------------------------------------------------------------------------------------------------------

            if (width < o.MinWidth || width > o.MaxWidth) return Fail($"Opening width {width * 1000:F0} mm is outside the plausible range.");
            if (height < o.MinHeight || height > o.MaxHeight) return Fail($"Opening height {height * 1000:F0} mm is outside the plausible range.");

            // ---- jamb / header support: real points must actually border the opening on each relevant side --------------------
            double JambCoverage(bool vertical, double at, double lo, double hi, bool outward)
            {
                var along = new List<double>();
                foreach (var p in pts)
                {
                    double coord = vertical ? p.X : p.Y, a = vertical ? p.Y : p.X;
                    if (a < lo - edgeTol || a > hi + edgeTol) continue;
                    double d = outward ? at - coord : coord - at;
                    if (d >= -edgeTol * 0.5 && d <= edgeTol) along.Add(a);
                }
                if (along.Count < 4) return 0;
                along.Sort();
                double len = hi - lo;
                return len > 1e-9 ? Math.Min(1.0, (along[along.Count - 1] - along[0]) / len) : 1.0;
            }
            double covLeft = JambCoverage(true, aLo, hLo, hHi, true);
            double covRight = JambCoverage(true, aHi, hLo, hHi, false);
            double covTop = JambCoverage(false, hHi, aLo, aHi, false);
            double covBottom = reachesFloor ? 1.0 : JambCoverage(false, hLo, aLo, aHi, true);
            if (covLeft < o.MinJambCoverage || covRight < o.MinJambCoverage || covTop < o.MinJambCoverage || covBottom < o.MinJambCoverage)
                return Fail("Could not find solid wall material on all sides of this gap - it may not be a real opening.");

            var spreadVals = new List<double>();
            void Spread(bool vertical, double at, double lo, double hi, bool outward)
            {
                foreach (var p in pts)
                {
                    double coord = vertical ? p.X : p.Y, a = vertical ? p.Y : p.X;
                    if (a < lo - edgeTol || a > hi + edgeTol) continue;
                    double d = outward ? at - coord : coord - at;
                    if (d >= -edgeTol * 0.5 && d <= edgeTol) spreadVals.Add(coord - at);
                }
            }
            Spread(true, aLo, hLo, hHi, true); Spread(true, aHi, hLo, hHi, false); Spread(false, hHi, aLo, aHi, false);
            if (!reachesFloor) Spread(false, hLo, aLo, aHi, true);
            double spread = spreadVals.Count >= 4 ? Stats2.MadSigma(spreadVals) : o.InlierTol;
            double coverage = (covLeft + covRight + covTop + covBottom) / 4;
            string grade = spread <= 0.008 && coverage >= 0.7 ? "Good" : spread <= 0.015 && coverage >= 0.5 ? "OK" : "Check";

            return new OpeningFitResult
            {
                Success = true, Kind = reachesFloor ? OpeningKind.Door : OpeningKind.Window,
                AlongCenter = (aLo + aHi) / 2, Width = width, Height = height, SillHeight = hLo - wallBase,
                Spread = spread, Coverage = coverage, Grade = grade,
            };
        }

        /// <summary>Treats a small, isolated blob of "occupied" cells as empty (for flood-fill purposes only) if it is
        /// narrow and does not reach the top of the search grid - a real jamb or pier is either wide (connected to the
        /// surrounding wall mass) or reaches the header; a person standing in the doorway is neither. A blob touching the
        /// left or right edge of the grid is never bridged, since the search window may simply be cutting off a much
        /// larger real structure there. (A door leaf caught ajar is handled separately, earlier, by excluding its own
        /// points from the occupancy grid in the first place - see the floor-touch pre-filter in Fit - since it commonly
        /// reads as full-height and this narrow/short test alone would not catch it.)</summary>
        private static bool[,] BridgeObstructions(bool[,] occ, int nx, int ny, int maxWidthCells)
        {
            var result = (bool[,])occ.Clone();
            var visited = new bool[nx, ny];
            for (int x0 = 0; x0 < nx; x0++)
                for (int y0 = 0; y0 < ny; y0++)
                {
                    if (!occ[x0, y0] || visited[x0, y0]) continue;
                    var cells = new List<(int x, int y)>();
                    var stack = new Stack<(int x, int y)>();
                    stack.Push((x0, y0)); visited[x0, y0] = true;
                    int minX = x0, maxX = x0, maxY = y0;
                    bool touchesLeftRight = false;
                    while (stack.Count > 0)
                    {
                        var (x, y) = stack.Pop();
                        cells.Add((x, y));
                        if (x < minX) minX = x; if (x > maxX) maxX = x; if (y > maxY) maxY = y;
                        if (x == 0 || x == nx - 1) touchesLeftRight = true;
                        foreach (var (nx2, ny2) in new[] { (x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1) })
                        {
                            if (nx2 < 0 || nx2 >= nx || ny2 < 0 || ny2 >= ny || visited[nx2, ny2] || !occ[nx2, ny2]) continue;
                            visited[nx2, ny2] = true;
                            stack.Push((nx2, ny2));
                        }
                    }
                    int width = maxX - minX + 1;
                    bool reachesTop = maxY >= ny - 1;
                    if (!touchesLeftRight && width <= maxWidthCells && !reachesTop)
                        foreach (var (x, y) in cells) result[x, y] = false;
                }
            return result;
        }
    }
}
