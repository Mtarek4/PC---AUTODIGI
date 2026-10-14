using System;

namespace PointCloudWalls.Core
{
    /// <summary>
    /// Picks the best of several candidate slice heights, given how many scan points were found at each. Used by the
    /// column tools' auto-height search (a plan-view cut plane can land in a sparse band of the scan even when a much
    /// denser band exists a bit higher or lower - e.g. near a slab or beam).
    /// </summary>
    public static class HeightSelector
    {
        /// <summary>
        /// <paramref name="counts"/>: point count found at each candidate height, in order. <paramref name="origIndex"/>:
        /// which one is the height the tool would otherwise use unchanged. Returns the chosen index.
        ///
        /// The RAW count at a single height is not a safe thing to maximise on its own: a beam, bracket or capital
        /// crossing the column at just one specific elevation can easily be denser there than the column's own,
        /// consistent surface - and picking that height fits the beam's shape, not the column's. A real column
        /// footprint holds roughly steady over a run of heights; a beam or bracket does not. So each candidate is
        /// scored by the WORST of itself and its two immediate neighbours (a "sustained" score) - winning requires
        /// good data on both sides, which a narrow one-elevation feature cannot fake. The first and last candidates
        /// have no neighbour on one side and can never win outright, so an isolated spike right at the edge of the
        /// search range cannot be picked either. A switch only happens when the alternative is CLEARLY better than
        /// the original (not just noise), guarded by both an absolute floor and a ratio over the original's own score.
        /// </summary>
        public static int SelectSustained(int[] counts, int origIndex, int minSwitchCount = 6, double minSwitchRatio = 1.3)
        {
            if (counts == null || counts.Length == 0) return origIndex;
            origIndex = Math.Max(0, Math.Min(counts.Length - 1, origIndex));

            int Sustained(int i) => i > 0 && i < counts.Length - 1 ? Math.Min(counts[i - 1], Math.Min(counts[i], counts[i + 1])) : counts[i];

            int bestIdx = origIndex, bestScore = Sustained(origIndex);
            for (int i = 1; i < counts.Length - 1; i++)
            {
                int sc = Sustained(i);
                if (sc > bestScore) { bestScore = sc; bestIdx = i; }
            }
            bool switched = bestIdx != origIndex && bestScore >= Math.Max(minSwitchCount, (int)(Sustained(origIndex) * minSwitchRatio));
            return switched ? bestIdx : origIndex;
        }
    }
}
