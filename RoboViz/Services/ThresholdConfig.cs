using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;

namespace RoboViz;

/// <summary>
/// Threshold-based evaluation for o-ring geometric metrics.
/// Port of _evaluate() and threshold loading from inspection_gui.py.
/// </summary>
public static class ThresholdConfig
{
    public static readonly MetricDef[] MetricDefs =
    [
        new("outer_radius",      "Outer Radius",     "px", "range", 1, "rework"),
        new("inner_radius",      "Inner Radius",     "px", "range", 1, "rework"),
        new("circularity_outer", "Outer Circularity", "",  "min",   3, "rework"),
        new("circularity_inner", "Inner Circularity", "",  "min",   3, "rework"),
        new("center_dist",       "Center Distance",  "px", "max",   1, "reject"),
        new("eccentricity_pct",  "Eccentricity",     "%",  "max",   2, "reject"),
    ];

    private static readonly int ReferenceResolution = 2448;

    /// <summary>
    /// Load tuned thresholds from a JSON file.
    /// Only loads the 6 metrics we use.
    /// </summary>
    public static Dictionary<string, MetricThreshold> LoadThresholds(string jsonPath)
    {
        var usedKeys = new HashSet<string>(MetricDefs.Select(m => m.Key));
        var thresholds = new Dictionary<string, MetricThreshold>();

        if (!File.Exists(jsonPath))
            return GetDefaultThresholds();

        string json = File.ReadAllText(jsonPath);
        using var doc = JsonDocument.Parse(json);

        foreach (var prop in doc.RootElement.EnumerateObject())
        {
            if (!usedKeys.Contains(prop.Name)) continue;
            double lo = prop.Value.GetProperty("lo").GetDouble();
            double hi = prop.Value.GetProperty("hi").GetDouble();
            thresholds[prop.Name] = new MetricThreshold(lo, hi);
        }

        // Fill any missing with defaults
        var defaults = GetDefaultThresholds();
        foreach (var key in usedKeys)
        {
            if (!thresholds.ContainsKey(key))
                thresholds[key] = defaults[key];
        }

        // Widen reject thresholds by 10% (matches Python logic)
        WidenRejectThresholds(thresholds);

        return thresholds;
    }

    /// <summary>
    /// Load thresholds from a CSV measurement stats file.
    /// Uses p5 as lo and p95 as hi for the 6 display metrics.
    /// CSV columns: metric,mean,std,min,max,p5,p95,n
    /// </summary>
    public static Dictionary<string, MetricThreshold> LoadThresholdsFromCsv(string csvPath)
    {
        var usedKeys = new HashSet<string>(MetricDefs.Select(m => m.Key));
        var thresholds = new Dictionary<string, MetricThreshold>();

        if (!File.Exists(csvPath))
            return GetDefaultThresholds();

        var lines = File.ReadAllLines(csvPath);
        foreach (var line in lines.Skip(1)) // skip header
        {
            var parts = line.Split(',');
            if (parts.Length < 7) continue;

            string metric = parts[0].Trim();
            if (!usedKeys.Contains(metric)) continue;

            if (!double.TryParse(parts[5], out double p5) ||
                !double.TryParse(parts[6], out double p95))
                continue;

            var def = MetricDefs.FirstOrDefault(d => d.Key == metric);
            if (def == null) continue;

            double lo, hi;
            if (def.ThreshType == "max")
            {
                lo = 0;
                hi = p95;
            }
            else if (def.ThreshType == "min")
            {
                lo = p5;
                hi = 9999;
            }
            else // range
            {
                lo = p5;
                hi = p95;
            }

            thresholds[metric] = new MetricThreshold(Math.Round(lo, 4), Math.Round(hi, 4));
        }

        // Fill missing with defaults
        var defaults = GetDefaultThresholds();
        foreach (var key in usedKeys)
        {
            if (!thresholds.ContainsKey(key))
                thresholds[key] = defaults[key];
        }

        // Widen reject thresholds by 10%
        WidenRejectThresholds(thresholds);

        return thresholds;
    }

    private static void WidenRejectThresholds(Dictionary<string, MetricThreshold> thresholds)
    {
        var rejectKeys = MetricDefs.Where(m => m.VerdictCategory == "reject").Select(m => m.Key).ToHashSet();
        foreach (var key in rejectKeys)
        {
            if (!thresholds.TryGetValue(key, out var t)) continue;
            double lo = t.Lo > 0 ? Math.Round(t.Lo * 0.9, 4) : t.Lo;
            double hi = t.Hi < 9999 ? Math.Round(t.Hi * 1.1, 4) : t.Hi;
            thresholds[key] = new MetricThreshold(lo, hi);
        }
    }

    public static Dictionary<string, MetricThreshold> GetDefaultThresholds() => new()
    {
        ["outer_radius"]      = new(650.0, 680.0),
        ["inner_radius"]      = new(375.0, 400.0),
        ["center_dist"]       = new(0.0, 35.0),
        ["eccentricity_pct"]  = new(0.0, 6.0),
        ["circularity_outer"] = new(0.75, 1.0),
        ["circularity_inner"] = new(0.75, 1.0),
    };

    /// <summary>
    /// Compute resolution scale factor for normalizing measurements.
    /// Linear metrics are divided by this scale.
    /// </summary>
    public static double ComputeResolutionScale(int imgW, int imgH)
    {
        int cur = Math.Max(imgW, imgH);
        return (double)cur / ReferenceResolution;
    }

    /// <summary>
    /// Normalize linear metrics by resolution scale.
    /// circularity and eccentricity are dimensionless — not scaled.
    /// </summary>
    public static Dictionary<string, double> NormalizeMeasurements(Dictionary<string, double> raw, double scale)
    {
        if (Math.Abs(scale - 1.0) < 1e-6) return raw;

        var normed = new Dictionary<string, double>(raw);
        // Only linear metrics are scaled (matches METRIC_SCALE_TYPE in Python)
        string[] linearKeys = ["outer_radius", "inner_radius", "center_dist"];
        foreach (var key in linearKeys)
        {
            if (normed.ContainsKey(key))
                normed[key] = normed[key] / scale;
        }
        return normed;
    }

    /// <summary>
    /// Evaluate all 6 metrics against thresholds.
    /// Returns per-metric pass/fail and overall verdict.
    /// </summary>
    public static (List<MetricEvalResult> Results, string Verdict, List<string> FailReasons)
        Evaluate(Dictionary<string, double> normedValues, Dictionary<string, MetricThreshold> thresholds)
    {
        var results = new List<MetricEvalResult>();
        var reworkFails = new List<string>();
        var rejectFails = new List<string>();

        foreach (var def in MetricDefs)
        {
            if (!normedValues.TryGetValue(def.Key, out double val)) continue;
            if (!thresholds.TryGetValue(def.Key, out var t)) continue;

            bool passed = true;

            // Special cases matching Python logic
            if (def.Key == "outer_radius")
            {
                if (val > t.Hi) passed = false;
            }
            else if (def.Key == "inner_radius")
            {
                if (val < t.Lo) passed = false;
            }
            else
            {
                if (def.ThreshType is "range" or "min" && val < t.Lo) passed = false;
                if (def.ThreshType is "range" or "max" && val > t.Hi) passed = false;
            }

            results.Add(new MetricEvalResult(
                def.Key, def.DisplayName, def.Unit,
                val, t.Lo, t.Hi, passed, def.VerdictCategory, def.Decimals));

            if (!passed)
            {
                if (def.VerdictCategory == "rework")
                    reworkFails.Add(def.DisplayName);
                else
                    rejectFails.Add(def.DisplayName);
            }
        }

        // Priority: rework first, then reject
        string verdict;
        var failReasons = new List<string>();
        if (reworkFails.Count > 0)
        {
            verdict = "REWORK";
            failReasons = reworkFails;
        }
        else if (rejectFails.Count > 0)
        {
            verdict = "REJECT";
            failReasons = rejectFails;
        }
        else
        {
            verdict = "PASS";
        }

        return (results, verdict, failReasons);
    }
}
