using System.Collections.Generic;
using System.Drawing;

namespace RoboViz;

/// <summary>
/// Geometric measurement results for an o-ring image.
/// </summary>
public class GeometricResult
{
    public double OuterRadius { get; set; }
    public double InnerRadius { get; set; }
    public double CenterDist { get; set; }
    public double EccentricityPct { get; set; }
    public double CircularityOuter { get; set; }
    public double CircularityInner { get; set; }
    public PointF OuterCenter { get; set; }
    public PointF InnerCenter { get; set; }

    public Dictionary<string, double> ToDictionary() => new()
    {
        ["outer_radius"] = OuterRadius,
        ["inner_radius"] = InnerRadius,
        ["center_dist"] = CenterDist,
        ["eccentricity_pct"] = EccentricityPct,
        ["circularity_outer"] = CircularityOuter,
        ["circularity_inner"] = CircularityInner,
    };
}
