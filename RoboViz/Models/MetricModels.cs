namespace RoboViz;

/// <summary>
/// 6 geometric metrics used for rework/reject evaluation.
/// Matches METRIC_DEFS from inspection_gui.py.
/// </summary>
public record MetricDef(
    string Key,
    string DisplayName,
    string Unit,
    string ThreshType,   // "range", "min", "max"
    int Decimals,
    string VerdictCategory // "rework" or "reject"
);

public record MetricThreshold(double Lo, double Hi);

public record MetricEvalResult(
    string Key,
    string DisplayName,
    string Unit,
    double Value,
    double Lo,
    double Hi,
    bool Passed,
    string VerdictCategory,
    int Decimals
);
