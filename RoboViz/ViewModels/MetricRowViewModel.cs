using System.Windows.Media;

namespace RoboViz;

public class MetricRowViewModel
{
    public string Label { get; set; } = "";
    public string ValueText { get; set; } = "";
    public string Value2Text { get; set; } = "";
    public string LoText { get; set; } = "";
    public string HiText { get; set; } = "";
    public string StatusText { get; set; } = "";
    public SolidColorBrush ValueColor { get; set; } = new(Colors.Gray);
    public SolidColorBrush Value2Color { get; set; } = new(Colors.Gray);
    public SolidColorBrush StatusColor { get; set; } = new(Colors.Gray);
}
