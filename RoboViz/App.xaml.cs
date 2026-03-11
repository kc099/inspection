using System;
using System.IO;
using System.Windows;
using System.Windows.Threading;

namespace RoboViz
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {
        protected override void OnStartup(StartupEventArgs e)
        {
            base.OnStartup(e);

            DispatcherUnhandledException += OnDispatcherUnhandledException;
            AppDomain.CurrentDomain.UnhandledException += OnUnhandledException;
            TaskScheduler.UnobservedTaskException += OnUnobservedTaskException;
        }

        private void OnDispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
        {
            LogCrash("DispatcherUnhandled", e.Exception);
            MessageBox.Show(e.Exception.ToString(), "Unhandled UI Exception", MessageBoxButton.OK, MessageBoxImage.Error);
            e.Handled = true;
        }

        private static void OnUnhandledException(object sender, UnhandledExceptionEventArgs e)
        {
            if (e.ExceptionObject is Exception ex)
            {
                LogCrash("AppDomainUnhandled", ex);
                MessageBox.Show(ex.ToString(), "Fatal Exception", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private static void OnUnobservedTaskException(object? sender, UnobservedTaskExceptionEventArgs e)
        {
            LogCrash("UnobservedTask", e.Exception);
            e.SetObserved();
        }

        private static void LogCrash(string source, Exception ex)
        {
            try
            {
                string logPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "crash.log");
                string entry = $"[{DateTime.Now:O}] [{source}] {ex}\n";
                File.AppendAllText(logPath, entry);
            }
            catch { }
        }
    }
}
