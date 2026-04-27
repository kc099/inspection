using System;
using System.IO;
using System.Windows;
using System.Windows.Threading;
using Microsoft.Win32;

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

            // Auto-start: only re-register if already enabled (preserves user's toggle choice)
            if (IsAutoStartEnabled())
                RegisterAutoStart();
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

        private const string AutoStartKey = @"Software\Microsoft\Windows\CurrentVersion\Run";
        private const string AutoStartName = "RoboViz";

        public static void RegisterAutoStart()
        {
            try
            {
                string exePath = Environment.ProcessPath ?? "";
                if (string.IsNullOrEmpty(exePath)) return;

                using var key = Registry.CurrentUser.OpenSubKey(AutoStartKey, writable: true);
                key?.SetValue(AutoStartName, $"\"{exePath}\"");
            }
            catch { }
        }

        public static void RemoveAutoStart()
        {
            try
            {
                using var key = Registry.CurrentUser.OpenSubKey(AutoStartKey, writable: true);
                key?.DeleteValue(AutoStartName, throwOnMissingValue: false);
            }
            catch { }
        }

        public static bool IsAutoStartEnabled()
        {
            try
            {
                using var key = Registry.CurrentUser.OpenSubKey(AutoStartKey, writable: false);
                return key?.GetValue(AutoStartName) != null;
            }
            catch { return false; }
        }
    }
}
