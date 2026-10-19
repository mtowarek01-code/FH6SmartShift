using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;
using GameReaderCommon;
using Newtonsoft.Json;
using SimHub.Plugins;

namespace FH6SmartShift
{
    [PluginName("FH6SmartShift")]
    [PluginDescription("The Architect's Masterpiece - v1.1.1 (FH6 & Forza Motorsport Virtual Dyno, 100% Map Gate, Dragy & Tuning Advisor)")]
    [PluginAuthor("mtowarek01")]
    public class FH6SmartShift : IPlugin, IDataPlugin, IWPFSettings
    {
        private const string StoreKey = "fh6SmartShiftingData"; 

        private const double DownshiftCooldownSeconds = 0.45; 
        private const double UpshiftCooldownSeconds = 0.25; 
        private const double RatioLearnShiftSettleSeconds = 0.35;
        private const double ClutchSettleSeconds = 0.15; 
        private const double SaveIntervalSeconds = 15.0; 
        private const double MathEvalIntervalSeconds = 0.15; 

        private const double SafeLandingRpmMultiplier = 0.93; 
        private const double MaxAllowedDownshiftMultiplier = 0.88; 
        
        private const double LearnWotThreshold = 240.0;
        private const double RatioLearnAccelThreshold = 25.0;
        private const double LightWotThreshold = 225.0;
        private const double LiftThreshold = 205.0;
        private const double BrakeDeadzone = 5.0;
        
        private const double SlipRatioThreshold = 0.20;
        private const double SlipAngleThreshold = 0.25;
        private const double SusTravelReboundThreshold = 0.08;

        private readonly object _lock = new object();
        private static readonly HashSet<string> _missingProps = new HashSet<string>();

        private static readonly string[] LabelsPeak = { "0: (-)", "1: (P)", "2: (P)", "3: (P)", "4: (P)", "5: (P)", "6: (P)", "7: (P)", "8: (P)", "9: (P)", "10: (P)" };
        private static readonly string[] LabelsLimitConfirmed = { "0: (-)", "1: (L)", "2: (L)", "3: (L)", "4: (L)", "5: (L)", "6: (L)", "7: (L)", "8: (L)", "9: (L)", "10: (L)" };
        private static readonly string[] LabelsLearn = { "0: (-)", "1: (P?)", "2: (P?)", "3: (P?)", "4: (P?)", "5: (P?)", "6: (P?)", "7: (P?)", "8: (P?)", "9: (P?)", "10: (P?)" };
        private static readonly string[] LabelsLimit = { "0: (-)", "1: (L?)", "2: (L?)", "3: (L?)", "4: (L?)", "5: (L?)", "6: (L?)", "7: (L?)", "8: (L?)", "9: (L?)", "10: (L?)" };
        private static readonly string[] LabelsRatio = { "0: (-)", "1: (R)", "2: (R)", "3: (R)", "4: (R)", "5: (R)", "6: (R)", "7: (R)", "8: (R)", "9: (R)", "10: (R)" };
        private static readonly string[] LabelsZero = { "0: (-)", "1: (-)", "2: (-)", "3: (-)", "4: (-)", "5: (-)", "6: (-)", "7: (-)", "8: (-)", "9: (-)", "10: (-)" };

        private enum Drivetrain { FWD = 0, RWD = 1, AWD = 2 }

        private readonly double[] _ratioScratch = new double[25];
        private readonly double[] _precomputedPower = new double[401];

        class GlobalEngineProfile 
        {
            public double[] PowerMap = new double[401]; 
            public double[] TorqueMap = new double[401]; 
            public double[] BoostMap = new double[401];
            public double MaxPower = 0;
            public double RpmAtMaxPower = 0;
            public double MaxTorque = 0;
            public double RpmAtMaxTorque = 0;
            public double MaxBoost = 0;
            public double GhostLimiterRpm = 0; 
            public double GhostLimiterCandidateRpm = 0;
            public double GhostLimiterCandidateTime = 0; 
            public int GhostLimiterHits = 0; 
            public int[] BandSamples = new int[401];
            public int MinMappedBand = 400; 
            public int MaxMappedBand = 0;
            public double MappingProgress = 0;

            public double Best0To100 = 0;
            public double Best100To200 = 0;
            public double Best0To200 = 0;
        }

        class GearState
        {
            public double ShiftTriggerRpm = 0;
            public bool PowerPeakCaught = false;
            public bool IsConfirmed = false; 
            public bool IsMathOptimal = false; 
            public bool TriggerIsEdge = false; 
            public bool TriggerIsGhost = false; 
            
            public double RpmPerRadS = 0; 
            public double SavedRpmPerRadS = 0; 
            
            public double GhostLimiterAccumulator = 0; 
            public double GhostLimiterBreakAccumulator = 0;
            
            public double[] RatioHistory = new double[25];
            public int RatioHistoryIdx = 0;
            public int RatioSamples = 0;
            public int RatioMismatchCount = 0;
        }

        private Dictionary<int, GearState> _gearStates = new Dictionary<int, GearState>();
        private GlobalEngineProfile _globalEngine = new GlobalEngineProfile();
        private Dictionary<string, string> _externalCarList = new Dictionary<string, string>();
        private DateTime _lastCarListWriteTime = DateTime.MinValue;
        private double _lastCarListCheckTime = -100;

        private int _resetCurrentRequested = 0;
        private int _resetAllRequested = 0;
        private int _resetLimitGearsRequested = 0;
        private int _resetRatiosRequested = 0; 
        private int _resetDragyRequested = 0;
        private int _toggleBeepRequested = 0;

        private int _lastGear = 0;
        private double _lastDownshiftTime = -100;
        private double _lastUpshiftTime = -100; 
        private double _lastDecoupledTime = -100;
        private double _lastSaveTime = -100;
        private double _lastMathEvalTime = -100;
        private double _lastMapScanTime = -100; 

        private double _frameDt = 0.016;
        private double _lastRpmChangeTime = 0;
        private double _lastRpmForAccel = 0;
        private double _currentRpmAccel = 0;
        private double _peakRpmBeforeCut = 0;
        private double _smoothedLiveBuffer = 50.0;
        
        private int _cleanWotFrames = 0;
        private int _lastLearnedBand = -1;
        private double _lastLearnedPower = 0;
        private double _lastLearnedTorque = 0;
        private double _lastLearnedTime = -100;
        
        private double _lastSusTravelRL = 0;
        private double _lastSusTravelRR = 0;

        private double _lastSpeedKmhForTrend = 0;
        private double _speedTrendKmhPerS = 0;
        private bool _climbDownshiftActive = false;

        private double _staticChassisDiveBase = 0.0;
        private bool _isChassisBaseInitialized = false;

        private double _smoothedSlideLevel = 0.0;
        private double _prevSmoothedSlideLevel = 0.0;
        private double _smoothedSlideTrend = 0.0;

        private double _prevSpeedKmhDragy = 0.0;
        private double _prevTimeDragy = 0.0;
        private bool _dragyArmedFromStop = false;
        private bool _dragyRunningFromStop = false;
        private bool _dragyPassed100FromStop = false;
        private double _dragyStart0Time = 0.0;
        private bool _dragyRunning100To200 = false;
        private double _dragyStart100Time = 0.0;

        private double _lastBeepTime = -100.0;
        private int _beepInFlight = 0;
        
        private Stopwatch _appTime = new Stopwatch();
        private Stopwatch _dtStopwatch = new Stopwatch();
        private bool _shiftLightActive = false;
        private double _smoothedLedProximity = 0.0;
        private int _currentLedStage = 0;
        private bool _lastDownshiftSuggested = false;

        private bool _isDataDirty = false;
        private bool _mathNeedsEval = false;
        private int _saveInFlight = 0;

        public PluginManager PluginManager { get; set; }
        private CarStore _store;
        private CarStore _snapshotStore;
        private string _key = ""; 
        private int _lastCarOrdinal = -1;
        private int _lastCarPI = -1;
        private int _lastCarDrivetrain = -1;
        private int _lastCarEngineMaxRpmRounded = -1;

        class Outputs
        {
            public string ActiveGameMode = "FH6";
            public string CarKey = "";
            public string CarName = "Unknown Car";
            public bool IsShiftBeepEnabled = false;
            public bool IsTractionGood = true;
            public bool IsDownshiftSafe = false;
            public bool IsDownshiftSuggested = false;
            public bool IsLearningComplete = false;
            public bool AeroBottomingWarning = false;
            public bool PowerOversteerWarning = false;
            public double OversteerSeverity = 0.0;
            public bool IsCounterSteering = false;
            public double CounterSteerSeverity = 0.0; 
            public double SlideSeverity = 0.0;
            public double SlideTrend = 0.0;
            public bool IsSlideDeepening = false;
            public bool IsSlideRecovering = false;
            public string SlideStateText = "GRIP";
            public string SlideBarColor = "#FF9900";
            public bool IsTopGear = false;
            public bool IsGripLost = false; 
            public double BodySlipAngle = 0;
            public double MaxUsableRpm = 0;
            public double RevLimiterProximity = 0;
            public int LedActiveCount = 0;
            public bool ShiftLightFlash = false;
            public double ShiftTriggerRpm = 0;
            public double RpmAtMaxPower = 0;
            public double RpmAtMaxTorque = 0;
            public double DownshiftTriggerRpm = 0;
            public double OptimalShiftRpm = 0;
            public double LiveMappingProgress = 0;

            public double DragyLast0To100 = 0.0;
            public double DragyBest0To100 = 0.0;
            public double DragyLast100To200 = 0.0;
            public double DragyBest100To200 = 0.0;
            public double DragyLast0To200 = 0.0;
            public double DragyBest0To200 = 0.0;
            public string DragyStatus = "READY";

            public double DbgSteerRaw = 0;
            public double DbgYawRaw = 0;
            public double DbgChassisDive = 0.0;
            public string EngineMapStatus = "LEARN"; 
            public string MapCoverageBar = "[░░░░░░░░░░░░░░░░░░░░]";
            public string CurrentGearStatusText = "-";
            public string CurrentGearColor = "#00000000";
            public string[] GearText = new string[11];
            public string[] GearColor = new string[11];

            public Outputs()
            {
                for(int i = 0; i <= 10; i++) { GearText[i] = LabelsZero[i]; GearColor[i] = "#E74C3C"; }
            }
        }
        
        private Outputs _out = new Outputs();

        private static string GetCommonDataDir()
        {
            string dir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "PluginsData", "Common");
            if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
            return dir;
        }

        private static string GetReportFilePath()
        {
            return Path.Combine(GetCommonDataDir(), "FH6SmartShift_GarageReport.html");
        }

        private static string GetCarListFilePath()
        {
            return Path.Combine(GetCommonDataDir(), "FH6SmartShift_CarList.json");
        }

        private void EnsureAndLoadCarList()
        {
            try
            {
                string path = GetCarListFilePath();
                if (!File.Exists(path))
                {
                    var starterMap = new Dictionary<string, string>();
                    File.WriteAllText(path, JsonConvert.SerializeObject(starterMap, Formatting.Indented), Encoding.UTF8);
                }

                var info = new FileInfo(path);
                if (info.LastWriteTimeUtc != _lastCarListWriteTime)
                {
                    string json = File.ReadAllText(path, Encoding.UTF8);
                    var loaded = JsonConvert.DeserializeObject<Dictionary<string, string>>(json);
                    if (loaded != null)
                    {
                        lock (_lock)
                        {
                            _externalCarList = loaded;
                            foreach (var kvp in loaded)
                            {
                                _store.CarNames[kvp.Key] = kvp.Value;
                                _snapshotStore.CarNames[kvp.Key] = kvp.Value;
                            }
                        }
                    }
                    _lastCarListWriteTime = info.LastWriteTimeUtc;
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine("[FH6SmartShift] CarList load error: " + ex.Message);
            }
        }

        private string ResolveCarName(int ordinal, string fullKey)
        {
            if (!string.IsNullOrEmpty(fullKey) && _store.CarNames.TryGetValue(fullKey, out string exactName) && !string.IsNullOrWhiteSpace(exactName))
                return exactName;
            string ordKey = ordinal.ToString();
            if (_store.CarNames.TryGetValue(ordKey, out string ordName) && !string.IsNullOrWhiteSpace(ordName))
                return ordName;
            if (!string.IsNullOrEmpty(fullKey) && _externalCarList.TryGetValue(fullKey, out string extExact) && !string.IsNullOrWhiteSpace(extExact))
                return extExact;
            if (_externalCarList.TryGetValue(ordKey, out string extOrd) && !string.IsNullOrWhiteSpace(extOrd))
                return extOrd;
            return $"Car #{ordinal}";
        }

        private void TriggerShiftBeepSound(double now)
        {
            if (!_store.ShiftBeepEnabled) return;
            if (now - _lastBeepTime < 0.40) return;
            _lastBeepTime = now;

            if (Interlocked.CompareExchange(ref _beepInFlight, 1, 0) == 0)
            {
                Task.Run(() =>
                {
                    try
                    {
                        Console.Beep(2650, 60);
                    }
                    catch
                    {
                    }
                    finally
                    {
                        Interlocked.Exchange(ref _beepInFlight, 0);
                    }
                });
            }
        }

        public Control GetWPFSettingsControl(PluginManager pluginManager)
        {
            var bgDark = new SolidColorBrush(Color.FromRgb(13, 17, 23));
            var cardBg = new SolidColorBrush(Color.FromRgb(22, 27, 34));
            var borderBrush = new SolidColorBrush(Color.FromRgb(48, 54, 61));
            var textWhite = new SolidColorBrush(Color.FromRgb(230, 237, 243));
            var textMuted = new SolidColorBrush(Color.FromRgb(139, 148, 158));
            var accentGreen = new SolidColorBrush(Color.FromRgb(35, 134, 54));
            var accentBlue = new SolidColorBrush(Color.FromRgb(31, 111, 235));
            var accentRed = new SolidColorBrush(Color.FromRgb(207, 34, 46));

            var rootScroll = new ScrollViewer
            {
                VerticalScrollBarVisibility = ScrollBarVisibility.Auto,
                Background = bgDark
            };

            var mainStack = new StackPanel
            {
                Margin = new Thickness(24),
                MaxWidth = 920,
                HorizontalAlignment = HorizontalAlignment.Left
            };

            var titleBlock = new TextBlock
            {
                Text = "🏎️ FH6SmartShift v1.1.1 — FH6 & Forza Motorsport Control Center",
                FontSize = 20,
                FontWeight = FontWeights.Bold,
                Foreground = textWhite,
                Margin = new Thickness(0, 0, 0, 6)
            };
            var subTitleBlock = new TextBlock
            {
                Text = "Real-time 401-Band Virtual Dyno (100% Map Gate), 12-LED Shift Engine, Dragy Timer & Gearbox Tuning Advisor.",
                FontSize = 13,
                Foreground = textMuted,
                Margin = new Thickness(0, 0, 0, 18)
            };
            mainStack.Children.Add(titleBlock);
            mainStack.Children.Add(subTitleBlock);

            var statusBorder = new Border
            {
                Background = cardBg,
                BorderBrush = borderBrush,
                BorderThickness = new Thickness(1),
                CornerRadius = new CornerRadius(8),
                Padding = new Thickness(18),
                Margin = new Thickness(0, 0, 0, 16)
            };
            var statusStack = new StackPanel();
            var liveHeader = new TextBlock
            {
                Text = "📡 LIVE VEHICLE & TELEMETRY STATUS (AUTO-DETECTS FH6 / FORZA MOTORSPORT)",
                FontSize = 12,
                FontWeight = FontWeights.Bold,
                Foreground = accentBlue,
                Margin = new Thickness(0, 0, 0, 8)
            };
            var liveCarText = new TextBlock
            {
                Text = "Active Car: Waiting for Forza UDP telemetry...",
                FontSize = 15,
                FontWeight = FontWeights.SemiBold,
                Foreground = textWhite,
                Margin = new Thickness(0, 0, 0, 4)
            };
            var liveDynoText = new TextBlock
            {
                Text = "Dyno Mapping: 0% (LEARN) | Peak Power: — | Optimal Shift Target: —",
                FontSize = 13.5,
                Foreground = textMuted,
                Margin = new Thickness(0, 0, 0, 4)
            };
            var liveDragyText = new TextBlock
            {
                Text = "Dragy Records — 0-100 km/h: — | 100-200 km/h: — | 0-200 km/h: —",
                FontSize = 13.5,
                Foreground = new SolidColorBrush(Color.FromRgb(63, 185, 80))
            };
            statusStack.Children.Add(liveHeader);
            statusStack.Children.Add(liveCarText);
            statusStack.Children.Add(liveDynoText);
            statusStack.Children.Add(liveDragyText);
            statusBorder.Child = statusStack;
            mainStack.Children.Add(statusBorder);

            var settingsBorder = new Border
            {
                Background = cardBg,
                BorderBrush = borderBrush,
                BorderThickness = new Thickness(1),
                CornerRadius = new CornerRadius(8),
                Padding = new Thickness(18),
                Margin = new Thickness(0, 0, 0, 16)
            };
            var settingsStack = new StackPanel();
            settingsStack.Children.Add(new TextBlock
            {
                Text = "⚙️ SHIFT LIGHT & AUDIO CONFIGURATION",
                FontSize = 12,
                FontWeight = FontWeights.Bold,
                Foreground = accentBlue,
                Margin = new Thickness(0, 0, 0, 12)
            });

            var beepCheck = new CheckBox
            {
                Content = "🔊 Enable F1 / GT3 Audio Shift Beep (2650 Hz tone when all 12 LEDs lock at ShiftLightFlash)",
                IsChecked = _store.ShiftBeepEnabled,
                Foreground = textWhite,
                FontSize = 13.5,
                Margin = new Thickness(0, 0, 0, 14)
            };
            beepCheck.Checked += (s, e) => { lock (_lock) { _store.ShiftBeepEnabled = true; _out.IsShiftBeepEnabled = true; _isDataDirty = true; } TriggerAsyncSave(); };
            beepCheck.Unchecked += (s, e) => { lock (_lock) { _store.ShiftBeepEnabled = false; _out.IsShiftBeepEnabled = false; _isDataDirty = true; } TriggerAsyncSave(); };
            settingsStack.Children.Add(beepCheck);

            double initialBuffer = _store.ReactionBufferFactor > 0.05 ? _store.ReactionBufferFactor : 0.25;
            var sliderLabel = new TextBlock
            {
                Text = $"⚡ Driver Reaction-Time Lead Buffer: {initialBuffer:0.00} s (Scales dynamic RPM lead in lower gears)",
                Foreground = textWhite,
                FontSize = 13.5,
                Margin = new Thickness(0, 0, 0, 6)
            };
            var bufferSlider = new Slider
            {
                Minimum = 0.10,
                Maximum = 0.40,
                Value = initialBuffer,
                TickFrequency = 0.01,
                IsSnapToTickEnabled = true,
                Margin = new Thickness(0, 0, 0, 4)
            };
            bufferSlider.ValueChanged += (s, e) =>
            {
                double val = Math.Round(bufferSlider.Value, 2);
                sliderLabel.Text = $"⚡ Driver Reaction-Time Lead Buffer: {val:0.00} s (Scales dynamic RPM lead in lower gears)";
                lock (_lock)
                {
                    _store.ReactionBufferFactor = val;
                    _snapshotStore.ReactionBufferFactor = val;
                    _isDataDirty = true;
                }
            };
            settingsStack.Children.Add(sliderLabel);
            settingsStack.Children.Add(bufferSlider);
            settingsBorder.Child = settingsStack;
            mainStack.Children.Add(settingsBorder);

            var actionsBorder = new Border
            {
                Background = cardBg,
                BorderBrush = borderBrush,
                BorderThickness = new Thickness(1),
                CornerRadius = new CornerRadius(8),
                Padding = new Thickness(18)
            };
            var actionsStack = new StackPanel();
            actionsStack.Children.Add(new TextBlock
            {
                Text = "🛠️ GARAGE INSPECTOR & MEMORY ACTIONS",
                FontSize = 12,
                FontWeight = FontWeights.Bold,
                Foreground = accentBlue,
                Margin = new Thickness(0, 0, 0, 12)
            });

            var wrapButtons = new WrapPanel { Orientation = Orientation.Horizontal };

            Button MakeButton(string label, Brush bg, Action onClick)
            {
                var btn = new Button
                {
                    Content = label,
                    Background = bg,
                    Foreground = Brushes.White,
                    FontWeight = FontWeights.SemiBold,
                    FontSize = 13,
                    Padding = new Thickness(14, 8, 14, 8),
                    Margin = new Thickness(0, 0, 10, 10),
                    BorderThickness = new Thickness(0),
                    Cursor = System.Windows.Input.Cursors.Hand
                };
                btn.Click += (s, e) => onClick();
                return btn;
            }

            wrapButtons.Children.Add(MakeButton("📊 Open Virtual Dyno & Tuning Report (HTML)", accentGreen, () => OpenGarageReportInBrowser()));
            wrapButtons.Children.Add(MakeButton("📂 Open Common Data Folder (JSON / CarList)", accentBlue, () =>
            {
                try { Process.Start(new ProcessStartInfo { FileName = GetCommonDataDir(), UseShellExecute = true }); } catch { }
            }));
            wrapButtons.Children.Add(MakeButton("⚙️ Reset Current Car Gear Ratios (After Tuning)", new SolidColorBrush(Color.FromRgb(56, 63, 71)), () => Interlocked.Exchange(ref _resetRatiosRequested, 1)));
            wrapButtons.Children.Add(MakeButton("⏱️ Reset Current Car Dragy Records", new SolidColorBrush(Color.FromRgb(56, 63, 71)), () => Interlocked.Exchange(ref _resetDragyRequested, 1)));
            wrapButtons.Children.Add(MakeButton("🔄 Reset Current Car Full Memory", accentRed, () => Interlocked.Exchange(ref _resetCurrentRequested, 1)));

            actionsStack.Children.Add(wrapButtons);
            actionsBorder.Child = actionsStack;
            mainStack.Children.Add(actionsBorder);

            var uiTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
            uiTimer.Tick += (s, e) =>
            {
                if (!mainStack.IsVisible) return;
                lock (_lock)
                {
                    if (string.IsNullOrEmpty(_out.CarKey))
                    {
                        liveCarText.Text = $"Active Car ({_out.ActiveGameMode}): Waiting for on-track telemetry...";
                        liveDynoText.Text = $"Saved Cars in Database: {_store.EngineProfiles.Count} | Audio Shift Beep: {(_store.ShiftBeepEnabled ? "ON" : "OFF")}";
                        liveDragyText.Text = "Dragy Status: READY";
                    }
                    else
                    {
                        liveCarText.Text = $"Active Car [{_out.ActiveGameMode}]: {_out.CarName}  ({_out.CarKey})";
                        double hp = _globalEngine.MaxPower / 745.7;
                        liveDynoText.Text = $"Dyno Mapping: {_out.LiveMappingProgress:0.0}% ({_out.EngineMapStatus}) | Peak Power: {hp:0.1} HP @ {_out.RpmAtMaxPower:0} RPM | Shift Target: {_out.OptimalShiftRpm:0} RPM";
                        string s0100 = _out.DragyBest0To100 > 0 ? $"{_out.DragyBest0To100:0.00} s" : "—";
                        string s100200 = _out.DragyBest100To200 > 0 ? $"{_out.DragyBest100To200:0.00} s" : "—";
                        string s0200 = _out.DragyBest0To200 > 0 ? $"{_out.DragyBest0To200:0.00} s" : "—";
                        liveDragyText.Text = $"Dragy PB — 0-100 km/h: {s0100}  |  100-200 km/h: {s100200}  |  0-200 km/h: {s0200}   ({ _out.DragyStatus })";
                    }
                    if (beepCheck.IsChecked != _store.ShiftBeepEnabled)
                        beepCheck.IsChecked = _store.ShiftBeepEnabled;
                }
            };
            uiTimer.Start();

            rootScroll.Content = mainStack;
            return rootScroll;
        }

        public void Init(PluginManager pluginManager)
        {
            try {
                string dir = GetCommonDataDir();
                string p = Path.Combine(dir, "FH6SmartShift_fh6SmartShiftingData.json");
                if (File.Exists(p)) {
                    string todayBak = p + ".bak_" + DateTime.Now.ToString("yyyyMMdd");
                    if (!File.Exists(todayBak)) {
                        File.Copy(p, todayBak, true);
                        Debug.WriteLine($"[FH6SmartShift] Utworzono dzienna kopie zapasowa: {todayBak}");
                    }
                    
                    var dirInfo = new DirectoryInfo(dir);
                    var baks = dirInfo.GetFiles("FH6SmartShift_fh6SmartShiftingData.json.bak_*");
                    if (baks.Length > 3) {
                        Array.Sort(baks, (a, b) => b.CreationTime.CompareTo(a.CreationTime));
                        for (int i = 3; i < baks.Length; i++) {
                            baks[i].Delete();
                        }
                    }
                }
            } 
            catch (Exception ex) 
            { 
                Debug.WriteLine($"[FH6SmartShift] Blad krytyczny systemu backupu: {ex.Message}");
            }

            _store = this.ReadCommonSettings<CarStore>(StoreKey, () => new CarStore()) ?? new CarStore();
            if (_store.CarNames == null) _store.CarNames = new Dictionary<string, string>();
            if (_store.ReactionBufferFactor < 0.10 || _store.ReactionBufferFactor > 0.45) _store.ReactionBufferFactor = 0.25;
            if (_store.SchemaVersion < 2) { _store.SchemaVersion = 2; this.SaveCommonSettings(StoreKey, _store); }
            
            bool globalBoostCleaned = false;
            foreach (var ep in _store.EngineProfiles.Values)
            {
                if (ep.BoostMap != null)
                {
                    double maxB = 0;
                    for (int i = 0; i < ep.BoostMap.Length; i++)
                    {
                        if (ep.BoostMap[i] < 0)
                        {
                            ep.BoostMap[i] = 0;
                            globalBoostCleaned = true;
                        }
                        if (ep.BoostMap[i] > maxB) maxB = ep.BoostMap[i];
                    }
                    if (maxB > ep.MaxBoost) ep.MaxBoost = maxB;
                }
            }
            if (globalBoostCleaned) _isDataDirty = true;

            _snapshotStore = CloneCarStore(_store);
            EnsureAndLoadCarList();
            _out.IsShiftBeepEnabled = _store.ShiftBeepEnabled;
            
            try { WriteHtmlGarageReport(_snapshotStore, ""); }
            catch (Exception ex) { Debug.WriteLine("[FH6SmartShift] Initial HTML report error: " + ex.Message); }
            
            _appTime.Start();
            _dtStopwatch.Start();
            
            pluginManager.AddAction("ResetPowerMemory", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetCurrentRequested, 1));
            pluginManager.AddAction("ResetAllCarsMemory", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetAllRequested, 1));
            pluginManager.AddAction("ResetLimitGears", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetLimitGearsRequested, 1));
            pluginManager.AddAction("ResetGearRatios", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetRatiosRequested, 1));
            pluginManager.AddAction("ResetDragyRecords", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetDragyRequested, 1));
            pluginManager.AddAction("ToggleShiftBeep", this.GetType(), (a, b) => Interlocked.Exchange(ref _toggleBeepRequested, 1));
            pluginManager.AddAction("OpenDynoReport", this.GetType(), (a, b) => OpenGarageReportInBrowser());

            this.AttachDelegate("ActiveGameMode", () => _out.ActiveGameMode);
            this.AttachDelegate("MaxUsableRpm", () => _out.MaxUsableRpm);
            this.AttachDelegate("RevLimiterProximity", () => _out.RevLimiterProximity);
            this.AttachDelegate("LedActiveCount", () => _out.LedActiveCount);
            this.AttachDelegate("ShiftLightFlash", () => _out.ShiftLightFlash);
            this.AttachDelegate("IsShiftBeepEnabled", () => _out.IsShiftBeepEnabled);
            this.AttachDelegate("CurrentCarKey", () => _out.CarKey);
            this.AttachDelegate("CurrentCarName", () => _out.CarName);
            this.AttachDelegate("IsLearningComplete", () => _out.IsLearningComplete);
            this.AttachDelegate("ShiftTriggerRpm", () => _out.ShiftTriggerRpm);
            this.AttachDelegate("RpmAtMaxPower", () => _out.RpmAtMaxPower);
            this.AttachDelegate("RpmAtMaxTorque", () => _out.RpmAtMaxTorque);
            this.AttachDelegate("DownshiftTriggerRpm", () => _out.DownshiftTriggerRpm);
            this.AttachDelegate("OptimalShiftRpm", () => _out.OptimalShiftRpm);
            this.AttachDelegate("IsDownshiftSafe", () => _out.IsDownshiftSafe);
            this.AttachDelegate("IsDownshiftSuggested", () => _out.IsDownshiftSuggested);
            this.AttachDelegate("IsTractionGood", () => _out.IsTractionGood);
            this.AttachDelegate("BodySlipAngle", () => _out.BodySlipAngle);
            this.AttachDelegate("AeroBottomingWarning", () => _out.AeroBottomingWarning);
            this.AttachDelegate("PowerOversteerWarning", () => _out.PowerOversteerWarning);
            this.AttachDelegate("OversteerSeverity", () => _out.OversteerSeverity);
            this.AttachDelegate("IsCounterSteering", () => _out.IsCounterSteering);
            this.AttachDelegate("CounterSteerSeverity", () => _out.CounterSteerSeverity); 
            this.AttachDelegate("SlideSeverity", () => _out.SlideSeverity);
            this.AttachDelegate("SlideTrend", () => _out.SlideTrend);
            this.AttachDelegate("IsSlideDeepening", () => _out.IsSlideDeepening);
            this.AttachDelegate("IsSlideRecovering", () => _out.IsSlideRecovering);
            this.AttachDelegate("SlideStateText", () => _out.SlideStateText);
            this.AttachDelegate("SlideBarColor", () => _out.SlideBarColor);
            this.AttachDelegate("IsTopGear", () => _out.IsTopGear);
            this.AttachDelegate("IsGripLost", () => _out.IsGripLost);
            this.AttachDelegate("LiveMappingProgress", () => _out.LiveMappingProgress);
            this.AttachDelegate("EngineMapStatus", () => _out.EngineMapStatus);
            this.AttachDelegate("MapCoverageBar", () => _out.MapCoverageBar);
            this.AttachDelegate("CurrentGearStatusText", () => _out.CurrentGearStatusText);
            this.AttachDelegate("CurrentGearColor", () => _out.CurrentGearColor);

            this.AttachDelegate("Dragy_Last_0_100", () => _out.DragyLast0To100);
            this.AttachDelegate("Dragy_Best_0_100", () => _out.DragyBest0To100);
            this.AttachDelegate("Dragy_Last_100_200", () => _out.DragyLast100To200);
            this.AttachDelegate("Dragy_Best_100_200", () => _out.DragyBest100To200);
            this.AttachDelegate("Dragy_Last_0_200", () => _out.DragyLast0To200);
            this.AttachDelegate("Dragy_Best_0_200", () => _out.DragyBest0To200);
            this.AttachDelegate("Dragy_Status", () => _out.DragyStatus);
            
            this.AttachDelegate("Dbg_SteerRaw", () => _out.DbgSteerRaw);
            this.AttachDelegate("Dbg_YawRaw", () => _out.DbgYawRaw);
            this.AttachDelegate("Dbg_ChassisDive", () => _out.DbgChassisDive);

            this.AttachDelegate("Dbg_GhostLimiter", () => _globalEngine.GhostLimiterRpm);
            this.AttachDelegate("Dbg_GhostCandidate", () => _globalEngine.GhostLimiterCandidateRpm);
            this.AttachDelegate("Dbg_GhostHits", () => _globalEngine.GhostLimiterHits);

            for (int i = 1; i <= 10; i++)
            {
                int gearIndex = i;
                this.AttachDelegate($"Gear{gearIndex}Text", () => _out.GearText[gearIndex]);
                this.AttachDelegate($"Gear{gearIndex}Color", () => _out.GearColor[gearIndex]);
            }
        }

        public void End(PluginManager pluginManager) 
        {
            Stopwatch sw = Stopwatch.StartNew();
            while (Volatile.Read(ref _saveInFlight) == 1 && sw.ElapsedMilliseconds < 500) { Thread.Sleep(10); }
            lock (_lock) 
            { 
                if (_isDataDirty) 
                { 
                    if (!string.IsNullOrEmpty(_key)) 
                    {
                        SaveEngineProfileToStore(); 
                        UpdateSnapshotWithCurrentCar();
                    }
                    this.SaveCommonSettings(StoreKey, _snapshotStore); 
                    try { WriteHtmlGarageReport(_snapshotStore, _key); }
                    catch (Exception ex) { Debug.WriteLine("[FH6SmartShift] End HTML report error: " + ex.Message); }
                } 
            }
        }

        private void OpenGarageReportInBrowser()
        {
            Task.Run(() =>
            {
                try
                {
                    EnsureAndLoadCarList();
                    CarStore snapshotCopy;
                    string activeKey;
                    lock (_lock)
                    {
                        if (!string.IsNullOrEmpty(_key))
                        {
                            SaveEngineProfileToStore();
                            UpdateSnapshotWithCurrentCar();
                        }
                        snapshotCopy = CloneCarStore(_snapshotStore);
                        activeKey = _key ?? "";
                    }

                    string reportPath = GetReportFilePath();
                    WriteHtmlGarageReport(snapshotCopy, activeKey);

                    Process.Start(new ProcessStartInfo
                    {
                        FileName = reportPath,
                        UseShellExecute = true
                    });
                }
                catch (Exception ex)
                {
                    Debug.WriteLine("[FH6SmartShift] OpenDynoReport failed: " + ex.Message);
                }
            });
        }

        private static void WriteHtmlGarageReport(CarStore store, string activeCarKey)
        {
            string jsonPayload = JsonConvert.SerializeObject(store ?? new CarStore());
            string safeActiveKey = JsonConvert.SerializeObject(activeCarKey ?? "");
            string reportPath = GetReportFilePath();

            string html = @"<!DOCTYPE html>
<html lang=""en"">
<head>
<meta charset=""UTF-8"">
<title>FH6SmartShift v1.1.1 — FH6 &amp; Forza Motorsport Virtual Dyno, Dragy &amp; Tuning Inspector</title>
<style>
  * { box-sizing: border-box; }
  body {
    background: #0d1117;
    color: #e6edf3;
    font-family: 'Segoe UI', -apple-system, BlinkMacSystemFont, Arial, sans-serif;
    margin: 0;
    padding: 24px;
  }
  .container { max-width: 1240px; margin: 0 auto; }
  .header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-wrap: wrap;
    gap: 12px;
    background: #161b22;
    border: 1px solid #30363d;
    border-radius: 10px;
    padding: 16px 22px;
    margin-bottom: 18px;
  }
  .header h1 { margin: 0; font-size: 19.5px; color: #fff; }
  .controls { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
  .btn {
    background: #21262d;
    color: #e6edf3;
    border: 1px solid #30363d;
    padding: 8px 12px;
    border-radius: 6px;
    font-weight: 600;
    cursor: pointer;
    font-size: 12.5px;
    transition: background 0.15s;
  }
  .btn:hover { background: #30363d; }
  .btn-green { background: #238636; border-color: #2ea043; color: #fff; }
  .btn-green:hover { background: #2ea043; }
  .btn-blue { background: #1f6feb; border-color: #388bfd; color: #fff; }
  .btn-blue:hover { background: #388bfd; }
  input[type=""file""] { display: none; }
  select {
    background: #0d1117;
    color: #e6edf3;
    border: 1px solid #30363d;
    padding: 8px 12px;
    border-radius: 6px;
    font-size: 13.5px;
    font-weight: 600;
    min-width: 320px;
  }
  .profile-banner {
    background: #161b22;
    border-left: 4px solid #2f81f7;
    border-top: 1px solid #30363d;
    border-right: 1px solid #30363d;
    border-bottom: 1px solid #30363d;
    border-radius: 8px;
    padding: 16px 20px;
    margin-bottom: 18px;
    font-size: 15px;
    line-height: 1.5;
  }
  .dragy-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 14px;
    margin-bottom: 20px;
  }
  .dragy-card {
    background: #161b22;
    border: 1px solid #30363d;
    border-radius: 8px;
    padding: 14px 18px;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }
  .dragy-label { font-size: 12.5px; color: #8b949e; font-weight: 700; text-transform: uppercase; letter-spacing: 0.8px; }
  .dragy-val { font-family: 'Consolas', monospace; font-size: 24px; font-weight: 700; color: #3fb950; }
  .table-wrap {
    background: #ffffff;
    color: #1f2328;
    border-radius: 8px;
    overflow: hidden;
    box-shadow: 0 8px 24px rgba(0,0,0,0.4);
    margin-bottom: 20px;
  }
  table { width: 100%; border-collapse: collapse; font-size: 14.5px; }
  th {
    background: #f6f8fa;
    color: #1f2328;
    font-weight: 600;
    text-align: left;
    padding: 10px 14px;
    border: 1px solid #d0d7de;
  }
  td { padding: 9px 14px; border: 1px solid #d0d7de; }
  tr:nth-child(even) td { background: #fcfcfd; }
  .dot {
    display: inline-block;
    width: 14px;
    height: 14px;
    border-radius: 50%;
    vertical-align: middle;
    margin-right: 6px;
    border: 1px solid rgba(0,0,0,0.35);
  }
  .dot-green { background: #2da44e; }
  .dot-orange { background: #f39c12; }
  .dot-yellow { background: #d4ac0d; }
  .dot-blue { background: #1f6feb; }
  .dot-red { background: #cf222e; }
  .advisor-card {
    background: #161b22;
    border: 1px solid #30363d;
    border-radius: 10px;
    padding: 18px 22px;
    margin-bottom: 20px;
  }
  .advisor-card h3 { margin: 0 0 12px 0; font-size: 16px; color: #fff; }
  .adv-item {
    padding: 9px 12px;
    border-radius: 6px;
    margin-bottom: 8px;
    font-size: 14px;
    line-height: 1.45;
  }
  .adv-ok { background: rgba(46, 160, 67, 0.12); border-left: 4px solid #3fb950; color: #e6edf3; }
  .adv-warn { background: rgba(210, 153, 34, 0.14); border-left: 4px solid #d29922; color: #e6edf3; }
  .adv-info { background: rgba(56, 139, 253, 0.12); border-left: 4px solid #58a6ff; color: #e6edf3; }
  .chart-card {
    background: #161b22;
    border: 1px solid #30363d;
    border-radius: 10px;
    padding: 20px;
  }
  .chart-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 12px;
  }
  .chart-header h3 { margin: 0; font-size: 16px; color: #fff; }
  .legend { font-size: 13px; color: #8b949e; display: flex; gap: 16px; align-items: center; }
  .leg-hp { color: #3fb950; font-weight: 700; }
  .leg-nm { color: #58a6ff; font-weight: 700; }
  canvas { width: 100%; height: 320px; display: block; }
</style>
</head>
<body>

<div class=""container"">
  <div class=""header"">
    <h1>🏎️ FH6SmartShift — Dyno, Dragy &amp; Gearbox Tuning Inspector</h1>
    <div class=""controls"">
      <select id=""carSelect"" onchange=""forcePreview=false;renderCar(this.value)""></select>
      <button class=""btn"" onclick=""renameCurrentCar()"">✏️ Name Car</button>
      <button class=""btn"" onclick=""exportCarListJson()"">💾 CarList.json</button>
      <button class=""btn btn-blue"" onclick=""exportSingleCarProfile()"">📤 Export Car</button>
      <label class=""btn btn-green"">
        📥 Import / Merge JSON
        <input type=""file"" id=""mergeFile"" accept="".json"">
      </label>
      <button class=""btn"" id=""saveMergedBtn"" style=""display:none;border-color:#d29922;color:#e3b341;"" onclick=""downloadFullDatabase()"">💾 Save Merged DB</button>
    </div>
  </div>

  <div class=""profile-banner"" id=""profileSummary"">
    No car profiles saved yet. Drive a car at wide-open throttle in Forza Horizon 6 or Forza Motorsport to populate the Virtual Dyno database.
  </div>

  <div class=""dragy-grid"">
    <div class=""dragy-card"">
      <div>
        <div class=""dragy-label"">⏱️ 0–100 km/h (Best PB)</div>
        <div style=""font-size:12px;color:#6e7681;"">Standing Launch Telemetry</div>
      </div>
      <div class=""dragy-val"" id=""dragy0to100"">—</div>
    </div>
    <div class=""dragy-card"">
      <div>
        <div class=""dragy-label"">🚀 100–200 km/h (Best PB)</div>
        <div style=""font-size:12px;color:#6e7681;"">Rolling / Standing Pull</div>
      </div>
      <div class=""dragy-val"" id=""dragy100to200"">—</div>
    </div>
    <div class=""dragy-card"">
      <div>
        <div class=""dragy-label"">🏁 0–200 km/h (Best PB)</div>
        <div style=""font-size:12px;color:#6e7681;"">Full Standing Sprint</div>
      </div>
      <div class=""dragy-val"" id=""dragy0to200"">—</div>
    </div>
  </div>

  <div class=""table-wrap"">
    <table>
      <thead>
        <tr>
          <th>Gear</th>
          <th>Gear Ratio (RpmPerRadS)</th>
          <th>RPM Drop on N+1 Upshift (dropRatio)</th>
          <th>Next Gear Landing RPM</th>
          <th>Saved JSON Threshold (Rpm)</th>
          <th>Physical Shift Target (OptimalShiftRpm)</th>
          <th>Dashboard Status</th>
        </tr>
      </thead>
      <tbody id=""gearTableBody""></tbody>
    </table>
  </div>

  <div class=""advisor-card"">
    <h3>🔧 Automated Gearbox Tuning Advisor (Telemetry Analysis)</h3>
    <div id=""tuningAdvisorBody""></div>
  </div>

  <div class=""chart-card"">
    <div class=""chart-header"">
      <h3 id=""chartTitle"">📈 401-Band Virtual Dyno Curve (Power HP &amp; Torque Nm)</h3>
      <div class=""legend"">
        <button class=""btn"" id=""previewBtn"" style=""display:none;padding:4px 10px;font-size:11.5px;"" onclick=""forcePreview=!forcePreview;renderCar(document.getElementById('carSelect').value)"">👁️ Toggle Partial Curve</button>
        <span class=""leg-hp"">● Engine Power (HP)</span>
        <span class=""leg-nm"">● Engine Torque (Nm)</span>
      </div>
    </div>
    <canvas id=""dynoCanvas"" width=""1120"" height=""320""></canvas>
  </div>
</div>

<script>
let dbData = " + jsonPayload + @";
let defaultCarKey = " + safeActiveKey + @";
let localNames = JSON.parse(localStorage.getItem('fh6_custom_car_names') || '{}');
let forcePreview = false;

function getRawCarName(key) {
  const cleanKey = key.replace(/^FM_/, '');
  const parts = cleanKey.split('_');
  const ord = parts[0];
  if (localNames[key]) return localNames[key];
  if (localNames[ord]) return localNames[ord];
  if (dbData.CarNames && dbData.CarNames[key]) return dbData.CarNames[key];
  if (dbData.CarNames && dbData.CarNames[ord]) return dbData.CarNames[ord];
  return `Car #${ord}`;
}

function getCarName(key) {
  const gameTag = key.startsWith('FM_') ? '[FM] ' : '[FH] ';
  return gameTag + getRawCarName(key);
}

function renameCurrentCar() {
  const sel = document.getElementById('carSelect');
  const key = sel.value;
  if (!key) return;
  const current = getRawCarName(key);
  const entered = prompt(`Enter custom name for ${key} (e.g. 'Audi RS 6 Avant 2021'):`, current);
  if (entered !== null && entered.trim() !== '') {
    localNames[key] = entered.trim();
    if (!dbData.CarNames) dbData.CarNames = {};
    dbData.CarNames[key] = entered.trim();
    localStorage.setItem('fh6_custom_car_names', JSON.stringify(localNames));
    populateCarSelect(key);
  }
}

function exportCarListJson() {
  const merged = Object.assign({}, dbData.CarNames || {}, localNames);
  const blob = new Blob([JSON.stringify(merged, null, 2)], {type: 'application/json;charset=utf-8'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'FH6SmartShift_CarList.json';
  a.click();
}

function exportSingleCarProfile() {
  const key = document.getElementById('carSelect').value;
  if (!key) return;
  const singlePayload = {
     FH6SmartShiftSingleCar: true,
     CarKey: key,
     CarName: getRawCarName(key),
     GearRatios: (dbData.GearRatios && dbData.GearRatios[key]) || {},
     EngineProfile: (dbData.EngineProfiles && dbData.EngineProfiles[key]) || {},
     AdvancedGears: (dbData.AdvancedGears && dbData.AdvancedGears[key]) || {}
  };
  const blob = new Blob([JSON.stringify(singlePayload, null, 2)], {type: 'application/json;charset=utf-8'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `FH6_CarProfile_${key}.json`;
  a.click();
}

document.getElementById('mergeFile').addEventListener('change', function(e) {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = function(evt) {
    try {
      const incoming = JSON.parse(evt.target.result);
      if (!dbData.GearRatios) dbData.GearRatios = {};
      if (!dbData.EngineProfiles) dbData.EngineProfiles = {};
      if (!dbData.AdvancedGears) dbData.AdvancedGears = {};
      if (!dbData.CarNames) dbData.CarNames = {};

      let lastMergedKey = '';
      let importedCount = 0;

      if (incoming.FH6SmartShiftSingleCar && incoming.CarKey) {
        const k = incoming.CarKey;
        dbData.GearRatios[k] = incoming.GearRatios || {};
        dbData.EngineProfiles[k] = incoming.EngineProfile || {};
        dbData.AdvancedGears[k] = incoming.AdvancedGears || {};
        if (incoming.CarName) {
          dbData.CarNames[k] = incoming.CarName;
          localNames[k] = incoming.CarName;
        }
        lastMergedKey = k;
        importedCount = 1;
      } else if (incoming.EngineProfiles || incoming.GearRatios) {
        const keys = new Set([
          ...Object.keys(incoming.EngineProfiles || {}),
          ...Object.keys(incoming.GearRatios || {})
        ]);
        keys.forEach(k => {
          if (incoming.GearRatios && incoming.GearRatios[k]) dbData.GearRatios[k] = incoming.GearRatios[k];
          if (incoming.EngineProfiles && incoming.EngineProfiles[k]) dbData.EngineProfiles[k] = incoming.EngineProfiles[k];
          if (incoming.AdvancedGears && incoming.AdvancedGears[k]) dbData.AdvancedGears[k] = incoming.AdvancedGears[k];
          lastMergedKey = k;
          importedCount++;
        });
        if (incoming.CarNames) {
          Object.assign(dbData.CarNames, incoming.CarNames);
        }
      } else {
        alert('Unrecognized JSON format.');
        return;
      }

      document.getElementById('saveMergedBtn').style.display = 'inline-block';
      populateCarSelect(lastMergedKey);
      alert(`Successfully imported/merged ${importedCount} car profile(s)! Click '💾 Save Merged DB' if you want to save the combined database file.`);
    } catch (err) {
      alert('Invalid JSON file: ' + err.message);
    }
  };
  reader.readAsText(file);
  e.target.value = '';
});

function downloadFullDatabase() {
  dbData.CarNames = Object.assign({}, dbData.CarNames || {}, localNames);
  const blob = new Blob([JSON.stringify(dbData, null, 2)], {type: 'application/json;charset=utf-8'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'FH6SmartShift_fh6SmartShiftingData.json';
  a.click();
}

function populateCarSelect(forceKey) {
  const sel = document.getElementById('carSelect');
  sel.innerHTML = '';
  const allKeys = new Set([
    ...Object.keys(dbData.EngineProfiles || {}),
    ...Object.keys(dbData.GearRatios || {})
  ]);
  const keys = Array.from(allKeys).sort();

  keys.forEach(k => {
    const opt = document.createElement('option');
    opt.value = k;
    const dtMap = {'DT0':'FWD', 'DT1':'RWD', 'DT2':'AWD'};
    let label = k;
    const cleanKey = k.replace(/^FM_/, '');
    const parts = cleanKey.split('_');
    const cName = getCarName(k);
    if (parts.length === 4) {
      label = `${cName} | ${parts[1]} | ${dtMap[parts[2]] || parts[2]} | Gauge ${parts[3].replace('R','')} RPM (${k})`;
    }
    opt.textContent = label;
    sel.appendChild(opt);
  });

  if (keys.length > 0) {
    const target = (forceKey && keys.includes(forceKey)) ? forceKey : ((defaultCarKey && keys.includes(defaultCarKey)) ? defaultCarKey : keys[0]);
    sel.value = target;
    renderCar(target);
  }
}

function getSmoothedBandValue(map, rpm) {
  if (!Array.isArray(map) || rpm <= 0) return 0;
  const idx = Math.min(400, Math.max(0, Math.round(rpm / 50)));
  let sum = 0, cnt = 0;
  for (let d = -1; d <= 1; d++) {
    const k = idx + d;
    if (k >= 0 && k <= 400 && map[k] > 0) { sum += map[k]; cnt++; }
  }
  return cnt > 0 ? (sum / cnt) : 0;
}

function computeMapProgress(ep, actualLimiter) {
  if (typeof ep.MappingProgress === 'number' && ep.MappingProgress > 0) return ep.MappingProgress;
  const pMap = ep.PowerMap || [];
  const rpmP = ep.RpmAtMaxPower || 6000;
  const startRpm = Math.min(3000, Math.max(1500, rpmP * 0.45));
  const endRpm = actualLimiter - 250;
  if (endRpm <= startRpm) return 0;
  const sBand = Math.max(0, Math.floor(startRpm / 50));
  const eBand = Math.min(400, Math.ceil(endRpm / 50));
  let filled = 0;
  for (let i = sBand; i <= eBand; i++) {
    if (pMap[i] > 0) filled++;
  }
  const pct = (filled / Math.max(1, eBand - sBand + 1)) * 100.0;
  return pct >= 99.0 ? 100.0 : Math.min(100.0, pct);
}

function renderCar(key) {
  const ep = (dbData.EngineProfiles && dbData.EngineProfiles[key]) || {};
  const ratios = (dbData.GearRatios && dbData.GearRatios[key]) || {};
  const adv = (dbData.AdvancedGears && dbData.AdvancedGears[key]) || {};
  const cName = getCarName(key);

  const gaugeMatch = key.match(/_R(\d+)$/);
  const gaugeRpm = gaugeMatch ? parseInt(gaugeMatch[1], 10) : 8000;

  const kw = (ep.MaxPower || 0) / 1000.0;
  const hp = (ep.MaxPower || 0) / 745.7;
  const nm = ep.MaxTorque || 0;
  const rpmP = ep.RpmAtMaxPower || 0;
  const rpmT = ep.RpmAtMaxTorque || 0;

  let maxBoostPsi = ep.MaxBoost || 0;
  if (Array.isArray(ep.BoostMap)) {
    for (let i = 0; i < ep.BoostMap.length; i++) {
      if (ep.BoostMap[i] > maxBoostPsi) maxBoostPsi = ep.BoostMap[i];
    }
  }
  const maxBoostBar = maxBoostPsi * 0.0689476;
  const boostText = maxBoostPsi > 0.1
    ? `${maxBoostPsi.toFixed(2)} psi (~${maxBoostBar.toFixed(2)} bar) of boost`
    : `Naturally Aspirated (0.0 psi boost)`;

  let maxMappedBand = 0;
  if (Array.isArray(ep.PowerMap)) {
    for (let i = 400; i >= 0; i--) {
      if (ep.PowerMap[i] > 0) { maxMappedBand = i; break; }
    }
  }
  const actualLimiter = (ep.GhostLimiterRpm && ep.GhostLimiterRpm > 0)
    ? ep.GhostLimiterRpm
    : Math.max(gaugeRpm, maxMappedBand * 50.0);

  const mapPct = computeMapProgress(ep, actualLimiter);
  const isMap100 = mapPct >= 99.5;

  const limiterText = (ep.GhostLimiterRpm && ep.GhostLimiterRpm > 0)
    ? `and a detected hidden rev limiter (GhostLimiterRpm) at <strong>${ep.GhostLimiterRpm.toFixed(1)} RPM</strong> (on an ${gaugeRpm} RPM gauge cluster).`
    : `and standard rev limiter up to <strong>${actualLimiter.toFixed(0)} RPM</strong> (on an ${gaugeRpm} RPM gauge cluster).`;

  const mapBadge = isMap100
    ? `<span style=""color:#3fb950;font-weight:700;"">[DYNO MAP: 100% COMPLETE]</span>`
    : `<span style=""color:#e3b341;font-weight:700;"">[DYNO MAP: ${mapPct.toFixed(1)}% — INCOMPLETE]</span>`;

  document.getElementById('profileSummary').innerHTML = `
    <button class=""btn"" style=""float:right;padding:4px 10px;font-size:12px;"" onclick=""copySummary()"">📋 Copy Summary</button>
    <strong>Engine &amp; Dyno Profile — ${cName} (<code>${key}</code>) ${mapBadge}:</strong>
    Peak power of <strong>${hp.toFixed(1)} HP (${kw.toFixed(1)} kW)</strong> at <strong>${rpmP.toFixed(0)} RPM</strong>,
    peak torque of <strong>${nm.toFixed(1)} Nm</strong> at <strong>${rpmT.toFixed(0)} RPM</strong>,
    <strong>${boostText}</strong>, ${limiterText}
  `;

  document.getElementById('dragy0to100').textContent = (ep.Best0To100 && ep.Best0To100 > 0) ? ep.Best0To100.toFixed(2) + ' s' : '—';
  document.getElementById('dragy100to200').textContent = (ep.Best100To200 && ep.Best100To200 > 0) ? ep.Best100To200.toFixed(2) + ' s' : '—';
  document.getElementById('dragy0to200').textContent = (ep.Best0To200 && ep.Best0To200 > 0) ? ep.Best0To200.toFixed(2) + ' s' : '—';

  const tbody = document.getElementById('gearTableBody');
  tbody.innerHTML = '';

  const gearNums = Object.keys(ratios).map(Number).filter(n => ratios[n] > 0).sort((a,b) => a - b);
  const maxGear = gearNums.length > 0 ? Math.max(...gearNums) : 6;

  const advisorTips = [];
  let optimalTransitions = 0;

  for (let g = 1; g <= maxGear; g++) {
    const rNow = ratios[g] || 0;
    const rNext = ratios[g + 1] || 0;
    const gAdv = adv[g];

    let ratioStr = rNow > 0 ? rNow.toFixed(2) : '—';
    let dropStr = '—';
    let landStr = '—';
    let savedStr = '<em>No entry</em>';
    let physStr = `<em>To limiter (${Math.round(actualLimiter)} RPM)</em>`;
    let statusHtml = `<span class=""dot dot-red""></span>${g}: (-) <em>(Unmapped)</em>`;

    if (rNow > 0 && !rNext && g === maxGear) {
      dropStr = '— (Top Gear)';
    }

    let physTargetVal = 0;
    if (gAdv && gAdv.Rpm > 0) {
      savedStr = `${gAdv.Rpm.toFixed(1)} RPM`;
      if (gAdv.Edg) {
        physTargetVal = Math.round(actualLimiter - 250.0);
        physStr = `<strong>${physTargetVal} RPM</strong>`;
        statusHtml = `<span class=""dot dot-orange""></span>${g}: (L?) <em>(Edge / Unconfirmed)</em>`;
      } else if (gAdv.Gho) {
        physTargetVal = Math.round(gAdv.Rpm - 50.0);
        physStr = `<strong>${physTargetVal} RPM</strong>`;
        statusHtml = `<span class=""dot dot-green""></span>${g}: (L) <em>(Limiter Confirmed)</em>`;
      } else if (gAdv.Opt) {
        physTargetVal = Math.round(gAdv.Rpm);
        physStr = `<strong>${physTargetVal} RPM</strong>`;
        statusHtml = `<span class=""dot dot-green""></span>${g}: (P) <em>(Math Peak Optimal)</em>`;
      } else {
        physTargetVal = Math.round(gAdv.Rpm);
        physStr = `<strong>${physTargetVal} RPM</strong>`;
        statusHtml = `<span class=""dot dot-yellow""></span>${g}: (P?) <em>(Unconfirmed Trigger)</em>`;
      }
    } else if (rNow > 0) {
      statusHtml = `<span class=""dot dot-blue""></span>${g}: (R) <em>(Ratio Locked)</em>`;
    }

    if (rNow > 0 && rNext > 0) {
      const dropRatio = rNext / rNow;
      const dropPct = ((dropRatio - 1.0) * 100.0).toFixed(1);
      dropStr = `${dropRatio.toFixed(3)} (${dropPct}%)`;
      const refShift = physTargetVal > 0 ? physTargetVal : actualLimiter;
      const landRpm = Math.round(refShift * dropRatio);
      landStr = `~${landRpm} RPM`;

      if (isMap100) {
        const pAtLand = getSmoothedBandValue(ep.PowerMap, landRpm);
        const bAtLand = getSmoothedBandValue(ep.BoostMap, landRpm);

        if (rpmT > 2500 && landRpm < rpmT * 0.90 && g >= 1) {
          advisorTips.push({
            type: 'warn',
            text: `⚠️ <strong>Gear ${g} → ${g+1} Too Wide (${dropPct}% drop):</strong> Landing at <strong>~${landRpm} RPM</strong> drops below Peak Torque (${rpmT.toFixed(0)} RPM). Shorten <strong>Gear ${g+1}</strong> (move slider toward Acceleration) so it lands closer to ${Math.round(rpmT)} RPM.`
          });
        } else if (ep.MaxPower > 0 && pAtLand > 0 && pAtLand < ep.MaxPower * 0.78 && g >= 2) {
          const pLossPct = ((1.0 - (pAtLand / ep.MaxPower)) * 100).toFixed(1);
          advisorTips.push({
            type: 'warn',
            text: `⚠️ <strong>Gear ${g} → ${g+1} Power Drop (-${pLossPct}% vs Peak):</strong> Landing at <strong>~${landRpm} RPM</strong> leaves the engine with only ${(pAtLand/745.7).toFixed(0)} HP. Consider tightening the gap between Gear ${g} and Gear ${g+1}.`
          });
        } else if (g <= 4 && dropRatio > 0.895) {
          advisorTips.push({
            type: 'info',
            text: `ℹ️ <strong>Gear ${g} → ${g+1} Very Close Ratio (${dropPct}% drop):</strong> Tight spacing in lower gears increases shift frequency. You can lengthen Gear ${g+1} slightly unless this is a dedicated 8–10 speed close-ratio build.`
          });
        } else {
          optimalTransitions++;
        }

        if (maxBoostPsi > 2.5 && bAtLand > 0 && bAtLand < maxBoostPsi * 0.72) {
          advisorTips.push({
            type: 'warn',
            text: `🐌 <strong>Turbo Lag Risk on Gear ${g} → ${g+1}:</strong> Boost pressure at ~${landRpm} RPM is only <strong>${bAtLand.toFixed(1)} psi</strong> (vs ${maxBoostPsi.toFixed(1)} psi peak). Shorten Gear ${g+1} to keep the turbo spooled!`
          });
        }
      }
    }

    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td><strong>Gear ${g}</strong></td>
      <td>${ratioStr}</td>
      <td>${dropStr}</td>
      <td>${landStr}</td>
      <td>${savedStr}</td>
      <td>${physStr}</td>
      <td>${statusHtml}</td>
    `;
    tbody.appendChild(tr);
  }

  const advContainer = document.getElementById('tuningAdvisorBody');
  advContainer.innerHTML = '';

  if (!isMap100) {
    const waitDiv = document.createElement('div');
    waitDiv.className = 'adv-item adv-warn';
    waitDiv.innerHTML = `⏳ <strong>Waiting for 100% Dyno Map (${mapPct.toFixed(1)}% Completed):</strong> During standing launches (0–200 km/h), gears 2–6 land above ~5,000 RPM and skip the mid-range (3,000–4,900 RPM). Do one clean wide-open throttle pull from <strong>~2,500 RPM in 3rd or 4th gear</strong> to reach 100% map completion and unlock verified Gearbox Tuning Advisor recommendations.`;
    advContainer.appendChild(waitDiv);
  } else {
    if (optimalTransitions > 0) {
      const okDiv = document.createElement('div');
      okDiv.className = 'adv-item adv-ok';
      okDiv.innerHTML = `✅ <strong>Powerband Alignment:</strong> <strong>${optimalTransitions}</strong> gear transition(s) land squarely inside the optimal power window between Peak Torque (${rpmT.toFixed(0)} RPM) and Peak Power (${rpmP.toFixed(0)} RPM).`;
      advContainer.appendChild(okDiv);
    }

    if (rpmP > 0 && actualLimiter - rpmP > 750) {
      const overRevDiv = document.createElement('div');
      overRevDiv.className = 'adv-item adv-info';
      overRevDiv.innerHTML = `💡 <strong>Early Peak Power Characteristic:</strong> Peak power occurs at <strong>${rpmP.toFixed(0)} RPM</strong> (${Math.round(actualLimiter - rpmP)} RPM before the ${actualLimiter.toFixed(0)} RPM limiter). Shifting at the green <strong>(P)</strong> targets instead of redline yields maximum wheel thrust.`;
      advContainer.appendChild(overRevDiv);
    }

    advisorTips.forEach(tip => {
      const d = document.createElement('div');
      d.className = 'adv-item ' + (tip.type === 'warn' ? 'adv-warn' : 'adv-info');
      d.innerHTML = tip.text;
      advContainer.appendChild(d);
    });
  }

  document.getElementById('previewBtn').style.display = isMap100 ? 'none' : 'inline-block';
  drawDynoChart(ep, isMap100, mapPct, actualLimiter);
}

function drawDynoChart(ep, isMap100, mapPct, actualLimiter) {
  const canvas = document.getElementById('dynoCanvas');
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  if (!isMap100 && !forcePreview) {
    ctx.fillStyle = '#161b22';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = '#30363d';
    ctx.lineWidth = 2;
    ctx.strokeRect(20, 20, canvas.width - 40, canvas.height - 40);

    ctx.fillStyle = '#e3b341';
    ctx.font = 'bold 20px Segoe UI';
    ctx.textAlign = 'center';
    ctx.fillText(`⏳ VIRTUAL DYNO CURVE LOCKED UNTIL 100% MAP COMPLETION (CURRENT: ${mapPct.toFixed(1)}%)`, canvas.width / 2, 115);

    ctx.fillStyle = '#8b949e';
    ctx.font = '14.5px Segoe UI';
    ctx.fillText('Standing 0–200 km/h launches spin 1st gear and land above ~5,000 RPM in gears 2–6, leaving 3,000–4,900 RPM unmapped.', canvas.width / 2, 155);
    ctx.fillStyle = '#e6edf3';
    ctx.fillText('👉 Pull cleanly from ~2,500 RPM to Redline in 3rd or 4th gear at 100% throttle to complete the map (or click Toggle Partial Curve above).', canvas.width / 2, 188);

    const barW = 520, barH = 18;
    const barX = (canvas.width - barW) / 2, barY = 220;
    ctx.fillStyle = '#21262d';
    ctx.fillRect(barX, barY, barW, barH);
    ctx.fillStyle = '#238636';
    ctx.fillRect(barX, barY, Math.max(8, (mapPct / 100.0) * barW), barH);
    ctx.strokeStyle = '#30363d';
    ctx.strokeRect(barX, barY, barW, barH);
    ctx.textAlign = 'start';
    return;
  }

  const pMap = ep.PowerMap || [];
  const tMap = ep.TorqueMap || [];
  let maxBand = 0;
  for (let i = 400; i >= 0; i--) { if (pMap[i] > 0) { maxBand = i; break; } }

  if (maxBand === 0) return;

  const padL = 55, padR = 55, padT = 20, padB = 35;
  const w = canvas.width - padL - padR;
  const h = canvas.height - padT - padB;
  const maxRpmPlot = Math.ceil((maxBand * 50 + 500) / 1000) * 1000;
  const maxHp = Math.max(100, ((ep.MaxPower || 100000) / 745.7) * 1.12);
  const maxNm = Math.max(100, (ep.MaxTorque || 200) * 1.12);

  ctx.strokeStyle = '#21262d';
  ctx.lineWidth = 1;
  for (let r = 1000; r <= maxRpmPlot; r += 1000) {
    const x = padL + (r / maxRpmPlot) * w;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + h); ctx.stroke();
    ctx.fillStyle = '#8b949e'; ctx.font = '12px Consolas';
    ctx.fillText(r + ' RPM', x - 24, padT + h + 22);
  }

  // Smoothed Torque Curve (No fake straight-line bridging across >150 RPM unmapped gaps)
  ctx.strokeStyle = '#58a6ff';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  let lastValidBandT = -10;
  for (let i = 1; i <= maxBand; i++) {
    if (tMap[i] <= 0) continue;
    const valT = getSmoothedBandValue(tMap, i * 50);
    const x = padL + ((i * 50) / maxRpmPlot) * w;
    const y = padT + h - (valT / maxNm) * h;
    if (i - lastValidBandT > 3) { ctx.moveTo(x, y); } else { ctx.lineTo(x, y); }
    lastValidBandT = i;
  }
  ctx.stroke();

  // Smoothed Power Curve (No fake straight-line bridging across >150 RPM unmapped gaps)
  ctx.strokeStyle = '#3fb950';
  ctx.lineWidth = 3;
  ctx.beginPath();
  let lastValidBandP = -10;
  for (let i = 1; i <= maxBand; i++) {
    if (pMap[i] <= 0) continue;
    const valP = getSmoothedBandValue(pMap, i * 50) / 745.7;
    const x = padL + ((i * 50) / maxRpmPlot) * w;
    const y = padT + h - (valP / maxHp) * h;
    if (i - lastValidBandP > 3) { ctx.moveTo(x, y); } else { ctx.lineTo(x, y); }
    lastValidBandP = i;
  }
  ctx.stroke();
}

function copySummary() {
  const txt = document.getElementById('profileSummary').innerText.replace('📋 Copy Summary', '').trim();
  navigator.clipboard.writeText(txt);
}

populateCarSelect();
</script>
</body>
</html>";

            File.WriteAllText(reportPath, html, Encoding.UTF8);
        }

        private CarStore CloneCarStore(CarStore source)
        {
            var clone = new CarStore
            {
                SchemaVersion = source.SchemaVersion,
                ShiftBeepEnabled = source.ShiftBeepEnabled,
                ReactionBufferFactor = source.ReactionBufferFactor,
                CarNames = source.CarNames != null ? new Dictionary<string, string>(source.CarNames) : new Dictionary<string, string>()
            };
            foreach (var ratio in source.GearRatios) clone.GearRatios[ratio.Key] = new Dictionary<int, double>(ratio.Value);
            foreach (var ag in source.AdvancedGears) 
            {
                clone.AdvancedGears[ag.Key] = new Dictionary<int, SavedGearData>();
                foreach (var g in ag.Value) clone.AdvancedGears[ag.Key][g.Key] = new SavedGearData { Rpm = g.Value.Rpm, Opt = g.Value.Opt, Edg = g.Value.Edg, Gho = g.Value.Gho };
            }
            foreach (var engine in source.EngineProfiles) 
            {
                var mapCopy = new double[401]; var torqueCopy = new double[401]; var boostCopy = new double[401];
                if (engine.Value.PowerMap != null && engine.Value.PowerMap.Length == 401) Array.Copy(engine.Value.PowerMap, mapCopy, 401);
                if (engine.Value.TorqueMap != null && engine.Value.TorqueMap.Length == 401) Array.Copy(engine.Value.TorqueMap, torqueCopy, 401);
                if (engine.Value.BoostMap != null && engine.Value.BoostMap.Length == 401) Array.Copy(engine.Value.BoostMap, boostCopy, 401);
                
                clone.EngineProfiles[engine.Key] = new SavedEngineProfile { 
                    PowerMap = mapCopy, TorqueMap = torqueCopy, BoostMap = boostCopy,
                    MaxPower = engine.Value.MaxPower, RpmAtMaxPower = engine.Value.RpmAtMaxPower,
                    MaxTorque = engine.Value.MaxTorque, RpmAtMaxTorque = engine.Value.RpmAtMaxTorque,
                    MaxBoost = engine.Value.MaxBoost,
                    GhostLimiterRpm = engine.Value.GhostLimiterRpm,
                    MappingProgress = engine.Value.MappingProgress,
                    Best0To100 = engine.Value.Best0To100,
                    Best100To200 = engine.Value.Best100To200,
                    Best0To200 = engine.Value.Best0To200
                };
            }
            return clone;
        }

        private void UpdateSnapshotWithCurrentCar()
        {
            if (string.IsNullOrEmpty(_key)) return;
            _snapshotStore.ShiftBeepEnabled = _store.ShiftBeepEnabled;
            _snapshotStore.ReactionBufferFactor = _store.ReactionBufferFactor;
            if (_store.CarNames != null)
            {
                foreach (var kvp in _store.CarNames) _snapshotStore.CarNames[kvp.Key] = kvp.Value;
            }
            
            if (_store.GearRatios.TryGetValue(_key, out var ratioDict)) 
                _snapshotStore.GearRatios[_key] = new Dictionary<int, double>(ratioDict);

            if (_store.AdvancedGears.TryGetValue(_key, out var advDict)) 
            {
                _snapshotStore.AdvancedGears[_key] = new Dictionary<int, SavedGearData>();
                foreach (var kvp in advDict) _snapshotStore.AdvancedGears[_key][kvp.Key] = new SavedGearData { Rpm = kvp.Value.Rpm, Opt = kvp.Value.Opt, Edg = kvp.Value.Edg, Gho = kvp.Value.Gho };
            }
                
            if (_store.EngineProfiles.TryGetValue(_key, out var ep)) 
            {
                var pMap = new double[401]; 
                if (ep.PowerMap != null && ep.PowerMap.Length == 401) Array.Copy(ep.PowerMap, pMap, 401);
                var tMap = new double[401]; 
                if (ep.TorqueMap != null && ep.TorqueMap.Length == 401) Array.Copy(ep.TorqueMap, tMap, 401);
                var bMap = new double[401]; 
                if (ep.BoostMap != null && ep.BoostMap.Length == 401) Array.Copy(ep.BoostMap, bMap, 401);
                
                _snapshotStore.EngineProfiles[_key] = new SavedEngineProfile {
                    PowerMap = pMap, TorqueMap = tMap, BoostMap = bMap,
                    MaxPower = ep.MaxPower, RpmAtMaxPower = ep.RpmAtMaxPower,
                    MaxTorque = ep.MaxTorque, RpmAtMaxTorque = ep.RpmAtMaxTorque,
                    MaxBoost = ep.MaxBoost,
                    GhostLimiterRpm = ep.GhostLimiterRpm,
                    MappingProgress = ep.MappingProgress,
                    Best0To100 = ep.Best0To100,
                    Best100To200 = ep.Best100To200,
                    Best0To200 = ep.Best0To200
                };
            }
        }

        private void TriggerAsyncSave()
        {
            CarStore toSave = null;
            string currentKeyCopy = "";
            lock (_lock)
            {
                if (!string.IsNullOrEmpty(_key))
                {
                    SaveEngineProfileToStore();
                    UpdateSnapshotWithCurrentCar();
                }
                
                _lastSaveTime = _appTime.Elapsed.TotalSeconds;
                if (Interlocked.CompareExchange(ref _saveInFlight, 1, 0) == 0)
                {
                    toSave = CloneCarStore(_snapshotStore);
                    currentKeyCopy = _key ?? "";
                    _isDataDirty = false;
                }
            }
            if (toSave != null)
            {
                Task.Run(() => { 
                    try 
                    { 
                        this.SaveCommonSettings(StoreKey, toSave); 
                        WriteHtmlGarageReport(toSave, currentKeyCopy);
                    } 
                    catch (Exception ex) 
                    { 
                        Debug.WriteLine("FH6SmartShift save failed: " + ex); 
                        lock (_lock) { _isDataDirty = true; } 
                    } 
                    finally { Interlocked.Exchange(ref _saveInFlight, 0); } 
                });
            }
        }

        private void SaveEngineProfileToStore()
        {
            if (string.IsNullOrEmpty(_key) || _globalEngine == null) return;
            if (_globalEngine.MaxPower == 0 && _globalEngine.Best0To100 == 0 && _globalEngine.Best100To200 == 0) return;

            var profileCopy = new double[401]; var torqueCopy = new double[401]; var boostCopy = new double[401];
            Array.Copy(_globalEngine.PowerMap, profileCopy, 401); 
            Array.Copy(_globalEngine.TorqueMap, torqueCopy, 401);
            Array.Copy(_globalEngine.BoostMap, boostCopy, 401);
            _store.EngineProfiles[_key] = new SavedEngineProfile
            {
                PowerMap = profileCopy, TorqueMap = torqueCopy, BoostMap = boostCopy,
                MaxPower = _globalEngine.MaxPower, RpmAtMaxPower = _globalEngine.RpmAtMaxPower,
                MaxTorque = _globalEngine.MaxTorque, RpmAtMaxTorque = _globalEngine.RpmAtMaxTorque,
                MaxBoost = _globalEngine.MaxBoost,
                GhostLimiterRpm = _globalEngine.GhostLimiterRpm,
                MappingProgress = _globalEngine.MappingProgress,
                Best0To100 = _globalEngine.Best0To100,
                Best100To200 = _globalEngine.Best100To200,
                Best0To200 = _globalEngine.Best0To200
            };
        }

        private void SaveGearTrigger(int gear, GearState gs)
        {
            if (string.IsNullOrEmpty(_key) || gs.ShiftTriggerRpm <= 0) return;
            
            if (!_store.AdvancedGears.ContainsKey(_key)) _store.AdvancedGears[_key] = new Dictionary<int, SavedGearData>();

            bool needsSave = false;
            if (!_store.AdvancedGears[_key].TryGetValue(gear, out var existing)) {
                needsSave = true;
            } else {
                if (existing.Opt != gs.IsMathOptimal || existing.Edg != gs.TriggerIsEdge || existing.Gho != gs.TriggerIsGhost || Math.Abs(existing.Rpm - gs.ShiftTriggerRpm) > 25) {
                    needsSave = true;
                }
            }

            if (needsSave) {
                _store.AdvancedGears[_key][gear] = new SavedGearData { Rpm = gs.ShiftTriggerRpm, Opt = gs.IsMathOptimal, Edg = gs.TriggerIsEdge, Gho = gs.TriggerIsGhost };
                _isDataDirty = true;
            }
        }

        private double GetRealShiftRpm(GearState gs, double actualLimiterLocal)
        {
            if (gs == null || gs.ShiftTriggerRpm <= 0) return 0.0;
            return gs.TriggerIsEdge ? actualLimiterLocal - 250.0 : gs.ShiftTriggerRpm - (gs.TriggerIsGhost ? 50.0 : 0.0);
        }

        private double GetInterpolatedValue(double targetRpm, double[] map)
        {
            if (targetRpm < 0 || targetRpm > 20000) return 0;
            int targetBand = (int)Math.Floor(targetRpm / 50.0);
            if (targetBand < 0 || targetBand > 400) return 0;

            int bestLower = -1;
            for (int i = targetBand; i >= Math.Max(0, targetBand - 4); i--) { if (map[i] > 0) { bestLower = i; break; } }
            
            int bestUpper = -1;
            for (int i = targetBand + 1; i <= Math.Min(400, targetBand + 4); i++) { if (map[i] > 0) { bestUpper = i; break; } }

            if (bestLower != -1 && bestUpper != -1 && (bestUpper - bestLower) <= 4)
            {
                double p0 = map[bestLower]; double p1 = map[bestUpper];
                double r0 = bestLower * 50.0; double r1 = bestUpper * 50.0;
                double t = (targetRpm - r0) / (r1 - r0);
                return p0 + t * (p1 - p0);
            }
            else if (bestLower != -1 && targetBand - bestLower <= 1) return map[bestLower];
            else if (bestUpper != -1 && bestUpper - targetBand <= 1) return map[bestUpper];
            
            return 0;
        }

        private double GetSmoothedValue(double targetRpm, double[] map)
        {
            double p1 = GetInterpolatedValue(targetRpm - 50, map);
            double p2 = GetInterpolatedValue(targetRpm, map);
            double p3 = GetInterpolatedValue(targetRpm + 50, map);
            
            double sum = 0; int count = 0;
            if (p1 > 0) { sum += p1; count++; }
            if (p2 > 0) { sum += p2; count++; }
            if (p3 > 0) { sum += p3; count++; }
            
            if (count > 0) return sum / count;
            return 0;
        }

        private double GetPrecomputedPower(double r)
        {
            int idx = (int)Math.Floor(r / 50.0);
            if (idx < 0) return 0;
            if (idx > 400) return _precomputedPower[400];
            return _precomputedPower[idx];
        }

        private bool HealEngineMapPockets()
        {
            bool healed = false;
            int minB = Math.Max(2, _globalEngine.MinMappedBand);
            int maxB = Math.Min(398, _globalEngine.MaxMappedBand - 2);

            for (int i = minB; i <= maxB; i++)
            {
                double leftP = _globalEngine.PowerMap[i - 1];
                double rightP = _globalEngine.PowerMap[i + 1];
                if (leftP > 10000.0 && rightP > 10000.0)
                {
                    double avgNeighbors = 0.5 * (leftP + rightP);
                    if (_globalEngine.PowerMap[i] == 0 || _globalEngine.PowerMap[i] < avgNeighbors * 0.88)
                    {
                        _globalEngine.PowerMap[i] = avgNeighbors;
                        _globalEngine.TorqueMap[i] = 0.5 * (_globalEngine.TorqueMap[i - 1] + _globalEngine.TorqueMap[i + 1]);
                        _globalEngine.BoostMap[i] = Math.Max(_globalEngine.BoostMap[i], 0.5 * (_globalEngine.BoostMap[i - 1] + _globalEngine.BoostMap[i + 1]));
                        if (_globalEngine.BandSamples[i] == 0) _globalEngine.BandSamples[i] = 2;
                        healed = true;
                    }
                }

                if (i + 2 <= maxB)
                {
                    double l2 = _globalEngine.PowerMap[i - 1];
                    double r2 = _globalEngine.PowerMap[i + 2];
                    if (l2 > 10000.0 && r2 > 10000.0)
                    {
                        double expected1 = l2 + (r2 - l2) * (1.0 / 3.0);
                        double expected2 = l2 + (r2 - l2) * (2.0 / 3.0);
                        if ((_globalEngine.PowerMap[i] == 0 || _globalEngine.PowerMap[i] < expected1 * 0.86) &&
                            (_globalEngine.PowerMap[i + 1] == 0 || _globalEngine.PowerMap[i + 1] < expected2 * 0.86))
                        {
                            _globalEngine.PowerMap[i] = expected1;
                            _globalEngine.PowerMap[i + 1] = expected2;
                            double tl = _globalEngine.TorqueMap[i - 1];
                            double tr = _globalEngine.TorqueMap[i + 2];
                            if (tl > 0 && tr > 0)
                            {
                                _globalEngine.TorqueMap[i] = tl + (tr - tl) * (1.0 / 3.0);
                                _globalEngine.TorqueMap[i + 1] = tl + (tr - tl) * (2.0 / 3.0);
                            }
                            if (_globalEngine.BandSamples[i] == 0) _globalEngine.BandSamples[i] = 2;
                            if (_globalEngine.BandSamples[i + 1] == 0) _globalEngine.BandSamples[i + 1] = 2;
                            healed = true;
                        }
                    }
                }
            }
            return healed;
        }

        private bool IsRatioMonotonicAcrossAllGears(int targetGear, double candidateRatio)
        {
            if (candidateRatio <= 4.0 || candidateRatio > 650.0) return false;

            foreach (var kvp in _gearStates)
            {
                int otherGear = kvp.Key;
                double otherRatio = kvp.Value.SavedRpmPerRadS;
                if (otherGear == targetGear || otherRatio <= 0) continue;

                int gearDiff = Math.Abs(targetGear - otherGear);
                double minStepFactor = Math.Pow(1.022, gearDiff);

                if (otherGear < targetGear)
                {
                    if (candidateRatio * minStepFactor >= otherRatio) return false;
                }
                else
                {
                    if (candidateRatio <= otherRatio * minStepFactor) return false;
                }
            }
            return true;
        }

        // Wykrywa i usuwa anomalie przełożeń powstałe na uślizgu (np. Gear 4 = 36.05 zamiast 43.32 w dyno3.jpg)
        private void SanitizeGearProgression()
        {
            for (int g = 2; g <= 9; g++)
            {
                if (_gearStates.TryGetValue(g - 1, out var prev) && prev.SavedRpmPerRadS > 0 &&
                    _gearStates.TryGetValue(g, out var curr) && curr.SavedRpmPerRadS > 0 &&
                    _gearStates.TryGetValue(g + 1, out var next) && next.SavedRpmPerRadS > 0)
                {
                    double dropPrevToCurr = curr.SavedRpmPerRadS / prev.SavedRpmPerRadS;
                    double dropCurrToNext = next.SavedRpmPerRadS / curr.SavedRpmPerRadS;

                    // Jeśli spadek na biegu g jest nienaturalnie ogromny (< 0.635), a na kolejnym biegu g+1 nienaturalnie płytki (> 0.875)
                    if (dropPrevToCurr < 0.635 && dropCurrToNext > 0.875 && (dropCurrToNext - dropPrevToCurr) > 0.24)
                    {
                        curr.SavedRpmPerRadS = 0;
                        curr.RpmPerRadS = 0;
                        curr.ShiftTriggerRpm = 0;
                        curr.IsConfirmed = false;
                        curr.IsMathOptimal = false;
                        curr.TriggerIsEdge = false;
                        curr.TriggerIsGhost = false;
                        curr.RatioSamples = 0;
                        if (_store.GearRatios.ContainsKey(_key)) _store.GearRatios[_key].Remove(g);
                        if (_store.AdvancedGears.ContainsKey(_key)) _store.AdvancedGears[_key].Remove(g);
                        _isDataDirty = true;
                        _mathNeedsEval = true;
                    }
                }
            }
        }

        private void CleanUpContaminatedBands(double ghostRpm)
        {
            int ghostBand = (int)Math.Floor((ghostRpm - 25.0) / 50.0);
            int scanFrom = Math.Max(1, (int)Math.Floor((ghostRpm - 250.0) / 50.0));
            
            for(int i = Math.Max(0, ghostBand); i <= 400; i++) {
                _globalEngine.PowerMap[i] = 0;
                _globalEngine.TorqueMap[i] = 0;
                _globalEngine.BoostMap[i] = 0;
                _globalEngine.BandSamples[i] = 0;
            }
            for(int i = scanFrom; i < ghostBand; i++) {
                if (i + 1 <= 400 && _globalEngine.PowerMap[i] > 0 && _globalEngine.PowerMap[i-1] > 0 && _globalEngine.PowerMap[i+1] > 0) {
                    if (_globalEngine.PowerMap[i] < _globalEngine.PowerMap[i-1] * 0.85 && 
                        _globalEngine.PowerMap[i+1] < _globalEngine.PowerMap[i-1] * 0.85) {
                        for(int j = i; j <= 400; j++) {
                            _globalEngine.PowerMap[j] = 0;
                            _globalEngine.TorqueMap[j] = 0;
                            _globalEngine.BoostMap[j] = 0;
                            _globalEngine.BandSamples[j] = 0;
                        }
                        break;
                    }
                }
            }
            int newMaxBand = 0;
            for(int i = 400; i >= 0; i--) {
                if (_globalEngine.PowerMap[i] > 0) { newMaxBand = i; break; }
            }
            _globalEngine.MaxMappedBand = newMaxBand;
        }

        private bool IsRangeDense(double fromRpm, double toRpm, double minFill = 0.78)
        {
            int a = Math.Max(0, (int)Math.Floor(fromRpm / 50.0));
            int b = Math.Min(400, (int)Math.Ceiling(toRpm / 50.0));
            if (b <= a) return false;
            int ok = 0;
            for (int i = a; i <= b; i++)
            {
                if (_globalEngine.BandSamples[i] > 0 || GetPrecomputedPower(i * 50.0) > 0) ok++;
            }
            return (double)ok / (b - a + 1) >= minFill;
        }

        private void EvaluateAllMathIntersections(double throttle, int currentGear, double engineMaxRpm, double currentRpm, double actualLimiterLocal)
        {
            HealEngineMapPockets();
            SanitizeGearProgression();
            Array.Clear(_precomputedPower, 0, _precomputedPower.Length);

            _globalEngine.MaxPower = 0;
            _globalEngine.MaxTorque = 0;
            for (int i = Math.Max(0, _globalEngine.MinMappedBand); i <= Math.Min(400, _globalEngine.MaxMappedBand); i++)
            {
                if (_globalEngine.BandSamples[i] == 0 && _globalEngine.PowerMap[i] <= 0) continue;
                double smoothedP = GetSmoothedValue(i * 50.0, _globalEngine.PowerMap);
                double smoothedT = GetSmoothedValue(i * 50.0, _globalEngine.TorqueMap);
                if (smoothedP > _globalEngine.MaxPower) { _globalEngine.MaxPower = smoothedP; _globalEngine.RpmAtMaxPower = i * 50.0; }
                if (smoothedT > _globalEngine.MaxTorque) { _globalEngine.MaxTorque = smoothedT; _globalEngine.RpmAtMaxTorque = i * 50.0; }
            }

            if (_globalEngine.MaxPower <= 0) return;

            double scanLimit = actualLimiterLocal + 30;

            int minBound = Math.Max(0, _globalEngine.MinMappedBand - 2);
            int maxBound = Math.Min(400, _globalEngine.MaxMappedBand + 2);

            for (int i = minBound; i <= maxBound; i++) 
            {
                _precomputedPower[i] = GetSmoothedValue(i * 50.0, _globalEngine.PowerMap);
            }

            double startRpm = Math.Max(1000, _globalEngine.RpmAtMaxPower * 0.55);
            double denseUpperLimit = actualLimiterLocal - 250.0;
            bool isUpperPowerBandDense = IsRangeDense(Math.Max(startRpm, _globalEngine.RpmAtMaxPower * 0.72), denseUpperLimit, 0.78);

            bool hasTurbo = _globalEngine.MaxBoost > 1.5;

            for (int g = 1; g <= 10; g++)
            {
                if (_gearStates.TryGetValue(g, out var gs) && gs.RpmPerRadS > 0)
                {
                    bool hasNextGear = _gearStates.TryGetValue(g + 1, out var nextGs) && nextGs.RpmPerRadS > 0;
                    
                    bool hasHigherGear = false;
                    foreach (var k in _gearStates) if (k.Key > g && k.Value.RpmPerRadS > 0) { hasHigherGear = true; break; }

                    if (hasNextGear)
                    {
                        double dropRatio = nextGs.RpmPerRadS / gs.RpmPerRadS;
                        if (dropRatio >= 1.0) continue; 

                        double validCeiling = 0;

                        for (double r = startRpm; r <= scanLimit - 30; r += 25)
                        {
                            if (GetPrecomputedPower(r) > 0) validCeiling = r;
                        }

                        if (validCeiling == 0) continue;

                        bool foundIntersection = false;
                        int consecutiveIntersections = 0;
                        double firstCross = 0;
                        
                        for (double r = startRpm; r <= validCeiling; r += 25)
                        {
                            double pNow = GetPrecomputedPower(r);
                            double targetLandRpm = r * dropRatio;
                            double pNext = GetPrecomputedPower(targetLandRpm);
                            
                            if (hasTurbo) {
                                double expectedBoostLand = GetInterpolatedValue(targetLandRpm, _globalEngine.BoostMap);
                                double maxBoostAtCurrent = GetInterpolatedValue(r, _globalEngine.BoostMap);
                                if (maxBoostAtCurrent > 0 && expectedBoostLand > 0 && expectedBoostLand < maxBoostAtCurrent * 0.7) {
                                    pNext *= 0.85; 
                                }
                            }
                            
                            if (pNow > 0 && pNext > 0 && pNext >= pNow * 1.015 && r >= _globalEngine.RpmAtMaxPower) 
                            { 
                                if (consecutiveIntersections == 0) firstCross = r;
                                consecutiveIntersections++;
                                if (consecutiveIntersections >= 5 || (consecutiveIntersections >= 2 && r >= validCeiling - 25))
                                {
                                    foundIntersection = true; 
                                    validCeiling = firstCross; 
                                    break; 
                                }
                            }
                            else
                            {
                                consecutiveIntersections = 0;
                            }
                        }

                        double activeZoneStart = Math.Max(startRpm, validCeiling * dropRatio);
                        bool isGearActiveZoneDense = IsRangeDense(activeZoneStart, Math.Min(validCeiling, denseUpperLimit), 0.78);
                        bool hitLimiterCeiling = validCeiling >= actualLimiterLocal - 300.0 && (_globalEngine.GhostLimiterRpm > 0 || isUpperPowerBandDense || isGearActiveZoneDense);
                        bool gearDense = foundIntersection && IsRangeDense(firstCross * dropRatio, Math.Min(validCeiling, actualLimiterLocal - 200.0), 0.78);
                        
                        if (foundIntersection && gearDense)
                        {
                            gs.IsConfirmed = true;
                            gs.TriggerIsEdge = false;
                            gs.IsMathOptimal = true;
                            gs.TriggerIsGhost = false;
                            gs.ShiftTriggerRpm = validCeiling;
                        }
                        else if (hitLimiterCeiling)
                        {
                            gs.IsConfirmed = true;
                            gs.TriggerIsEdge = false;
                            gs.IsMathOptimal = false;
                            gs.TriggerIsGhost = true;
                            gs.ShiftTriggerRpm = actualLimiterLocal - 30.0;
                        }
                        else if (isUpperPowerBandDense || (!gs.IsMathOptimal && !gs.TriggerIsGhost))
                        {
                            gs.IsConfirmed = false;
                            gs.TriggerIsEdge = true;
                            gs.IsMathOptimal = false;
                            gs.TriggerIsGhost = false;
                            gs.ShiftTriggerRpm = Math.Min(actualLimiterLocal - 50.0, validCeiling + 100.0);
                        }
                        
                        gs.PowerPeakCaught = true;
                        SaveGearTrigger(g, gs);
                    }
                    else if (!hasHigherGear)
                    {
                        bool prevGearConfirmed = g > 1 && _gearStates.TryGetValue(g - 1, out var prevG) && prevG.IsConfirmed;
                        bool reachedHighRpm = throttle >= LearnWotThreshold && currentGear == g && currentRpm >= actualLimiterLocal - 120.0;

                        if (reachedHighRpm || (g >= 4 && prevGearConfirmed && gs.SavedRpmPerRadS > 0 && isUpperPowerBandDense))
                        {
                            gs.ShiftTriggerRpm = actualLimiterLocal - 30.0;
                            gs.PowerPeakCaught = true;
                            gs.IsConfirmed = true;
                            gs.IsMathOptimal = false;
                            gs.TriggerIsGhost = true;
                            gs.TriggerIsEdge = false;
                            SaveGearTrigger(g, gs);
                        }
                    }
                }
            }
        }

        private double GetDynamicReactionBuffer(double rpmAccel, int currentGear)
        {
            double factor = (_store != null && _store.ReactionBufferFactor >= 0.10) ? _store.ReactionBufferFactor : 0.25;
            double buffer = rpmAccel * factor; 
            if (buffer < 50.0) return 50.0;
            if (currentGear <= 2) { if (buffer > 1500.0) return 1500.0; }
            else { if (buffer > 600.0) return 600.0; }
            return buffer; 
        }

        private void UpdateDragyTimers(double speedKmh, double now, double accel, double brake, bool isGrounded, int currentGear)
        {
            if (currentGear < 1 || !isGrounded || brake > 40.0)
            {
                if (speedKmh < 1.5)
                {
                    _dragyArmedFromStop = true;
                    _dragyRunningFromStop = false;
                    _dragyPassed100FromStop = false;
                    _out.DragyStatus = "READY (0 KM/H)";
                }
                else
                {
                    _dragyRunningFromStop = false;
                    _dragyRunning100To200 = false;
                }
                _prevSpeedKmhDragy = speedKmh;
                _prevTimeDragy = now;
                return;
            }

            double dt = now - _prevTimeDragy;
            if (dt <= 0.0001 || dt > 0.25)
            {
                _prevSpeedKmhDragy = speedKmh;
                _prevTimeDragy = now;
                return;
            }

            if (speedKmh < 1.5)
            {
                _dragyArmedFromStop = true;
                _dragyRunningFromStop = false;
                _dragyPassed100FromStop = false;
                _dragyRunning100To200 = false;
                _out.DragyStatus = "READY (LAUNCH)";
            }

            if (_dragyArmedFromStop && !_dragyRunningFromStop && _prevSpeedKmhDragy < 1.5 && speedKmh >= 1.5 && accel >= 80.0)
            {
                double frac = (1.5 - _prevSpeedKmhDragy) / Math.Max(0.01, speedKmh - _prevSpeedKmhDragy);
                _dragyStart0Time = _prevTimeDragy + (Math.Max(0.0, Math.Min(1.0, frac)) * dt);
                _dragyRunningFromStop = true;
                _dragyArmedFromStop = false;
                _dragyPassed100FromStop = false;
                _out.DragyStatus = "TIMING 0-100...";
            }

            if (_prevSpeedKmhDragy < 100.0 && speedKmh >= 100.0 && accel >= 120.0)
            {
                double frac100 = (100.0 - _prevSpeedKmhDragy) / Math.Max(0.01, speedKmh - _prevSpeedKmhDragy);
                double exactCross100Time = _prevTimeDragy + (Math.Max(0.0, Math.Min(1.0, frac100)) * dt);

                if (_dragyRunningFromStop && !_dragyPassed100FromStop)
                {
                    double t0To100 = exactCross100Time - _dragyStart0Time;
                    if (t0To100 > 0.8 && t0To100 < 25.0)
                    {
                        _out.DragyLast0To100 = Math.Round(t0To100, 2);
                        if (_globalEngine.Best0To100 <= 0 || t0To100 < _globalEngine.Best0To100)
                        {
                            _globalEngine.Best0To100 = Math.Round(t0To100, 2);
                            _out.DragyBest0To100 = _globalEngine.Best0To100;
                            _isDataDirty = true;
                        }
                        _out.DragyStatus = $"0-100: {_out.DragyLast0To100:0.00}s";
                    }
                    _dragyPassed100FromStop = true;
                }

                _dragyStart100Time = exactCross100Time;
                _dragyRunning100To200 = true;
            }

            if (_prevSpeedKmhDragy < 200.0 && speedKmh >= 200.0)
            {
                double frac200 = (200.0 - _prevSpeedKmhDragy) / Math.Max(0.01, speedKmh - _prevSpeedKmhDragy);
                double exactCross200Time = _prevTimeDragy + (Math.Max(0.0, Math.Min(1.0, frac200)) * dt);

                if (_dragyRunning100To200)
                {
                    double t100To200 = exactCross200Time - _dragyStart100Time;
                    if (t100To200 > 1.0 && t100To200 < 35.0)
                    {
                        _out.DragyLast100To200 = Math.Round(t100To200, 2);
                        if (_globalEngine.Best100To200 <= 0 || t100To200 < _globalEngine.Best100To200)
                        {
                            _globalEngine.Best100To200 = Math.Round(t100To200, 2);
                            _out.DragyBest100To200 = _globalEngine.Best100To200;
                            _isDataDirty = true;
                        }
                        _out.DragyStatus = $"100-200: {_out.DragyLast100To200:0.00}s";
                    }
                    _dragyRunning100To200 = false;
                }

                if (_dragyRunningFromStop)
                {
                    double t0To200 = exactCross200Time - _dragyStart0Time;
                    if (t0To200 > 2.0 && t0To200 < 50.0)
                    {
                        _out.DragyLast0To200 = Math.Round(t0To200, 2);
                        if (_globalEngine.Best0To200 <= 0 || t0To200 < _globalEngine.Best0To200)
                        {
                            _globalEngine.Best0To200 = Math.Round(t0To200, 2);
                            _out.DragyBest0To200 = _globalEngine.Best0To200;
                            _isDataDirty = true;
                        }
                    }
                    _dragyRunningFromStop = false;
                }
            }

            if (_dragyRunningFromStop && (now - _dragyStart0Time) > 35.0) _dragyRunningFromStop = false;
            if (_dragyRunning100To200 && (speedKmh < 94.0 || (now - _dragyStart100Time) > 30.0)) _dragyRunning100To200 = false;

            _prevSpeedKmhDragy = speedKmh;
            _prevTimeDragy = now;
        }

        private void ResetOutputs()
        {
            lock (_lock)
            {
                _out.CarKey = "";
                _out.CarName = "Unknown Car";
                _out.MaxUsableRpm = 0;
                _out.RevLimiterProximity = 0;
                _out.LedActiveCount = 0;
                _out.ShiftLightFlash = false;
                _out.ShiftTriggerRpm = 0;
                _out.OptimalShiftRpm = 0;
                _out.DownshiftTriggerRpm = 0;
                _out.BodySlipAngle = 0;
                _out.IsDownshiftSafe = false;
                _out.IsDownshiftSuggested = false;
                _out.IsLearningComplete = false;
                _out.AeroBottomingWarning = false;
                _out.PowerOversteerWarning = false;
                _out.OversteerSeverity = 0.0;
                _out.IsCounterSteering = false;
                _out.CounterSteerSeverity = 0.0;
                _out.SlideSeverity = 0.0;
                _out.SlideTrend = 0.0;
                _out.IsSlideDeepening = false;
                _out.IsSlideRecovering = false;
                _out.SlideStateText = "GRIP";
                _out.SlideBarColor = "#FF9900";
                _out.IsTopGear = false;
                _out.IsTractionGood = true;
                _out.IsGripLost = false;
                _out.LiveMappingProgress = 0;
                _out.DbgSteerRaw = 0;
                _out.DbgYawRaw = 0;
                _out.DbgChassisDive = 0.0;
                _out.EngineMapStatus = "LEARN"; 
                _out.MapCoverageBar = "[░░░░░░░░░░░░░░░░░░░░]";
                _out.CurrentGearStatusText = "-";
                _out.CurrentGearColor = "#00000000";
                _shiftLightActive = false;
                _smoothedLedProximity = 0.0;
                _currentLedStage = 0;
                _smoothedSlideLevel = 0.0;
                _prevSmoothedSlideLevel = 0.0;
                _smoothedSlideTrend = 0.0;
                _lastDownshiftSuggested = false;
                for (int i = 0; i <= 10; i++)
                {
                    _out.GearText[i] = LabelsZero[i];
                    _out.GearColor[i] = "#E74C3C";
                }
            }
        }

        private static double ReadDouble(PluginManager pm, string prop, double defaultVal = 0.0) 
        { 
            try 
            {
                object v = pm.GetPropertyValue(prop); 
                if (v == null) 
                {
                    if (_missingProps.Add(prop)) Debug.WriteLine($"[FH6SmartShift] Brak wlasciwosci (null): {prop}");
                    return defaultVal;
                }
                if (v is double d) return d;
                if (v is float f) return (double)f;
                if (v is int i) return (double)i;
                if (v is byte b) return (double)b;
                if (v is short s) return (double)s;
                return Convert.ToDouble(v);
            } 
            catch (Exception ex)
            { 
                if (_missingProps.Add(prop)) Debug.WriteLine($"[FH6SmartShift] Wyjatek dla {prop}: {ex.Message}");
                return defaultVal; 
            } 
        }

        public void DataUpdate(PluginManager pm, ref GameData data)
        {
            bool doResetAll = Interlocked.Exchange(ref _resetAllRequested, 0) == 1;
            bool doResetCurrent = Interlocked.Exchange(ref _resetCurrentRequested, 0) == 1;
            bool doResetLimitGears = Interlocked.Exchange(ref _resetLimitGearsRequested, 0) == 1;
            bool doResetRatios = Interlocked.Exchange(ref _resetRatiosRequested, 0) == 1;
            bool doResetDragy = Interlocked.Exchange(ref _resetDragyRequested, 0) == 1;
            bool doToggleBeep = Interlocked.Exchange(ref _toggleBeepRequested, 0) == 1;

            if (doToggleBeep)
            {
                lock (_lock)
                {
                    _store.ShiftBeepEnabled = !_store.ShiftBeepEnabled;
                    _snapshotStore.ShiftBeepEnabled = _store.ShiftBeepEnabled;
                    _out.IsShiftBeepEnabled = _store.ShiftBeepEnabled;
                    _isDataDirty = true;
                }
                TriggerAsyncSave();
            }

            if (doResetDragy)
            {
                lock (_lock)
                {
                    _globalEngine.Best0To100 = 0;
                    _globalEngine.Best100To200 = 0;
                    _globalEngine.Best0To200 = 0;
                    _out.DragyLast0To100 = 0;
                    _out.DragyBest0To100 = 0;
                    _out.DragyLast100To200 = 0;
                    _out.DragyBest100To200 = 0;
                    _out.DragyLast0To200 = 0;
                    _out.DragyBest0To200 = 0;
                    _out.DragyStatus = "RESET";
                    _isDataDirty = true;
                }
                TriggerAsyncSave();
            }

            if (doResetAll) 
            { 
                lock (_lock) 
                { 
                    bool keepBeep = _store.ShiftBeepEnabled;
                    double keepBuf = _store.ReactionBufferFactor;
                    var keepNames = _store.CarNames;
                    _store = new CarStore { SchemaVersion = 2, ShiftBeepEnabled = keepBeep, ReactionBufferFactor = keepBuf, CarNames = keepNames }; 
                    _snapshotStore = CloneCarStore(_store);
                    _gearStates.Clear(); 
                    _globalEngine = new GlobalEngineProfile(); 
                    _isDataDirty = true; 
                    _key = "";
                    _lastCarOrdinal = -1;
                    _lastCarPI = -1;
                    _lastCarDrivetrain = -1;
                    _lastCarEngineMaxRpmRounded = -1;
                    _peakRpmBeforeCut = 0;
                    _cleanWotFrames = 0;
                    _lastLearnedBand = -1;
                    _lastLearnedPower = 0;
                    _lastLearnedTime = -100;
                    _lastMapScanTime = -100;
                    _smoothedLiveBuffer = 50.0;
                    _smoothedLedProximity = 0.0;
                    _currentLedStage = 0;
                    _smoothedSlideLevel = 0.0;
                    _prevSmoothedSlideLevel = 0.0;
                    _smoothedSlideTrend = 0.0;
                    _speedTrendKmhPerS = 0;
                    _lastSpeedKmhForTrend = 0;
                    _climbDownshiftActive = false;
                    _staticChassisDiveBase = 0.0;
                    _isChassisBaseInitialized = false;
                    _mathNeedsEval = true;
                } 
                TriggerAsyncSave();
            }
            if (doResetCurrent) 
            { 
                lock (_lock) 
                { 
                    if (!string.IsNullOrEmpty(_key)) 
                    { 
                        if (_store.GearRatios.ContainsKey(_key)) _store.GearRatios.Remove(_key); 
                        if (_store.EngineProfiles.ContainsKey(_key)) _store.EngineProfiles.Remove(_key); 
                        if (_store.AdvancedGears.ContainsKey(_key)) _store.AdvancedGears.Remove(_key);

                        if (_snapshotStore.GearRatios.ContainsKey(_key)) _snapshotStore.GearRatios.Remove(_key);
                        if (_snapshotStore.EngineProfiles.ContainsKey(_key)) _snapshotStore.EngineProfiles.Remove(_key);
                        if (_snapshotStore.AdvancedGears.ContainsKey(_key)) _snapshotStore.AdvancedGears.Remove(_key);
                    } 
                    _gearStates.Clear(); 
                    _globalEngine = new GlobalEngineProfile(); 
                    _isDataDirty = true; 
                    _peakRpmBeforeCut = 0;
                    _cleanWotFrames = 0;
                    _lastLearnedBand = -1;
                    _lastLearnedPower = 0;
                    _lastLearnedTime = -100;
                    _lastMapScanTime = -100;
                    _smoothedLiveBuffer = 50.0;
                    _smoothedLedProximity = 0.0;
                    _currentLedStage = 0;
                    _smoothedSlideLevel = 0.0;
                    _prevSmoothedSlideLevel = 0.0;
                    _smoothedSlideTrend = 0.0;
                    _speedTrendKmhPerS = 0;
                    _lastSpeedKmhForTrend = 0;
                    _climbDownshiftActive = false;
                    _staticChassisDiveBase = 0.0;
                    _isChassisBaseInitialized = false;
                    _mathNeedsEval = true;
                } 
                TriggerAsyncSave();
            }
            if (doResetLimitGears)
            {
                lock (_lock)
                {
                    if (!string.IsNullOrEmpty(_key))
                    {
                        bool anyRemoved = false;
                        foreach (var kvp in _gearStates)
                        {
                            var gear = kvp.Key;
                            var gs = kvp.Value;
                            if (gs.ShiftTriggerRpm > 0 && !gs.IsMathOptimal)
                            {
                                gs.ShiftTriggerRpm = 0;
                                gs.PowerPeakCaught = false;
                                gs.IsConfirmed = false;
                                gs.TriggerIsEdge = false;
                                gs.TriggerIsGhost = false;

                                if (_store.AdvancedGears.ContainsKey(_key)) _store.AdvancedGears[_key].Remove(gear);
                                if (_snapshotStore.AdvancedGears.ContainsKey(_key)) _snapshotStore.AdvancedGears[_key].Remove(gear);
                                anyRemoved = true;
                            }
                        }
                        if (anyRemoved)
                        {
                            _isDataDirty = true;
                            _mathNeedsEval = true;
                        }
                    }
                }
                TriggerAsyncSave();
            }
            if (doResetRatios)
            {
                lock (_lock)
                {
                    if (!string.IsNullOrEmpty(_key))
                    {
                        if (_store.GearRatios.ContainsKey(_key)) _store.GearRatios.Remove(_key);
                        if (_store.AdvancedGears.ContainsKey(_key)) _store.AdvancedGears.Remove(_key);
                        if (_snapshotStore.GearRatios.ContainsKey(_key)) _snapshotStore.GearRatios.Remove(_key);
                        if (_snapshotStore.AdvancedGears.ContainsKey(_key)) _snapshotStore.AdvancedGears.Remove(_key);
                    }
                    _gearStates.Clear();
                    _isDataDirty = true;
                    _mathNeedsEval = true;
                }
                TriggerAsyncSave();
            }

            string activeGameName = pm.GameName ?? "";
            bool isMotorsport = activeGameName.IndexOf("Motorsport", StringComparison.OrdinalIgnoreCase) >= 0 
                             || activeGameName.IndexOf("FM", StringComparison.OrdinalIgnoreCase) >= 0;
            string gamePrefix = isMotorsport ? "FM_" : "";
            _out.ActiveGameMode = isMotorsport ? "FM" : "FH6";

            int currentOrdinal = (int)ReadDouble(pm, "DataCorePlugin.GameRawData.CarOrdinal");
            
            if (currentOrdinal <= 0) 
            { 
                ResetOutputs(); 
                return; 
            }

            double currentRpm = ReadDouble(pm, "DataCorePlugin.GameRawData.CurrentEngineRpm");
            double engineMaxRpm = ReadDouble(pm, "DataCorePlugin.GameRawData.EngineMaxRpm"); 
            double isRaceOn = ReadDouble(pm, "DataCorePlugin.GameRawData.IsRaceOn", 1.0);

            if (currentRpm <= 0 || engineMaxRpm <= 0 || isRaceOn <= 0) 
            {
                ResetOutputs();
                return;
            }

            double now = _appTime.Elapsed.TotalSeconds;

            if (now - _lastCarListCheckTime > 10.0)
            {
                _lastCarListCheckTime = now;
                EnsureAndLoadCarList();
            }

            int currentPI = (int)ReadDouble(pm, "DataCorePlugin.GameRawData.CarPerformanceIndex");
            if (currentPI <= 0) 
            { 
                ResetOutputs(); 
                return; 
            }

            double engineIdleRpm = ReadDouble(pm, "DataCorePlugin.GameRawData.EngineIdleRpm");
            int dtType = (int)ReadDouble(pm, "DataCorePlugin.GameRawData.DrivetrainType");
            int roundedMaxRpm = (int)(Math.Round(engineMaxRpm / 100.0) * 100);
            string expectedKey = $"{gamePrefix}{currentOrdinal}_PI{currentPI}_DT{dtType}_R{roundedMaxRpm}";

            if (expectedKey != _key) 
            {
                lock (_lock) 
                { 
                    if (!string.IsNullOrEmpty(_key)) 
                    { 
                        SaveEngineProfileToStore(); 
                        UpdateSnapshotWithCurrentCar();
                        _isDataDirty = true;
                    }

                    _lastCarOrdinal = currentOrdinal;
                    _lastCarPI = currentPI;
                    _lastCarDrivetrain = dtType;
                    _lastCarEngineMaxRpmRounded = roundedMaxRpm;
                    
                    string oldKey = $"{currentOrdinal}_PI{currentPI}";
                    _key = expectedKey; 

                    if (!isMotorsport)
                    {
                        bool hasOldData = _store.EngineProfiles.ContainsKey(oldKey) || _store.GearRatios.ContainsKey(oldKey) || _store.AdvancedGears.ContainsKey(oldKey);
                        if (hasOldData && !_store.EngineProfiles.ContainsKey(_key) && !_store.GearRatios.ContainsKey(_key) && !_store.AdvancedGears.ContainsKey(_key))
                        {
                            if (_store.EngineProfiles.TryGetValue(oldKey, out var oldEp)) {
                                _store.EngineProfiles[_key] = oldEp;
                                _store.EngineProfiles.Remove(oldKey);
                                _snapshotStore.EngineProfiles.Remove(oldKey);
                            }
                            if (_store.GearRatios.TryGetValue(oldKey, out var oldGr)) {
                                _store.GearRatios[_key] = oldGr;
                                _store.GearRatios.Remove(oldKey);
                                _snapshotStore.GearRatios.Remove(oldKey);
                            }
                            if (_store.AdvancedGears.TryGetValue(oldKey, out var oldAg)) {
                                _store.AdvancedGears[_key] = oldAg;
                                _store.AdvancedGears.Remove(oldKey);
                                _snapshotStore.AdvancedGears.Remove(oldKey);
                            }
                            _isDataDirty = true;
                        }
                    }
                    
                    _gearStates.Clear(); 
                    _globalEngine = new GlobalEngineProfile();
                    _cleanWotFrames = 0;
                    _lastLearnedBand = -1;
                    _lastLearnedPower = 0;
                    _lastLearnedTorque = 0;
                    _lastLearnedTime = -100;
                    _lastMapScanTime = -100;
                    _smoothedLiveBuffer = 50.0;
                    _smoothedLedProximity = 0.0;
                    _currentLedStage = 0;
                    _smoothedSlideLevel = 0.0;
                    _prevSmoothedSlideLevel = 0.0;
                    _smoothedSlideTrend = 0.0;
                    _speedTrendKmhPerS = 0;
                    _lastSpeedKmhForTrend = 0;
                    _climbDownshiftActive = false;
                    _staticChassisDiveBase = 0.0;
                    _isChassisBaseInitialized = false;
                    _lastGear = 0; _peakRpmBeforeCut = 0; _currentRpmAccel = 0;
                    _dragyArmedFromStop = false;
                    _dragyRunningFromStop = false;
                    _dragyRunning100To200 = false;
                    _out.DragyLast0To100 = 0;
                    _out.DragyLast100To200 = 0;
                    _out.DragyLast0To200 = 0;
                    _out.DragyStatus = "READY";

                    if (_store.EngineProfiles.TryGetValue(_key, out var savedEngine))
                    {
                        if (savedEngine.PowerMap != null && savedEngine.PowerMap.Length == 401) Array.Copy(savedEngine.PowerMap, _globalEngine.PowerMap, 401);
                        if (savedEngine.TorqueMap != null && savedEngine.TorqueMap.Length == 401) Array.Copy(savedEngine.TorqueMap, _globalEngine.TorqueMap, 401);
                        if (savedEngine.BoostMap != null && savedEngine.BoostMap.Length == 401) 
                        {
                            Array.Copy(savedEngine.BoostMap, _globalEngine.BoostMap, 401);
                        }
                        
                        _globalEngine.MaxPower = savedEngine.MaxPower; _globalEngine.RpmAtMaxPower = savedEngine.RpmAtMaxPower;
                        _globalEngine.MaxTorque = savedEngine.MaxTorque; _globalEngine.RpmAtMaxTorque = savedEngine.RpmAtMaxTorque;
                        _globalEngine.MaxBoost = savedEngine.MaxBoost;
                        _globalEngine.GhostLimiterRpm = savedEngine.GhostLimiterRpm;
                        _globalEngine.MappingProgress = savedEngine.MappingProgress;
                        _globalEngine.Best0To100 = savedEngine.Best0To100;
                        _globalEngine.Best100To200 = savedEngine.Best100To200;
                        _globalEngine.Best0To200 = savedEngine.Best0To200;

                        _globalEngine.MinMappedBand = 400; _globalEngine.MaxMappedBand = 0;
                        for(int i=0; i<401; i++) {
                            if (_globalEngine.PowerMap[i] > 0) {
                                _globalEngine.BandSamples[i] = 6; 
                                if (i < _globalEngine.MinMappedBand) _globalEngine.MinMappedBand = i;
                                if (i > _globalEngine.MaxMappedBand) _globalEngine.MaxMappedBand = i;
                            }
                            if (_globalEngine.BoostMap[i] > _globalEngine.MaxBoost) _globalEngine.MaxBoost = _globalEngine.BoostMap[i];
                        }

                        if (HealEngineMapPockets()) _isDataDirty = true;

                        double loadedMappedCeilingRpm = _globalEngine.MaxMappedBand * 50.0;
                        if (_globalEngine.GhostLimiterRpm > 0 && (_globalEngine.GhostLimiterRpm >= engineMaxRpm - 35.0 || (loadedMappedCeilingRpm >= 4000.0 && _globalEngine.GhostLimiterRpm > loadedMappedCeilingRpm + 250.0)))
                        {
                            _globalEngine.GhostLimiterRpm = 0;
                            savedEngine.GhostLimiterRpm = 0;
                            _isDataDirty = true;
                        }
                    }

                    _out.DragyBest0To100 = _globalEngine.Best0To100;
                    _out.DragyBest100To200 = _globalEngine.Best100To200;
                    _out.DragyBest0To200 = _globalEngine.Best0To200;

                    if (_store.GearRatios.TryGetValue(_key, out var savedRatios))
                    {
                        var sortedGears = new List<int>(savedRatios.Keys);
                        sortedGears.Sort();
                        
                        bool cleanLoop = true;
                        while(cleanLoop) {
                            cleanLoop = false;
                            int worstGear = -1;
                            int maxViolations = 0;
                            
                            for(int i = 0; i < sortedGears.Count; i++) {
                                int g1 = sortedGears[i];
                                double r1 = savedRatios[g1];
                                if (r1 <= 0) continue;
                                
                                int violations = 0;
                                for(int j = 0; j < sortedGears.Count; j++) {
                                    if (i == j) continue;
                                    int g2 = sortedGears[j];
                                    double r2 = savedRatios[g2];
                                    if (r2 <= 0) continue;
                                    int diff = Math.Abs(g2 - g1);
                                    double minStep = Math.Pow(1.022, diff);
                                    if (g1 < g2 && r1 <= r2 * minStep) violations++;
                                    else if (g1 > g2 && r1 * minStep >= r2) violations++;
                                }
                                
                                if (violations > maxViolations || (violations == maxViolations && violations > 0 && g1 > worstGear)) {
                                    maxViolations = violations;
                                    worstGear = g1;
                                }
                            }
                            
                            if (maxViolations > 0 && worstGear != -1) {
                                savedRatios.Remove(worstGear);
                                sortedGears.Remove(worstGear);
                                if (_store.AdvancedGears.TryGetValue(_key, out var agClean)) agClean.Remove(worstGear);
                                _isDataDirty = true;
                                cleanLoop = true;
                            }
                        }

                        foreach(var gear in sortedGears) {
                            if (savedRatios.ContainsKey(gear) && savedRatios[gear] > 0) {
                                if (!_gearStates.ContainsKey(gear)) _gearStates[gear] = new GearState();
                                _gearStates[gear].RpmPerRadS = savedRatios[gear];
                                _gearStates[gear].SavedRpmPerRadS = savedRatios[gear];
                                _gearStates[gear].RatioSamples = 0; 
                            }
                        }
                        SanitizeGearProgression();
                        _mathNeedsEval = true;
                    }

                    if (_store.AdvancedGears.TryGetValue(_key, out var advGears))
                    {
                        foreach(var kvp in advGears)
                        {
                            if (!_gearStates.ContainsKey(kvp.Key)) _gearStates[kvp.Key] = new GearState();
                            
                            double loadedRpm = kvp.Value.Rpm;
                            if (_globalEngine.GhostLimiterRpm > 0 && loadedRpm > _globalEngine.GhostLimiterRpm && !kvp.Value.Gho) {
                                loadedRpm = _globalEngine.GhostLimiterRpm - 30;
                            }
                            
                            _gearStates[kvp.Key].ShiftTriggerRpm = loadedRpm;
                            _gearStates[kvp.Key].PowerPeakCaught = true;
                            _gearStates[kvp.Key].IsConfirmed = kvp.Value.Opt || kvp.Value.Gho;
                            _gearStates[kvp.Key].IsMathOptimal = kvp.Value.Opt;
                            _gearStates[kvp.Key].TriggerIsEdge = kvp.Value.Edg;
                            _gearStates[kvp.Key].TriggerIsGhost = kvp.Value.Gho;
                        }
                    }
                }
                
                bool localDirty;
                lock (_lock) { localDirty = _isDataDirty; }
                if (localDirty) TriggerAsyncSave();
            }

            int currentGear = (int)ReadDouble(pm, "DataCorePlugin.GameRawData.Gear"); 
            double accel = ReadDouble(pm, "DataCorePlugin.GameRawData.Accel");
            double brake = ReadDouble(pm, "DataCorePlugin.GameRawData.Brake");
            double clutch = ReadDouble(pm, "DataCorePlugin.GameRawData.Clutch");
            double handbrake = ReadDouble(pm, "DataCorePlugin.GameRawData.HandBrake");
            double velX = ReadDouble(pm, "DataCorePlugin.GameRawData.VelocityX");
            double velZ = ReadDouble(pm, "DataCorePlugin.GameRawData.VelocityZ");
            double currentPower = Math.Max(ReadDouble(pm, "DataCorePlugin.GameRawData.Power"), 0);
            double currentTorque = Math.Max(ReadDouble(pm, "DataCorePlugin.GameRawData.Torque"), 0);

            double susNormFL = ReadDouble(pm, "DataCorePlugin.GameRawData.NormalizedSuspensionTravelFrontLeft");
            double susNormFR = ReadDouble(pm, "DataCorePlugin.GameRawData.NormalizedSuspensionTravelFrontRight");
            double susNormRL = ReadDouble(pm, "DataCorePlugin.GameRawData.NormalizedSuspensionTravelRearLeft");
            double susNormRR = ReadDouble(pm, "DataCorePlugin.GameRawData.NormalizedSuspensionTravelRearRight");
            
            double puddleFL = ReadDouble(pm, "DataCorePlugin.GameRawData.WheelInPuddleDepthFrontLeft");
            double puddleFR = ReadDouble(pm, "DataCorePlugin.GameRawData.WheelInPuddleDepthFrontRight");
            double puddleRL = ReadDouble(pm, "DataCorePlugin.GameRawData.WheelInPuddleDepthRearLeft");
            double puddleRR = ReadDouble(pm, "DataCorePlugin.GameRawData.WheelInPuddleDepthRearRight");
            
            double rumbleFL = ReadDouble(pm, "DataCorePlugin.GameRawData.SurfaceRumbleFrontLeft");
            double rumbleFR = ReadDouble(pm, "DataCorePlugin.GameRawData.SurfaceRumbleFrontRight");
            double rumbleRL = ReadDouble(pm, "DataCorePlugin.GameRawData.SurfaceRumbleRearLeft");
            double rumbleRR = ReadDouble(pm, "DataCorePlugin.GameRawData.SurfaceRumbleRearRight");
            
            double combinedSlipRL = ReadDouble(pm, "DataCorePlugin.GameRawData.TireCombinedSlipRearLeft");
            double combinedSlipRR = ReadDouble(pm, "DataCorePlugin.GameRawData.TireCombinedSlipRearRight");

            double damage1 = ReadDouble(pm, "DataCorePlugin.GameRawData.Damage1");
            double damage2 = ReadDouble(pm, "DataCorePlugin.GameRawData.Damage2");

            bool isGrounded = susNormFL > 0.02 && susNormFR > 0.02 && susNormRL > 0.02 && susNormRR > 0.02;
            bool inPuddle = puddleFL > 0.01 || puddleFR > 0.01 || puddleRL > 0.01 || puddleRR > 0.01;
            bool hasDamage = damage1 > 0.05 || damage2 > 0.05;
            bool isRumble = rumbleFL > 0 || rumbleFR > 0 || rumbleRL > 0 || rumbleRR > 0;

            Drivetrain drivetrainType = (Drivetrain)dtType;
            double speedKmh = Math.Sqrt((velX * velX) + (velZ * velZ)) * 3.6;

            lock (_lock)
            {
                UpdateDragyTimers(speedKmh, now, accel, brake, isGrounded, currentGear);
            }
            
            double steer = ReadDouble(pm, "DataCorePlugin.GameRawData.Steer");
            double angularVelY = ReadDouble(pm, "DataCorePlugin.GameRawData.AngularVelocityY");

            if (currentGear < 1) 
            {
                _lastSusTravelRL = ReadDouble(pm, "DataCorePlugin.GameRawData.SuspensionTravelMetersRearLeft");
                _lastSusTravelRR = ReadDouble(pm, "DataCorePlugin.GameRawData.SuspensionTravelMetersRearRight");
                _cleanWotFrames = 0;
                _lastLearnedBand = -1;
                _lastLearnedPower = 0;
                _lastLearnedTime = -100;
                _climbDownshiftActive = false;
                UpdateOutputsAndState(currentOrdinal, currentGear, engineIdleRpm, engineMaxRpm, currentRpm, now, true, accel, velX, velZ, true, brake, steer, 0.0, 0.0, engineMaxRpm, susNormFL, susNormFR, susNormRL, susNormRR, combinedSlipRL, combinedSlipRR, 0.0, angularVelY, drivetrainType);
                return;
            }

            double susTravelRL = ReadDouble(pm, "DataCorePlugin.GameRawData.SuspensionTravelMetersRearLeft");
            double susTravelRR = ReadDouble(pm, "DataCorePlugin.GameRawData.SuspensionTravelMetersRearRight");
            double accelX = ReadDouble(pm, "DataCorePlugin.GameRawData.AccelerationX");
            double accelZ = ReadDouble(pm, "DataCorePlugin.GameRawData.AccelerationZ");
            
            double deltaTravelRL = Math.Abs(susTravelRL - _lastSusTravelRL);
            double deltaTravelRR = Math.Abs(susTravelRR - _lastSusTravelRR);
            bool isReboundDetected = (deltaTravelRL > SusTravelReboundThreshold || deltaTravelRR > SusTravelReboundThreshold);
            
            _lastSusTravelRL = susTravelRL;
            _lastSusTravelRR = susTravelRR;

            double wheelSpeedFL = ReadDouble(pm, "DataCorePlugin.GameRawData.WheelRotationSpeedFrontLeft");
            double wheelSpeedFR = ReadDouble(pm, "DataCorePlugin.GameRawData.WheelRotationSpeedFrontRight");
            double wheelSpeedRL = ReadDouble(pm, "DataCorePlugin.GameRawData.WheelRotationSpeedRearLeft");
            double wheelSpeedRR = ReadDouble(pm, "DataCorePlugin.GameRawData.WheelRotationSpeedRearRight");

            double slipRatioFL = ReadDouble(pm, "DataCorePlugin.GameRawData.TireSlipRatioFrontLeft");
            double slipRatioFR = ReadDouble(pm, "DataCorePlugin.GameRawData.TireSlipRatioFrontRight");
            double slipRatioRL = ReadDouble(pm, "DataCorePlugin.GameRawData.TireSlipRatioRearLeft");
            double slipRatioRR = ReadDouble(pm, "DataCorePlugin.GameRawData.TireSlipRatioRearRight");
            
            double slipAngleFL = ReadDouble(pm, "DataCorePlugin.GameRawData.TireSlipAngleFrontLeft");
            double slipAngleFR = ReadDouble(pm, "DataCorePlugin.GameRawData.TireSlipAngleFrontRight");
            double slipAngleRL = ReadDouble(pm, "DataCorePlugin.GameRawData.TireSlipAngleRearLeft");
            double slipAngleRR = ReadDouble(pm, "DataCorePlugin.GameRawData.TireSlipAngleRearRight");

            double maxDrivenSlipRatio = 0;
            double maxDrivenSlipAngle = Math.Max(Math.Abs(slipAngleRL), Math.Abs(slipAngleRR));
            double maxAllSlip = Math.Max(Math.Max(Math.Abs(slipRatioFL), Math.Abs(slipRatioFR)), Math.Max(Math.Abs(slipRatioRL), Math.Abs(slipRatioRR)));
            
            double avgWheelSpeed = 0;

            if (drivetrainType == Drivetrain.AWD) 
            {
                avgWheelSpeed = (Math.Abs(wheelSpeedFL) + Math.Abs(wheelSpeedFR) + Math.Abs(wheelSpeedRL) + Math.Abs(wheelSpeedRR)) / 4.0;
                maxDrivenSlipRatio = maxAllSlip;
            } 
            else if (drivetrainType == Drivetrain.FWD) 
            {
                maxDrivenSlipRatio = Math.Max(Math.Abs(slipRatioFL), Math.Abs(slipRatioFR));
                maxDrivenSlipAngle = Math.Max(Math.Abs(slipAngleFL), Math.Abs(slipAngleFR));
                avgWheelSpeed = (Math.Abs(wheelSpeedFL) + Math.Abs(wheelSpeedFR)) / 2.0;
            } 
            else 
            {
                maxDrivenSlipRatio = Math.Max(Math.Abs(slipRatioRL), Math.Abs(slipRatioRR));
                avgWheelSpeed = (Math.Abs(wheelSpeedRL) + Math.Abs(wheelSpeedRR)) / 2.0;
            }

            // Zaostrzony filtr dla nauki przełożeń, aby uślizg na wyższych biegach nie zaniżał przełożenia (np. 36.05 zamiast 43.32)
            double ratioSlipLimit = (currentGear <= 2) ? 0.14 : 0.09;
            bool isRatioLearnSafe = isGrounded && !isReboundDetected && !inPuddle && (maxDrivenSlipRatio < ratioSlipLimit) && (maxDrivenSlipAngle < 0.12) && (brake < BrakeDeadzone);
            double currentRatio = (isRatioLearnSafe && avgWheelSpeed > 10.0 && currentRpm > 1200.0) ? currentRpm / avgWheelSpeed : 0;

            double maxRearSlipAngle = Math.Max(Math.Abs(slipAngleRL), Math.Abs(slipAngleRR));
            double maxRearCombinedSlip = Math.Max(Math.Abs(combinedSlipRL), Math.Abs(combinedSlipRR));
            double currentOversteer = 0.0;
            
            if (speedKmh > 15.0)
            {
                double angleSpan = isMotorsport ? 0.36 : 0.45;
                double angleScore = Math.Max(0, (maxRearSlipAngle - 0.08) / angleSpan); 
                double slipScore = Math.Max(0, (maxRearCombinedSlip - 0.45) / 0.75); 
                
                currentOversteer = Math.Min(1.0, (angleScore * 0.65) + (slipScore * 0.35));
                if (drivetrainType == Drivetrain.AWD) currentOversteer *= 0.80; 
            }

            bool isDrivetrainDecoupled = (clutch >= 2.0 || handbrake >= 2.0);
            if (isDrivetrainDecoupled) _lastDecoupledTime = now;
            bool isAfterClutch = (now - _lastDecoupledTime) < ClutchSettleSeconds;

            if (_lastGear >= 1 && currentGear != _lastGear)
            {
                if (currentGear < _lastGear) { _lastDownshiftTime = now; _lastUpshiftTime = -100; _shiftLightActive = false; }
                else { _lastUpshiftTime = now; _lastDownshiftTime = -100; _shiftLightActive = false; }
                
                _smoothedLedProximity = 0.0;
                _currentLedStage = 0;
                _cleanWotFrames = 0;

                if (_gearStates.TryGetValue(_lastGear, out var oldGs)) { oldGs.RatioSamples = 0; oldGs.RatioHistoryIdx = 0; }
                if (_gearStates.TryGetValue(currentGear, out var newGs)) { newGs.RatioSamples = 0; newGs.RatioHistoryIdx = 0; }
                _lastLearnedBand = -1;
                _lastLearnedPower = 0;
                _lastLearnedTime = -100;
                _climbDownshiftActive = false;

                _mathNeedsEval = true;
            }
            _lastGear = currentGear;
            
            bool isAfterAnyShift = (now - _lastDownshiftTime < DownshiftCooldownSeconds) || 
                                   (now - _lastUpshiftTime < UpshiftCooldownSeconds) || isAfterClutch;

            bool isAfterShiftForRatio = (now - _lastDownshiftTime < DownshiftCooldownSeconds) || 
                                        (now - _lastUpshiftTime < RatioLearnShiftSettleSeconds) || isAfterClutch;

            double rawDt = _dtStopwatch.Elapsed.TotalSeconds;
            _dtStopwatch.Restart();
            _frameDt = Math.Max(0.001, Math.Min(rawDt, 0.05));
            
            if (_frameDt > 0)
            {
                double currentAccelKmh = (speedKmh - _lastSpeedKmhForTrend) / _frameDt;
                double tauSpeed = 0.5; 
                double alphaSpeed = 1.0 - Math.Exp(-_frameDt / tauSpeed);
                _speedTrendKmhPerS = (_speedTrendKmhPerS * (1.0 - alphaSpeed)) + (currentAccelKmh * alphaSpeed);
            }
            _lastSpeedKmhForTrend = speedKmh;

            if (rawDt >= 0.10 || isDrivetrainDecoupled || isAfterAnyShift) 
            { 
                _currentRpmAccel = 0; 
                _lastRpmForAccel = currentRpm;
                _lastRpmChangeTime = now;
            }
            else if (currentRpm != _lastRpmForAccel)
            {
                double span = Math.Max(0.001, Math.Min(now - _lastRpmChangeTime, 0.05));
                if (span >= 0.008)
                {
                    double tauRPM = 0.1;
                    double alphaRPM = 1.0 - Math.Exp(-span / tauRPM);
                    _currentRpmAccel = (_currentRpmAccel * (1.0 - alphaRPM)) + (((currentRpm - _lastRpmForAccel) / span) * alphaRPM); 
                    _lastRpmForAccel = currentRpm; 
                    _lastRpmChangeTime = now;
                }
            }
            else if (now - _lastRpmChangeTime > 0.05)
            {
                _currentRpmAccel *= 0.8;
            }

            double angleLimit = isMotorsport ? 0.22 : SlipAngleThreshold;
            bool isLowGear = currentGear >= 1 && currentGear <= 3;
            double slipTolerance = isMotorsport
                ? (isLowGear ? 0.45 : 0.28)
                : (isLowGear ? 0.60 : 0.38); 
            
            bool tractionOk = maxDrivenSlipRatio < slipTolerance && maxDrivenSlipAngle < angleLimit;

            bool isLearnWot = accel >= LearnWotThreshold; 
            bool isDrivetrainEngaged = clutch < 2.0 && handbrake < 2.0;

            lock (_lock)
            {
                if (!_gearStates.ContainsKey(currentGear)) _gearStates[currentGear] = new GearState();
                var gs = _gearStates[currentGear];

                bool cleanTelemetryForLimiter = isGrounded && !inPuddle && !hasDamage && !isReboundDetected && !isRumble && isDrivetrainEngaged && !isAfterAnyShift && tractionOk;

                if (cleanTelemetryForLimiter && currentRpm > _peakRpmBeforeCut && _currentRpmAccel > 0)
                {
                    _peakRpmBeforeCut = currentRpm;
                }

                double mappedCeilingRpm = _globalEngine.MaxMappedBand * 50.0;
                if (_globalEngine.GhostLimiterRpm > 0 && (_globalEngine.GhostLimiterRpm >= engineMaxRpm - 35.0 || (mappedCeilingRpm >= 4000.0 && _globalEngine.GhostLimiterRpm > mappedCeilingRpm + 250.0)))
                {
                    _globalEngine.GhostLimiterRpm = 0;
                    foreach (var kvp in _gearStates)
                    {
                        if (!kvp.Value.IsMathOptimal)
                        {
                            kvp.Value.IsConfirmed = false;
                            kvp.Value.TriggerIsGhost = false;
                        }
                    }
                    _isDataDirty = true;
                    _mathNeedsEval = true;
                }

                if (accel < LiftThreshold || brake > BrakeDeadzone || isDrivetrainDecoupled || isAfterAnyShift)
                {
                    if (_globalEngine.GhostLimiterCandidateRpm > 0 && _globalEngine.GhostLimiterHits <= 1 && (now - _globalEngine.GhostLimiterCandidateTime) < 0.08)
                    {
                        _globalEngine.GhostLimiterCandidateRpm = 0;
                        _globalEngine.GhostLimiterCandidateTime = 0;
                        _globalEngine.GhostLimiterHits = 0;
                        _isDataDirty = true; 
                        _mathNeedsEval = true;
                    }
                    _peakRpmBeforeCut = 0;
                    _cleanWotFrames = 0;
                    _lastLearnedBand = -1;
                    _lastLearnedPower = 0;
                    _lastLearnedTime = -100;
                }

                if (isLearnWot && brake < BrakeDeadzone && cleanTelemetryForLimiter)
                {
                    if (_globalEngine.GhostLimiterRpm > 0 && currentRpm > _globalEngine.GhostLimiterRpm + 150 && _currentRpmAccel > 0)
                    {
                        gs.GhostLimiterBreakAccumulator += _frameDt;
                        if (gs.GhostLimiterBreakAccumulator >= 0.10)
                        {
                            _globalEngine.GhostLimiterRpm = 0;
                            _isDataDirty = true;
                            _mathNeedsEval = true;
                            foreach (var kvp in _gearStates) { 
                                if (kvp.Value.TriggerIsGhost) {
                                    kvp.Value.IsConfirmed = false; 
                                    kvp.Value.TriggerIsGhost = false; 
                                }
                            }
                            gs.GhostLimiterBreakAccumulator = 0;
                        }
                    }
                    else 
                    { 
                        gs.GhostLimiterBreakAccumulator = 0; 
                    }

                    double minGhostTriggerRpm = _globalEngine.RpmAtMaxPower > 0
                        ? Math.Min(_globalEngine.RpmAtMaxPower + 100.0, Math.Max(3000.0, mappedCeilingRpm - 100.0))
                        : engineMaxRpm * 0.65;
                    bool isHighRpm = mappedCeilingRpm >= 3000.0 && currentRpm >= minGhostTriggerRpm;

                    if (isHighRpm && _globalEngine.GhostLimiterRpm == 0)
                    {
                        double expectedPower = GetInterpolatedValue(currentRpm, _globalEngine.PowerMap);
                        double referencePower = expectedPower > 0 ? expectedPower : _globalEngine.MaxPower;
                        double powerRatio = referencePower > 0 ? (currentPower / referencePower) : 1.0;

                        bool isRpmWall = currentRpm >= (mappedCeilingRpm - 50.0) && Math.Abs(_currentRpmAccel) < 60.0;
                        bool isLimiterHit = (referencePower > 0 && powerRatio < 0.85) || _currentRpmAccel < -150.0 || isRpmWall;

                        if (isLimiterHit)
                        {
                            gs.GhostLimiterAccumulator += _frameDt;
                            if (gs.GhostLimiterAccumulator >= 0.05)
                            {
                                double currentPeak = _peakRpmBeforeCut > 0 ? _peakRpmBeforeCut : currentRpm;
                                
                                if (currentPeak < engineMaxRpm - 35.0)
                                {
                                    if (_globalEngine.GhostLimiterCandidateRpm == 0)
                                    {
                                        _globalEngine.GhostLimiterCandidateRpm = currentPeak;
                                        _globalEngine.GhostLimiterCandidateTime = now;
                                        _globalEngine.GhostLimiterHits = 1;
                                        _isDataDirty = true;
                                    }
                                    else if (Math.Abs(_globalEngine.GhostLimiterCandidateRpm - currentPeak) <= 150 && (now - _globalEngine.GhostLimiterCandidateTime) >= 0.08)
                                    {
                                        double potentialGhost = Math.Max(_globalEngine.GhostLimiterCandidateRpm, currentPeak);
                                        int checkIdx = (int)Math.Floor((potentialGhost + 150) / 50.0);
                                        int validIntervals = 0;
                                        
                                        if (checkIdx <= 400)
                                        {
                                            for (int i = checkIdx; i <= 400; i++) { if (_globalEngine.PowerMap[i] > 0) validIntervals++; }
                                        }

                                        if (validIntervals >= 3) { 
                                            _globalEngine.GhostLimiterCandidateRpm = 0; 
                                            _globalEngine.GhostLimiterCandidateTime = 0;
                                            _globalEngine.GhostLimiterHits = 0;
                                        } 
                                        else
                                        {
                                            _globalEngine.GhostLimiterHits++;
                                            _globalEngine.GhostLimiterCandidateTime = now;
                                            
                                            if (_globalEngine.GhostLimiterHits >= 2 || checkIdx > 400)
                                            {
                                                _globalEngine.GhostLimiterRpm = potentialGhost;
                                                CleanUpContaminatedBands(potentialGhost);
                                                _globalEngine.GhostLimiterCandidateRpm = 0; 
                                                _globalEngine.GhostLimiterCandidateTime = 0;
                                                _globalEngine.GhostLimiterHits = 0;
                                                _isDataDirty = true;
                                                _mathNeedsEval = true; 
                                            }
                                        }
                                    }
                                    else if (Math.Abs(_globalEngine.GhostLimiterCandidateRpm - currentPeak) > 150)
                                    {
                                        _globalEngine.GhostLimiterCandidateRpm = currentPeak;
                                        _globalEngine.GhostLimiterCandidateTime = now;
                                        _globalEngine.GhostLimiterHits = 1;
                                        _isDataDirty = true;
                                    }
                                }
                                gs.GhostLimiterAccumulator = 0;
                            }
                        }
                        else 
                        { 
                            gs.GhostLimiterAccumulator = Math.Max(0, gs.GhostLimiterAccumulator - (_frameDt * 0.25)); 
                        }
                    }
                    else { gs.GhostLimiterAccumulator = Math.Max(0, gs.GhostLimiterAccumulator - (_frameDt * 0.25)); }
                }
                else { gs.GhostLimiterAccumulator = 0; gs.GhostLimiterBreakAccumulator = 0; }

                int currentRpmBand = (int)Math.Floor(currentRpm / 50.0);
                bool isRecentLearn = (now - _lastLearnedTime) < 0.35;
                if (!isRecentLearn)
                {
                    _lastLearnedBand = -1;
                    _lastLearnedPower = 0;
                }

                double actualLimiterLocal = _globalEngine.GhostLimiterRpm > 0 ? _globalEngine.GhostLimiterRpm : Math.Max(engineMaxRpm, _globalEngine.MaxMappedBand * 50.0);
                bool isLimiterProximity = currentRpm > actualLimiterLocal - 50.0 || (isLearnWot && _currentRpmAccel <= 0 && currentRpm > actualLimiterLocal - 250.0);
                
                double localMapPower = GetSmoothedValue(currentRpm, _globalEngine.PowerMap);
                bool dipDetected = (_globalEngine.GhostLimiterRpm > 0 && currentRpm > _globalEngine.GhostLimiterRpm - 150 && _currentRpmAccel <= 0)
                                   || (_lastLearnedPower > 0 && currentPower < _lastLearnedPower * 0.84)
                                   || (localMapPower > 0 && currentPower < localMapPower * 0.88);
                
                bool filtersOk = isGrounded && !inPuddle && !hasDamage && !isReboundDetected && !isRumble && isDrivetrainEngaged && isLearnWot && !isAfterAnyShift && tractionOk;

                if (filtersOk && currentPower > 5000.0 && !dipDetected && !isLimiterProximity)
                {
                    _cleanWotFrames++;
                }
                else
                {
                    _cleanWotFrames = 0;
                }

                if (_cleanWotFrames >= 2 && currentPower > 5000.0)
                {
                    double currentBoost = Math.Max(0.0, ReadDouble(pm, "DataCorePlugin.GameRawData.Boost")); 
                    if (currentBoost > _globalEngine.MaxBoost) _globalEngine.MaxBoost = currentBoost;

                    bool boostOk = true;
                    if (_globalEngine.MaxBoost > 3.0 && currentRpm >= 2800.0)
                    {
                        double expectedBoost = GetInterpolatedValue(currentRpm, _globalEngine.BoostMap);
                        double minRequiredBoost = expectedBoost > 0
                            ? Math.Max(expectedBoost * 0.88, _globalEngine.MaxBoost * 0.78)
                            : _globalEngine.MaxBoost * 0.78;
                        boostOk = currentBoost >= minRequiredBoost;
                    }

                    if (boostOk) 
                    {
                        if (currentRpmBand >= 0 && currentRpmBand <= 400)
                        {
                            // Interpolacja tylko dla małych kroków (<= 5 pasm = 250 RPM), żeby nie rysować sztucznych linii przez pół obrotomierza
                            if (_lastLearnedBand > 0 && currentRpmBand > _lastLearnedBand && (currentRpmBand - _lastLearnedBand) <= 5)
                            {
                                if (_lastLearnedBand < _globalEngine.MinMappedBand) _globalEngine.MinMappedBand = _lastLearnedBand;
                                if (currentRpmBand > _globalEngine.MaxMappedBand) _globalEngine.MaxMappedBand = currentRpmBand;

                                for (int i = _lastLearnedBand + 1; i <= currentRpmBand; i++)
                                {
                                    double t = (double)(i - _lastLearnedBand) / (currentRpmBand - _lastLearnedBand);
                                    double sweepPower = _lastLearnedPower + t * (currentPower - _lastLearnedPower);
                                    double sweepTorque = _lastLearnedTorque + t * (currentTorque - _lastLearnedTorque);

                                    _globalEngine.BandSamples[i]++;
                                    double existingP = _globalEngine.PowerMap[i];
                                    // Jeśli nowa próbka ma wyraźnie wyższą moc (pełne doładowanie vs wcześniejszy spool-up), nadpisz szybko (0.80)
                                    double alpha = existingP == 0 ? 1.0 : (sweepPower > existingP * 1.03 ? 0.80 : (sweepPower >= existingP ? 0.45 : 0.04));

                                    _globalEngine.PowerMap[i] = existingP == 0 ? sweepPower : (existingP * (1.0 - alpha)) + (sweepPower * alpha);
                                    _globalEngine.TorqueMap[i] = _globalEngine.TorqueMap[i] == 0 ? sweepTorque : (_globalEngine.TorqueMap[i] * (1.0 - alpha)) + (sweepTorque * alpha);
                                    _globalEngine.BoostMap[i] = _globalEngine.BoostMap[i] == 0 ? currentBoost : Math.Max(_globalEngine.BoostMap[i], currentBoost);
                                }
                            }
                            else
                            {
                                if (currentRpmBand < _globalEngine.MinMappedBand) _globalEngine.MinMappedBand = currentRpmBand;
                                if (currentRpmBand > _globalEngine.MaxMappedBand) _globalEngine.MaxMappedBand = currentRpmBand;

                                _globalEngine.BandSamples[currentRpmBand]++;
                                double existingP = _globalEngine.PowerMap[currentRpmBand];
                                double alpha = existingP == 0 ? 1.0 : (currentPower > existingP * 1.03 ? 0.80 : (currentPower >= existingP ? 0.45 : 0.04));

                                _globalEngine.PowerMap[currentRpmBand] = existingP == 0 ? currentPower : (existingP * (1.0 - alpha)) + (currentPower * alpha);
                                _globalEngine.TorqueMap[currentRpmBand] = _globalEngine.TorqueMap[currentRpmBand] == 0 ? currentTorque : (_globalEngine.TorqueMap[currentRpmBand] * (1.0 - alpha)) + (currentTorque * alpha);
                                _globalEngine.BoostMap[currentRpmBand] = _globalEngine.BoostMap[currentRpmBand] == 0 ? currentBoost : Math.Max(_globalEngine.BoostMap[currentRpmBand], currentBoost);
                            }

                            _lastLearnedTime = now;
                            _lastLearnedBand = currentRpmBand;
                            _lastLearnedPower = currentPower;
                            _lastLearnedTorque = currentTorque;
                            _isDataDirty = true;
                            _mathNeedsEval = true;
                        }
                    }
                }
                else if (!filtersOk || dipDetected)
                {
                    _lastLearnedBand = -1;
                    _lastLearnedPower = 0;
                    _lastLearnedTime = -100;
                }

                if (currentRatio > 0 && accel >= RatioLearnAccelThreshold && isDrivetrainEngaged && !isAfterShiftForRatio && Math.Abs(_currentRpmAccel) < 4500.0)
                {
                    gs.RatioHistory[gs.RatioHistoryIdx] = currentRatio;
                    gs.RatioHistoryIdx = (gs.RatioHistoryIdx + 1) % 25;
                    if (gs.RatioSamples < 25) gs.RatioSamples++;
                    
                    if (gs.RatioSamples >= 10)
                    {
                        int samplesToTake = gs.RatioSamples;
                        Array.Copy(gs.RatioHistory, _ratioScratch, samplesToTake);
                        Array.Sort(_ratioScratch, 0, samplesToTake);
                        double median = _ratioScratch[samplesToTake / 2];

                        double p25 = _ratioScratch[samplesToTake / 4];
                        double p75 = _ratioScratch[(samplesToTake * 3) / 4];
                        bool isTightCluster = median > 0 && ((p75 - p25) / median) < 0.012;

                        if (isTightCluster)
                        {
                            if (gs.SavedRpmPerRadS > 0)
                            {
                                // NATYCHMIASTOWA AUTOKOREKTA W GÓRĘ: Jeśli wcześniej bieg zapisał się na uślizgu (np. 36.05 zamiast 43.32),
                                // a teraz jedziemy z idealną trakcją (< 4% uślizgu) i widzimy wyższe przełożenie mechaniczne, od razu je naprawiamy!
                                if (maxAllSlip < 0.04 && median > gs.SavedRpmPerRadS * 1.025 && IsRatioMonotonicAcrossAllGears(currentGear, median))
                                {
                                    gs.RpmPerRadS = median;
                                    gs.SavedRpmPerRadS = median;
                                    gs.RatioMismatchCount = 0;
                                    if (!_store.GearRatios.ContainsKey(_key)) _store.GearRatios[_key] = new Dictionary<int, double>();
                                    _store.GearRatios[_key][currentGear] = gs.RpmPerRadS;
                                    _isDataDirty = true;
                                    _mathNeedsEval = true;
                                }
                                else if (maxAllSlip < 0.04 && samplesToTake >= 18 && Math.Abs(median - gs.SavedRpmPerRadS) / gs.SavedRpmPerRadS > 0.085)
                                {
                                    gs.RatioMismatchCount++;
                                    gs.RatioSamples = 0;
                                    gs.RatioHistoryIdx = 0;
                                    
                                    int badGears = 0;
                                    foreach (var state in _gearStates.Values) if (state.RatioMismatchCount >= 3) badGears++;

                                    if (badGears >= 3)
                                    {
                                        Interlocked.Exchange(ref _resetRatiosRequested, 1);
                                    }
                                    else if (gs.RatioMismatchCount >= 4)
                                    {
                                        gs.SavedRpmPerRadS = 0;
                                        gs.RpmPerRadS = 0;
                                        gs.ShiftTriggerRpm = 0;
                                        gs.IsConfirmed = false;
                                        gs.IsMathOptimal = false;
                                        gs.TriggerIsEdge = false;
                                        gs.TriggerIsGhost = false;
                                        gs.RatioMismatchCount = 0;
                                        if (_store.AdvancedGears.ContainsKey(_key)) _store.AdvancedGears[_key].Remove(currentGear);
                                        if (_store.GearRatios.ContainsKey(_key)) _store.GearRatios[_key].Remove(currentGear);
                                        _isDataDirty = true;
                                        _mathNeedsEval = true;
                                    }
                                }
                                else if (maxAllSlip < 0.04 && Math.Abs(median - gs.SavedRpmPerRadS) / gs.SavedRpmPerRadS <= 0.025)
                                {
                                    gs.RatioMismatchCount = 0;
                                    gs.RpmPerRadS = (gs.SavedRpmPerRadS * 0.90) + (median * 0.10);
                                    gs.SavedRpmPerRadS = gs.RpmPerRadS;
                                    if (_store.GearRatios.ContainsKey(_key)) _store.GearRatios[_key][currentGear] = gs.SavedRpmPerRadS;
                                }
                            }
                            else if (IsRatioMonotonicAcrossAllGears(currentGear, median))
                            {
                                gs.RpmPerRadS = median;
                                gs.SavedRpmPerRadS = median;
                                gs.RatioMismatchCount = 0;
                                if (!_store.GearRatios.ContainsKey(_key)) _store.GearRatios[_key] = new Dictionary<int, double>();
                                _store.GearRatios[_key][currentGear] = gs.RpmPerRadS;
                                _isDataDirty = true;
                                _mathNeedsEval = true;
                            }
                        }
                    }
                }

                if (_mathNeedsEval && (now - _lastMathEvalTime) > MathEvalIntervalSeconds)
                {
                    EvaluateAllMathIntersections(accel, currentGear, engineMaxRpm, currentRpm, actualLimiterLocal);
                    _lastMathEvalTime = now;
                    _mathNeedsEval = false;
                }
            }

            double currentLatG = accelX / 9.81;
            double currentLonG = accelZ / 9.81;
            double magnitudeG = Math.Sqrt((currentLatG * currentLatG) + (currentLonG * currentLonG));

            double actualLimiter = engineMaxRpm;
            lock (_lock)
            {
                actualLimiter = _globalEngine.GhostLimiterRpm > 0 ? _globalEngine.GhostLimiterRpm : Math.Max(engineMaxRpm, _globalEngine.MaxMappedBand * 50.0);
            }

            UpdateOutputsAndState(currentOrdinal, currentGear, engineIdleRpm, engineMaxRpm, currentRpm, now, isDrivetrainDecoupled, accel, velX, velZ, tractionOk, brake, steer, magnitudeG, maxDrivenSlipRatio, actualLimiter, susNormFL, susNormFR, susNormRL, susNormRR, combinedSlipRL, combinedSlipRR, currentOversteer, angularVelY, drivetrainType);
        }

        private void UpdateOutputsAndState(int currentOrdinal, int currentGear, double engineIdleRpm, double engineMaxRpm, double currentRpm, double now, bool isDrivetrainDecoupled, double accel, double velX, double velZ, bool tractionOk, double brake, double steer, double magnitudeG, double maxDrivenSlipRatio, double actualLimiter, double susNormFL, double susNormFR, double susNormRL, double susNormRR, double combinedSlipRL, double combinedSlipRR, double currentOversteer, double angularVelY, Drivetrain drivetrainType)
        {
            bool isLearned = false;
            double trigger = engineMaxRpm; 
            double downshiftTrigger = engineMaxRpm * 0.6;
            double speedKmh = Math.Sqrt((velX * velX) + (velZ * velZ)) * 3.6;

            lock (_lock)
            {
                double frontSusAvg = (susNormFL + susNormFR) * 0.5;
                double rearSusAvg = (susNormRL + susNormRR) * 0.5;
                double currentRawDive = frontSusAvg - rearSusAvg;

                bool isCruising = brake < BrakeDeadzone && Math.Abs(_speedTrendKmhPerS) < 2.0 && Math.Abs(steer) < 15.0 && speedKmh > 40.0 && tractionOk;
                
                if (isCruising && _frameDt > 0)
                {
                    if (!_isChassisBaseInitialized)
                    {
                        _staticChassisDiveBase = currentRawDive;
                        _isChassisBaseInitialized = true;
                    }
                    else
                    {
                        double tauSag = 2.0; 
                        double alphaSag = 1.0 - Math.Exp(-_frameDt / tauSag);
                        _staticChassisDiveBase = (_staticChassisDiveBase * (1.0 - alphaSag)) + (currentRawDive * alphaSag);
                    }
                }
                else if (!_isChassisBaseInitialized && speedKmh > 10.0 && brake < BrakeDeadzone)
                {
                    _staticChassisDiveBase = currentRawDive;
                    _isChassisBaseInitialized = true;
                }

                double dynamicDive = Math.Max(0.0, currentRawDive - _staticChassisDiveBase);
                _out.DbgChassisDive = dynamicDive;

                _out.IsGripLost = (magnitudeG > 1.25) || (maxDrivenSlipRatio > 0.30);
                _out.AeroBottomingWarning = (susNormFL >= 0.98 || susNormFR >= 0.98 || susNormRL >= 0.98 || susNormRR >= 0.98) && speedKmh > 200.0;
                
                _out.PowerOversteerWarning = (drivetrainType != Drivetrain.FWD) && (combinedSlipRL > 0.6 || combinedSlipRR > 0.6) && (accel >= LightWotThreshold) && (currentGear >= 1);
                
                _out.OversteerSeverity = currentOversteer;
                
                _out.DbgSteerRaw = steer;
                _out.DbgYawRaw = angularVelY;
                
                double bodySlipAngle = speedKmh > 5.0 ? (Math.Atan2(velX, Math.Abs(velZ) + 0.01) * 180.0 / Math.PI) : 0;
                _out.BodySlipAngle = bodySlipAngle;

                double absBodySlipDeg = Math.Abs(bodySlipAngle);
                double bodyAngleComponent = Math.Max(0.0, (absBodySlipDeg - 2.5) / 25.0);
                double wheelSlipComponent = Math.Max(0.0, (maxDrivenSlipRatio - 0.18) / 0.82);
                double rawSlideSeverity = 0.0;

                if (speedKmh > 15.0)
                {
                    rawSlideSeverity = Math.Min(1.0, Math.Max(currentOversteer, (bodyAngleComponent * 0.65) + (wheelSlipComponent * 0.35)));
                }

                double tauSlide = rawSlideSeverity >= _smoothedSlideLevel ? 0.05 : 0.10;
                double alphaSlide = 1.0 - Math.Exp(-_frameDt / tauSlide);
                _prevSmoothedSlideLevel = _smoothedSlideLevel;
                _smoothedSlideLevel = (_smoothedSlideLevel * (1.0 - alphaSlide)) + (rawSlideSeverity * alphaSlide);

                double instSlideRate = _frameDt > 0.0005 ? (_smoothedSlideLevel - _prevSmoothedSlideLevel) / _frameDt : 0.0;
                double alphaTrend = 1.0 - Math.Exp(-_frameDt / 0.12);
                _smoothedSlideTrend = (_smoothedSlideTrend * (1.0 - alphaTrend)) + (instSlideRate * alphaTrend);

                bool isSlidingActive = _smoothedSlideLevel > 0.08;
                bool isDeepening = isSlidingActive && (_smoothedSlideTrend > 0.10 || (_smoothedSlideLevel > 0.65 && _smoothedSlideTrend > -0.03));
                bool isRecovering = isSlidingActive && !isDeepening && (_smoothedSlideTrend < -0.08);

                _out.SlideSeverity = Math.Round(_smoothedSlideLevel, 4);
                _out.SlideTrend = Math.Round(Math.Max(-1.0, Math.Min(1.0, _smoothedSlideTrend)), 4);
                _out.IsSlideDeepening = isDeepening;
                _out.IsSlideRecovering = isRecovering;

                if (!isSlidingActive)
                {
                    _out.SlideStateText = "GRIP";
                    _out.SlideBarColor = "#2ECC71";
                }
                else if (isDeepening)
                {
                    _out.SlideStateText = "▲ DEEPENING";
                    _out.SlideBarColor = "#E74C3C";
                }
                else if (isRecovering)
                {
                    _out.SlideStateText = "◄ RECOVERING";
                    _out.SlideBarColor = "#2ECC71";
                }
                else
                {
                    _out.SlideStateText = "● HOLDING";
                    _out.SlideBarColor = "#FF9900";
                }

                _out.CounterSteerSeverity = _out.SlideSeverity;
                _out.IsCounterSteering = isSlidingActive;

                bool isTopGear = false;
                if (currentGear >= 4 && _gearStates.TryGetValue(currentGear, out var cgTop) && cgTop.SavedRpmPerRadS > 0 && cgTop.IsConfirmed)
                {
                    int maxKnownGear = currentGear;
                    foreach (var k in _gearStates) if (k.Value.SavedRpmPerRadS > 0 && k.Key > maxKnownGear) maxKnownGear = k.Key;

                    if (maxKnownGear == currentGear)
                    {
                        isTopGear = true;
                    }
                    else if (maxKnownGear == currentGear + 1 && _gearStates.TryGetValue(maxKnownGear, out var ngTop) && ngTop.SavedRpmPerRadS > 0)
                    {
                        double dropRatio = ngTop.SavedRpmPerRadS / cgTop.SavedRpmPerRadS;
                        if (dropRatio >= 0.88) isTopGear = true; 
                    }
                }
                _out.IsTopGear = isTopGear;

                double rawLiveBuffer = GetDynamicReactionBuffer(Math.Max(0, _currentRpmAccel), currentGear);
                if (!tractionOk) rawLiveBuffer = Math.Max(rawLiveBuffer, 100.0); 

                double tauUI = 0.10; 
                double alphaUI = 1.0 - Math.Exp(-_frameDt / tauUI);

                _smoothedLiveBuffer = (_smoothedLiveBuffer * (1.0 - alphaUI)) + (rawLiveBuffer * alphaUI);
                double liveBuffer = _smoothedLiveBuffer;

                if (_gearStates.TryGetValue(currentGear, out var currentGs) && (currentGs.IsConfirmed || currentGs.TriggerIsEdge))
                {
                    isLearned = true; 
                    double realCurrentShift = GetRealShiftRpm(currentGs, actualLimiter);
                    
                    if (realCurrentShift > 0)
                    {
                        trigger = realCurrentShift - liveBuffer;
                        
                        if (currentGear > 1 && _gearStates.TryGetValue(currentGear - 1, out var prevGs) && currentGs.RpmPerRadS > 0 && prevGs.RpmPerRadS > 0 && prevGs.PowerPeakCaught)
                        {
                            double dropFromPrev = currentGs.RpmPerRadS / prevGs.RpmPerRadS;
                            double realPrevShift = GetRealShiftRpm(prevGs, actualLimiter);
                            
                            if (realPrevShift > 0)
                            {
                                double baseStart = realPrevShift * SafeLandingRpmMultiplier;
                                
                                double powerDropMod = 1.0;
                                if (_globalEngine.MaxPower > 0)
                                {
                                    double powerAtShift = GetSmoothedValue(realPrevShift, _globalEngine.PowerMap);
                                    double powerAtLand = GetSmoothedValue(realPrevShift * dropFromPrev, _globalEngine.PowerMap);
                                    if (powerAtShift > 0 && powerAtLand > 0 && powerAtLand < powerAtShift * 0.9)
                                    {
                                        powerDropMod = 0.95;
                                    }
                                }

                                downshiftTrigger = Math.Min(baseStart * dropFromPrev * powerDropMod, realCurrentShift * MaxAllowedDownshiftMultiplier);
                            }
                        }
                    }
                    else
                    {
                        trigger = actualLimiter - liveBuffer - 250.0;
                    }
                }
                else
                {
                    trigger = actualLimiter - liveBuffer - 250.0;
                }

                double brakeIntensity = brake / 255.0;
                double diveBoost = (brake > BrakeDeadzone) ? Math.Min(250.0, dynamicDive * 400.0) : 0.0;
                double dynamicDownshiftBoost = (brakeIntensity * 600.0) + diveBoost;
                downshiftTrigger += dynamicDownshiftBoost;

                if (currentGear > 1 && _gearStates.TryGetValue(currentGear, out var curGs) && _gearStates.TryGetValue(currentGear - 1, out var pGs) && curGs.RpmPerRadS > 0 && pGs.RpmPerRadS > 0)
                {
                    double dropFromPrev = curGs.RpmPerRadS / pGs.RpmPerRadS;
                    double hardCeiling = (actualLimiter - 150.0) * dropFromPrev;
                    downshiftTrigger = Math.Min(downshiftTrigger, hardCeiling);
                }

                trigger = Math.Min(trigger, actualLimiter - 50.0);

                _out.CarKey = _key;
                _out.CarName = ResolveCarName(currentOrdinal, _key);
                _out.IsShiftBeepEnabled = _store.ShiftBeepEnabled;
                _out.IsTractionGood = tractionOk;
                _out.RpmAtMaxPower = _globalEngine.RpmAtMaxPower;
                _out.RpmAtMaxTorque = _globalEngine.RpmAtMaxTorque;
                
                double optimalShift = actualLimiter - 50.0;
                if (_gearStates.TryGetValue(currentGear, out var optimalGs) && optimalGs.ShiftTriggerRpm > 0)
                {
                    double tempOptimal = GetRealShiftRpm(optimalGs, actualLimiter);
                    if (tempOptimal > 0) optimalShift = tempOptimal;
                }
                _out.OptimalShiftRpm = optimalShift;

                _out.MaxUsableRpm = trigger;
                _out.ShiftTriggerRpm = trigger;
                _out.DownshiftTriggerRpm = downshiftTrigger;
                
                _out.IsDownshiftSafe = (currentGear > 1) && (currentRpm <= downshiftTrigger);

                bool isAfterAnyShift = (now - _lastDownshiftTime < DownshiftCooldownSeconds) || 
                                       (now - _lastUpshiftTime < UpshiftCooldownSeconds) || 
                                       (now - _lastDecoupledTime < ClutchSettleSeconds);

                if (isAfterAnyShift) 
                {
                    _lastDownshiftSuggested = false;
                    _climbDownshiftActive = false;
                }
                else
                {
                    if (_out.IsDownshiftSafe && (currentRpm <= downshiftTrigger - 250) && speedKmh > 10.0) 
                        _lastDownshiftSuggested = true;
                    else if (!_out.IsDownshiftSafe || currentRpm > downshiftTrigger - 100) 
                        _lastDownshiftSuggested = false;

                    if (currentGear > 1) 
                    {
                        if (_gearStates.TryGetValue(currentGear, out var cg) && _gearStates.TryGetValue(currentGear - 1, out var pg) && cg.RpmPerRadS > 0 && pg.RpmPerRadS > 0)
                        {
                            double landRpm = currentRpm * (pg.RpmPerRadS / cg.RpmPerRadS);
                            double pNow = GetSmoothedValue(currentRpm, _globalEngine.PowerMap);
                            double pLand = GetSmoothedValue(landRpm, _globalEngine.PowerMap);
                            double prevShift = GetRealShiftRpm(pg, actualLimiter);
                            
                            if (prevShift > 0)
                            {
                                bool enter = accel >= LightWotThreshold && _speedTrendKmhPerS <= 1.0 
                                             && pNow > 0 && pLand > 0 && pLand > pNow * 1.05 && landRpm < prevShift - 300.0 
                                             && maxDrivenSlipRatio < 0.15;
                                             
                                bool stay  = accel >= LiftThreshold && pNow > 0 && pLand > 0 && pLand > pNow * 1.02 && landRpm < prevShift - 200.0;
                                
                                _climbDownshiftActive = _climbDownshiftActive ? stay : enter;
                                
                                if (_climbDownshiftActive) 
                                {
                                    _lastDownshiftSuggested = true;
                                }
                            }
                            else
                            {
                                _climbDownshiftActive = false;
                            }
                        }
                        else
                        {
                            _climbDownshiftActive = false;
                        }
                    }
                    else
                    {
                        _climbDownshiftActive = false;
                    }
                }
                
                _out.IsDownshiftSuggested = _lastDownshiftSuggested;
                
                _out.IsLearningComplete = isLearned;

                // Pełne okno mapowania od ~3000 RPM (lub 0.45 * RpmAtMaxPower) do actualLimiter - 250 RPM
                double refPeakRpm = _globalEngine.RpmAtMaxPower > 2000 ? _globalEngine.RpmAtMaxPower : engineMaxRpm * 0.80;
                double baseRpmProgress = Math.Min(3000.0, Math.Max(1500.0, refPeakRpm * 0.45));
                double denseUpperLimit = actualLimiter - 250.0;
                
                if (denseUpperLimit > baseRpmProgress)
                {
                    int startBand = Math.Max(0, (int)Math.Floor(baseRpmProgress / 50.0));
                    int endBand = Math.Min(400, (int)Math.Ceiling(denseUpperLimit / 50.0));
                    int expectedBands = Math.Max(1, endBand - startBand + 1);
                    int filledBands = 0;

                    for (int i = startBand; i <= endBand; i++)
                    {
                        if (_globalEngine.PowerMap[i] > 0) filledBands++;
                    }
                    
                    double rawPct = ((double)filledBands / expectedBands) * 100.0;
                    _out.LiveMappingProgress = rawPct >= 99.0 ? 100.0 : Math.Min(100.0, Math.Max(0.0, rawPct));
                    _globalEngine.MappingProgress = _out.LiveMappingProgress;
                }
                else
                {
                    _out.LiveMappingProgress = 0;
                    _globalEngine.MappingProgress = 0;
                }

                double range = denseUpperLimit - baseRpmProgress;
                if (now - _lastMapScanTime >= 0.5)
                {
                    _lastMapScanTime = now;
                    if (range > 1000)
                    {
                        char[] bar = new char[20];
                        double rpmPerBlock = range / 20.0;
                        for (int b = 0; b < 20; b++)
                        {
                            double blockStartRpm = baseRpmProgress + (b * rpmPerBlock);
                            double blockEndRpm = blockStartRpm + rpmPerBlock;
                            int startBand = Math.Max(0, (int)Math.Floor(blockStartRpm / 50.0));
                            int endBand = Math.Min(400, (int)Math.Ceiling(blockEndRpm / 50.0));
                            
                            bool hasData = false;
                            for (int i = startBand; i <= endBand; i++)
                            {
                                if (_globalEngine.PowerMap[i] > 0)
                                {
                                    hasData = true;
                                    break;
                                }
                            }
                            bar[b] = hasData ? '█' : '░';
                        }
                        _out.MapCoverageBar = "[" + new string(bar) + "]";
                    }
                }
                
                bool isEngineMapTrusted = _out.LiveMappingProgress >= 99.5;
                _out.EngineMapStatus = isEngineMapTrusted ? "OK" : "LEARN";

                for (int i = 1; i <= 10; i++)
                {
                    if (_gearStates.TryGetValue(i, out var gState))
                    {
                        if (gState.IsConfirmed)
                        {
                            if (gState.IsMathOptimal)
                            {
                                _out.GearText[i] = LabelsPeak[i];
                                _out.GearColor[i] = "#4CAF50"; 
                            }
                            else
                            {
                                _out.GearText[i] = LabelsLimitConfirmed[i];
                                _out.GearColor[i] = "#4CAF50"; 
                            }
                        }
                        else if (gState.TriggerIsEdge)
                        {
                            _out.GearText[i] = LabelsLimit[i];
                            _out.GearColor[i] = "#F39C12"; 
                        }
                        else if (gState.PowerPeakCaught && gState.ShiftTriggerRpm > 0)
                        {
                            _out.GearText[i] = LabelsLearn[i];
                            _out.GearColor[i] = "#F1C40F"; 
                        }
                        else if (gState.RpmPerRadS > 0)
                        {
                            _out.GearText[i] = LabelsRatio[i];
                            _out.GearColor[i] = "#3498DB"; 
                        }
                        else
                        {
                            _out.GearText[i] = LabelsZero[i];
                            _out.GearColor[i] = "#E74C3C"; 
                        }
                    }
                    else
                    {
                        _out.GearText[i] = LabelsZero[i];
                        _out.GearColor[i] = "#E74C3C"; 
                    }
                }

                if (currentGear >= 1 && currentGear <= 10)
                {
                    _out.CurrentGearStatusText = _out.GearText[currentGear];
                    _out.CurrentGearColor = _out.GearColor[currentGear];
                }
                else
                {
                    _out.CurrentGearStatusText = "-";
                    _out.CurrentGearColor = "#00000000";
                }
            }

            double effectiveTarget = Math.Max(engineIdleRpm + 500.0, trigger);
            double ledSpan = Math.Max(1200.0, (effectiveTarget - engineIdleRpm) * 0.28);
            double startLight = Math.Max(engineIdleRpm + 200.0, effectiveTarget - ledSpan);

            double rawProximity = 0.0;
            if (effectiveTarget > startLight && currentGear >= 1 && !isDrivetrainDecoupled)
            {
                rawProximity = (currentRpm - startLight) / (effectiveTarget - startLight);
                if (rawProximity < 0.0) rawProximity = 0.0;
                if (rawProximity > 1.0) rawProximity = 1.0;
            }

            if (currentGear < 1 || isDrivetrainDecoupled)
            {
                _smoothedLedProximity = 0.0;
                _currentLedStage = 0;
                _shiftLightActive = false;
            }
            else
            {
                bool isPushingWot = accel >= LightWotThreshold && brake < BrakeDeadzone;
                if (rawProximity >= _smoothedLedProximity)
                {
                    double alphaUp = 1.0 - Math.Exp(-_frameDt / 0.015);
                    _smoothedLedProximity += (rawProximity - _smoothedLedProximity) * alphaUp;
                }
                else if (isPushingWot && _currentRpmAccel >= -80.0 && rawProximity > _smoothedLedProximity - 0.06)
                {
                }
                else
                {
                    double alphaDown = 1.0 - Math.Exp(-_frameDt / 0.080);
                    _smoothedLedProximity += (rawProximity - _smoothedLedProximity) * alphaDown;
                }
            }

            double scaledLeds = _smoothedLedProximity * 12.0;
            if (scaledLeds >= _currentLedStage + 0.95)
            {
                _currentLedStage = Math.Min(12, (int)Math.Floor(scaledLeds + 0.05));
            }
            else if (scaledLeds < _currentLedStage - 0.20)
            {
                _currentLedStage = Math.Max(0, (int)Math.Floor(scaledLeds + 0.20));
            }

            bool wasShiftLightActive = _shiftLightActive;
            if (!isDrivetrainDecoupled && currentRpm >= effectiveTarget && accel >= LightWotThreshold)
            {
                _shiftLightActive = true;
            }
            if (_shiftLightActive && (isDrivetrainDecoupled || currentRpm < effectiveTarget - 200.0 || accel < LiftThreshold))
            {
                _shiftLightActive = false;
            }

            if (_shiftLightActive && !wasShiftLightActive)
            {
                TriggerShiftBeepSound(now);
            }

            if (_shiftLightActive)
            {
                _currentLedStage = 12;
                _smoothedLedProximity = 1.0;
            }

            bool localDirty;
            lock (_lock)
            {
                _out.RevLimiterProximity = Math.Round(_smoothedLedProximity, 4);
                _out.LedActiveCount = _currentLedStage;
                _out.ShiftLightFlash = _shiftLightActive;
                localDirty = _isDataDirty;
            }

            if (localDirty && (now - _lastSaveTime) > SaveIntervalSeconds) TriggerAsyncSave();
        }
    }

    public class SavedGearData {
        public double Rpm { get; set; }
        public bool Opt { get; set; }
        public bool Edg { get; set; }
        public bool Gho { get; set; }
    }

    class SavedEngineProfile { 
        public double[] PowerMap { get; set; } = new double[401]; 
        public double[] TorqueMap { get; set; } = new double[401]; 
        public double[] BoostMap { get; set; } = new double[401]; 
        public double MaxPower { get; set; } = 0; 
        public double RpmAtMaxPower { get; set; } = 0; 
        public double MaxTorque { get; set; } = 0; 
        public double RpmAtMaxTorque { get; set; } = 0; 
        public double MaxBoost { get; set; } = 0;
        public double GhostLimiterRpm { get; set; } = 0; 
        public double MappingProgress { get; set; } = 0;
        public double Best0To100 { get; set; } = 0;
        public double Best100To200 { get; set; } = 0;
        public double Best0To200 { get; set; } = 0;
    }
    
    class CarStore {
        public int SchemaVersion { get; set; } = 2;
        public bool ShiftBeepEnabled { get; set; } = false;
        public double ReactionBufferFactor { get; set; } = 0.25;
        public Dictionary<string, string> CarNames { get; set; } = new Dictionary<string, string>();
        public Dictionary<string, Dictionary<int, double>> GearRatios { get; set; } = new Dictionary<string, Dictionary<int, double>>(); 
        public Dictionary<string, SavedEngineProfile> EngineProfiles { get; set; } = new Dictionary<string, SavedEngineProfile>();
        public Dictionary<string, Dictionary<int, SavedGearData>> AdvancedGears { get; set; } = new Dictionary<string, Dictionary<int, SavedGearData>>();
    }
}
