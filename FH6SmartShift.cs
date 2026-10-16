using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using GameReaderCommon;
using Newtonsoft.Json;
using SimHub.Plugins;

namespace FH6SmartShift
{
    [PluginName("FH6SmartShift")]
    [PluginDescription("The Architect's Masterpiece - v1.0.0 (Pure Math & Telemetry Monolith)")]
    [PluginAuthor("mtowarek01")]
    public class FH6SmartShift : IPlugin, IDataPlugin
    {
        private const string StoreKey = "fh6SmartShiftingData"; 

        private const double DownshiftCooldownSeconds = 0.45; 
        private const double UpshiftCooldownSeconds = 0.25; 
        private const double ClutchSettleSeconds = 0.12; 
        private const double SaveIntervalSeconds = 15.0; 
        private const double MathEvalIntervalSeconds = 0.15; 

        private const double SafeLandingRpmMultiplier = 0.93; 
        private const double MaxAllowedDownshiftMultiplier = 0.88; 
        
        private const double LearnWotThreshold = 240.0;
        private const double RatioLearnAccelThreshold = 75.0;
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
            public double GhostLimiterRpm = 0; 
            public double GhostLimiterCandidateRpm = 0;
            public double GhostLimiterCandidateTime = 0; 
            public int GhostLimiterHits = 0; 
            public int[] BandSamples = new int[401];
            public int MinMappedBand = 400; 
            public int MaxMappedBand = 0;
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

        private int _resetCurrentRequested = 0;
        private int _resetAllRequested = 0;
        private int _resetLimitGearsRequested = 0;
        private int _resetRatiosRequested = 0; 

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
            public string CarKey = "";
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
            public double DbgSteerRaw = 0;
            public double DbgYawRaw = 0;
            public double DbgChassisDive = 0.0;
            public string EngineMapStatus = "LEARN"; 
            public string MapCoverageBar = "[░░░░░░░░░░░░░░░░░░░░]";
            public string[] GearText = new string[11];
            public string[] GearColor = new string[11];

            public Outputs()
            {
                for(int i = 0; i <= 10; i++) { GearText[i] = LabelsZero[i]; GearColor[i] = "#E74C3C"; }
            }
        }
        
        private Outputs _out = new Outputs();

        private static string GetReportFilePath()
        {
            string dir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "PluginsData", "Common");
            if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
            return Path.Combine(dir, "FH6SmartShift_GarageReport.html");
        }

        public void Init(PluginManager pluginManager)
        {
            try {
                string dir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "PluginsData", "Common");
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
            if (_store.SchemaVersion < 2) { _store.SchemaVersion = 2; this.SaveCommonSettings(StoreKey, _store); }
            
            bool globalBoostCleaned = false;
            foreach (var ep in _store.EngineProfiles.Values)
            {
                if (ep.BoostMap != null)
                {
                    for (int i = 0; i < ep.BoostMap.Length; i++)
                    {
                        if (ep.BoostMap[i] < 0)
                        {
                            ep.BoostMap[i] = 0;
                            globalBoostCleaned = true;
                        }
                    }
                }
            }
            if (globalBoostCleaned) _isDataDirty = true;

            _snapshotStore = CloneCarStore(_store);
            
            try { WriteHtmlGarageReport(_snapshotStore, ""); }
            catch (Exception ex) { Debug.WriteLine("[FH6SmartShift] Initial HTML report error: " + ex.Message); }
            
            _appTime.Start();
            _dtStopwatch.Start();
            
            pluginManager.AddAction("ResetPowerMemory", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetCurrentRequested, 1));
            pluginManager.AddAction("ResetAllCarsMemory", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetAllRequested, 1));
            pluginManager.AddAction("ResetLimitGears", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetLimitGearsRequested, 1));
            pluginManager.AddAction("ResetGearRatios", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetRatiosRequested, 1));
            pluginManager.AddAction("OpenDynoReport", this.GetType(), (a, b) => OpenGarageReportInBrowser());

            this.AttachDelegate("MaxUsableRpm", () => _out.MaxUsableRpm);
            this.AttachDelegate("RevLimiterProximity", () => _out.RevLimiterProximity);
            this.AttachDelegate("LedActiveCount", () => _out.LedActiveCount);
            this.AttachDelegate("ShiftLightFlash", () => _out.ShiftLightFlash);
            this.AttachDelegate("CurrentCarKey", () => _out.CarKey);
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
<title>FH6SmartShift — Virtual Dyno &amp; Gearbox Inspector</title>
<style>
  * { box-sizing: border-box; }
  body {
    background: #0d1117;
    color: #e6edf3;
    font-family: 'Segoe UI', -apple-system, BlinkMacSystemFont, Arial, sans-serif;
    margin: 0;
    padding: 24px;
  }
  .container { max-width: 1200px; margin: 0 auto; }
  .header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-wrap: wrap;
    gap: 16px;
    background: #161b22;
    border: 1px solid #30363d;
    border-radius: 10px;
    padding: 18px 24px;
    margin-bottom: 20px;
  }
  .header h1 { margin: 0; font-size: 21px; color: #fff; }
  .controls { display: flex; gap: 12px; align-items: center; flex-wrap: wrap; }
  .file-label {
    background: #238636;
    color: #fff;
    padding: 9px 15px;
    border-radius: 6px;
    font-weight: 600;
    cursor: pointer;
    font-size: 13.5px;
  }
  .file-label:hover { background: #2ea043; }
  input[type=""file""] { display: none; }
  select {
    background: #0d1117;
    color: #e6edf3;
    border: 1px solid #30363d;
    padding: 9px 14px;
    border-radius: 6px;
    font-size: 14px;
    font-weight: 600;
    min-width: 310px;
  }
  .profile-banner {
    background: #161b22;
    border-left: 4px solid #2f81f7;
    border-top: 1px solid #30363d;
    border-right: 1px solid #30363d;
    border-bottom: 1px solid #30363d;
    border-radius: 8px;
    padding: 16px 20px;
    margin-bottom: 20px;
    font-size: 15px;
    line-height: 1.5;
  }
  .copy-btn {
    background: #21262d;
    color: #c9d1d9;
    border: 1px solid #30363d;
    padding: 5px 12px;
    border-radius: 5px;
    cursor: pointer;
    font-size: 12px;
    float: right;
  }
  .copy-btn:hover { background: #30363d; }
  .table-wrap {
    background: #ffffff;
    color: #1f2328;
    border-radius: 8px;
    overflow: hidden;
    box-shadow: 0 8px 24px rgba(0,0,0,0.4);
    margin-bottom: 24px;
  }
  table {
    width: 100%;
    border-collapse: collapse;
    font-size: 14.5px;
  }
  th {
    background: #f6f8fa;
    color: #1f2328;
    font-weight: 600;
    text-align: left;
    padding: 10px 14px;
    border: 1px solid #d0d7de;
  }
  td {
    padding: 9px 14px;
    border: 1px solid #d0d7de;
  }
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
  .legend { font-size: 13px; color: #8b949e; display: flex; gap: 16px; }
  .leg-hp { color: #3fb950; font-weight: 700; }
  .leg-nm { color: #58a6ff; font-weight: 700; }
  canvas { width: 100%; height: 320px; display: block; }
</style>
</head>
<body>

<div class=""container"">
  <div class=""header"">
    <h1>🏎️ FH6SmartShift — Virtual Dyno &amp; Gearbox Inspector</h1>
    <div class=""controls"">
      <select id=""carSelect"" onchange=""renderCar(this.value)""></select>
      <label class=""file-label"">
        📂 Load External JSON
        <input type=""file"" id=""jsonFile"" accept="".json"">
      </label>
    </div>
  </div>

  <div class=""profile-banner"" id=""profileSummary"">
    No car profiles saved yet. Drive a car at wide-open throttle in Forza Horizon 6 to populate the Virtual Dyno database.
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

  <div class=""chart-card"">
    <div class=""chart-header"">
      <h3>📈 401-Band Virtual Dyno Curve (Power HP &amp; Torque Nm)</h3>
      <div class=""legend"">
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

document.getElementById('jsonFile').addEventListener('change', function(e) {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = function(evt) {
    try {
      dbData = JSON.parse(evt.target.result);
      populateCarSelect();
    } catch (err) {
      alert('Invalid JSON file: ' + err.message);
    }
  };
  reader.readAsText(file);
});

function populateCarSelect() {
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
    const parts = k.split('_');
    if (parts.length === 4) {
      label = `Car #${parts[0]} | ${parts[1]} | ${dtMap[parts[2]] || parts[2]} | Gauge ${parts[3].replace('R','')} RPM (${k})`;
    }
    opt.textContent = label;
    sel.appendChild(opt);
  });

  if (keys.length > 0) {
    const target = (defaultCarKey && keys.includes(defaultCarKey)) ? defaultCarKey : keys[0];
    sel.value = target;
    renderCar(target);
  }
}

function renderCar(key) {
  const ep = (dbData.EngineProfiles && dbData.EngineProfiles[key]) || {};
  const ratios = (dbData.GearRatios && dbData.GearRatios[key]) || {};
  const adv = (dbData.AdvancedGears && dbData.AdvancedGears[key]) || {};

  const gaugeMatch = key.match(/_R(\d+)$/);
  const gaugeRpm = gaugeMatch ? parseInt(gaugeMatch[1], 10) : 8000;

  const kw = (ep.MaxPower || 0) / 1000.0;
  const hp = (ep.MaxPower || 0) / 745.7;
  const nm = ep.MaxTorque || 0;
  const rpmP = ep.RpmAtMaxPower || 0;
  const rpmT = ep.RpmAtMaxTorque || 0;

  let maxBoostPsi = 0;
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

  const limiterText = (ep.GhostLimiterRpm && ep.GhostLimiterRpm > 0)
    ? `and a detected hidden rev limiter (GhostLimiterRpm) at <strong>${ep.GhostLimiterRpm.toFixed(1)} RPM</strong> (on an ${gaugeRpm} RPM gauge cluster).`
    : `and standard rev limiter up to <strong>${actualLimiter.toFixed(0)} RPM</strong> (on an ${gaugeRpm} RPM gauge cluster).`;

  document.getElementById('profileSummary').innerHTML = `
    <button class=""copy-btn"" onclick=""copySummary()"">📋 Copy Summary</button>
    <strong>Engine &amp; Dyno Profile (<code>${key}</code>):</strong>
    Peak power of <strong>${hp.toFixed(1)} HP (${kw.toFixed(1)} kW)</strong> at <strong>${rpmP.toFixed(0)} RPM</strong>,
    peak torque of <strong>${nm.toFixed(1)} Nm</strong> at <strong>${rpmT.toFixed(0)} RPM</strong>,
    <strong>${boostText}</strong>, ${limiterText}
  `;

  const tbody = document.getElementById('gearTableBody');
  tbody.innerHTML = '';

  const gearNums = Object.keys(ratios).map(Number).filter(n => ratios[n] > 0).sort((a,b) => a - b);
  const maxGear = gearNums.length > 0 ? Math.max(...gearNums) : 6;

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

  drawDynoChart(ep);
}

function drawDynoChart(ep) {
  const canvas = document.getElementById('dynoCanvas');
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);

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

  ctx.strokeStyle = '#58a6ff';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  let started = false;
  for (let i = 1; i <= maxBand; i++) {
    if (tMap[i] <= 0) continue;
    const x = padL + ((i * 50) / maxRpmPlot) * w;
    const y = padT + h - (tMap[i] / maxNm) * h;
    if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
  }
  ctx.stroke();

  ctx.strokeStyle = '#3fb950';
  ctx.lineWidth = 3;
  ctx.beginPath();
  started = false;
  for (let i = 1; i <= maxBand; i++) {
    if (pMap[i] <= 0) continue;
    const hp = pMap[i] / 745.7;
    const x = padL + ((i * 50) / maxRpmPlot) * w;
    const y = padT + h - (hp / maxHp) * h;
    if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
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
            var clone = new CarStore { SchemaVersion = source.SchemaVersion };
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
                    PowerMap = mapCopy, TorqueMap = torqueCopy, BoostMap = boostCopy, MaxPower = engine.Value.MaxPower, RpmAtMaxPower = engine.Value.RpmAtMaxPower, MaxTorque = engine.Value.MaxTorque, RpmAtMaxTorque = engine.Value.RpmAtMaxTorque, GhostLimiterRpm = engine.Value.GhostLimiterRpm
                };
            }
            return clone;
        }

        private void UpdateSnapshotWithCurrentCar()
        {
            if (string.IsNullOrEmpty(_key)) return;
            
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
                    GhostLimiterRpm = ep.GhostLimiterRpm
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
            if (string.IsNullOrEmpty(_key) || _globalEngine == null || _globalEngine.MaxPower == 0) return;
            var profileCopy = new double[401]; var torqueCopy = new double[401]; var boostCopy = new double[401];
            Array.Copy(_globalEngine.PowerMap, profileCopy, 401); 
            Array.Copy(_globalEngine.TorqueMap, torqueCopy, 401);
            Array.Copy(_globalEngine.BoostMap, boostCopy, 401);
            _store.EngineProfiles[_key] = new SavedEngineProfile { PowerMap = profileCopy, TorqueMap = torqueCopy, BoostMap = boostCopy, MaxPower = _globalEngine.MaxPower, RpmAtMaxPower = _globalEngine.RpmAtMaxPower, MaxTorque = _globalEngine.MaxTorque, RpmAtMaxTorque = _globalEngine.RpmAtMaxTorque, GhostLimiterRpm = _globalEngine.GhostLimiterRpm };
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
            for (int i = targetBand; i >= Math.Max(0, targetBand - 6); i--) { if (map[i] > 0) { bestLower = i; break; } }
            
            int bestUpper = -1;
            for (int i = targetBand + 1; i <= Math.Min(400, targetBand + 6); i++) { if (map[i] > 0) { bestUpper = i; break; } }

            if (bestLower != -1 && bestUpper != -1)
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
                    if (_globalEngine.PowerMap[i] > 0 && _globalEngine.PowerMap[i] < avgNeighbors * 0.84)
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
                        if (_globalEngine.PowerMap[i] > 0 && _globalEngine.PowerMap[i + 1] > 0 &&
                            _globalEngine.PowerMap[i] < expected1 * 0.82 && _globalEngine.PowerMap[i + 1] < expected2 * 0.82)
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
                            healed = true;
                        }
                    }
                }
            }
            return healed;
        }

        private bool IsRatioMonotonicAcrossAllGears(int targetGear, double candidateRatio)
        {
            if (candidateRatio <= 5.0 || candidateRatio > 600.0) return false;

            foreach (var kvp in _gearStates)
            {
                int otherGear = kvp.Key;
                double otherRatio = kvp.Value.SavedRpmPerRadS;
                if (otherGear == targetGear || otherRatio <= 0) continue;

                int gearDiff = Math.Abs(targetGear - otherGear);
                double minStepFactor = Math.Pow(1.035, gearDiff);

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

            bool hasTurbo = false;
            for (int i = 0; i <= 400; i++) {
                if (_globalEngine.BoostMap[i] > 0.1) { hasTurbo = true; break; }
            }

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
            double buffer = rpmAccel * 0.25; 
            if (buffer < 50.0) return 50.0;
            if (currentGear <= 2) { if (buffer > 1500.0) return 1500.0; }
            else { if (buffer > 600.0) return 600.0; }
            return buffer; 
        }

        private void ResetOutputs()
        {
            lock (_lock)
            {
                _out.CarKey = "";
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

            if (doResetAll) 
            { 
                lock (_lock) 
                { 
                    _store = new CarStore { SchemaVersion = 2 }; 
                    _snapshotStore = new CarStore { SchemaVersion = 2 };
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

            int currentPI = (int)ReadDouble(pm, "DataCorePlugin.GameRawData.CarPerformanceIndex");
            if (currentPI <= 0) 
            { 
                ResetOutputs(); 
                return; 
            }

            double engineIdleRpm = ReadDouble(pm, "DataCorePlugin.GameRawData.EngineIdleRpm");
            int dtType = (int)ReadDouble(pm, "DataCorePlugin.GameRawData.DrivetrainType");
            int roundedMaxRpm = (int)(Math.Round(engineMaxRpm / 100.0) * 100);

            if (currentOrdinal != _lastCarOrdinal || currentPI != _lastCarPI || dtType != _lastCarDrivetrain || roundedMaxRpm != _lastCarEngineMaxRpmRounded) 
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
                    _key = $"{currentOrdinal}_PI{currentPI}_DT{dtType}_R{roundedMaxRpm}"; 

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
                        _globalEngine.GhostLimiterRpm = savedEngine.GhostLimiterRpm;

                        _globalEngine.MinMappedBand = 400; _globalEngine.MaxMappedBand = 0;
                        for(int i=0; i<401; i++) {
                            if (_globalEngine.PowerMap[i] > 0) {
                                _globalEngine.BandSamples[i] = 6; 
                                if (i < _globalEngine.MinMappedBand) _globalEngine.MinMappedBand = i;
                                if (i > _globalEngine.MaxMappedBand) _globalEngine.MaxMappedBand = i;
                            }
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
                                    double minStep = Math.Pow(1.035, diff);
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
                UpdateOutputsAndState(currentGear, engineIdleRpm, engineMaxRpm, currentRpm, now, true, accel, velX, velZ, true, brake, steer, 0.0, 0.0, engineMaxRpm, susNormFL, susNormFR, susNormRL, susNormRR, combinedSlipRL, combinedSlipRR, 0.0, angularVelY, drivetrainType);
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
            bool isRatioLearnSafe = isGrounded && !isReboundDetected && (maxAllSlip < 0.08) && (brake < BrakeDeadzone);

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

            double currentRatio = (isRatioLearnSafe && avgWheelSpeed > 16.0 && currentRpm > 1400.0) ? currentRpm / avgWheelSpeed : 0;

            double maxRearSlipAngle = Math.Max(Math.Abs(slipAngleRL), Math.Abs(slipAngleRR));
            double maxRearCombinedSlip = Math.Max(Math.Abs(combinedSlipRL), Math.Abs(combinedSlipRR));
            double currentOversteer = 0.0;
            
            if (speedKmh > 15.0)
            {
                double angleScore = Math.Max(0, (maxRearSlipAngle - 0.08) / 0.45); 
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

            bool isLowGear = currentGear >= 1 && currentGear <= 3;
            double slipTolerance = isLowGear ? 0.60 : 0.38; 
            
            bool tractionOk = maxDrivenSlipRatio < slipTolerance && maxDrivenSlipAngle < SlipAngleThreshold;

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
                    bool boostOk = true;
                    bool hasTurbo = false;
                    for (int i = 0; i <= 400; i++) {
                        if (_globalEngine.BoostMap[i] > 0.1) { hasTurbo = true; break; }
                    }

                    if (hasTurbo) 
                    {
                        double expectedBoost = GetInterpolatedValue(currentRpm, _globalEngine.BoostMap);
                        boostOk = currentBoost >= expectedBoost * 0.85;
                    }

                    if (boostOk) 
                    {
                        if (currentRpmBand >= 0 && currentRpmBand <= 400)
                        {
                            if (_lastLearnedBand > 0 && currentRpmBand > _lastLearnedBand && (currentRpmBand - _lastLearnedBand) <= 12)
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
                                    double alpha = existingP == 0 ? 1.0 : (sweepPower >= existingP ? 0.35 : 0.08);

                                    _globalEngine.PowerMap[i] = existingP == 0 ? sweepPower : (existingP * (1.0 - alpha)) + (sweepPower * alpha);
                                    _globalEngine.TorqueMap[i] = _globalEngine.TorqueMap[i] == 0 ? sweepTorque : (_globalEngine.TorqueMap[i] * (1.0 - alpha)) + (sweepTorque * alpha);
                                    _globalEngine.BoostMap[i] = _globalEngine.BoostMap[i] == 0 ? currentBoost : Math.Max(_globalEngine.BoostMap[i] * 0.95, (_globalEngine.BoostMap[i] * (1.0 - alpha)) + (currentBoost * alpha));
                                }
                            }
                            else
                            {
                                if (currentRpmBand < _globalEngine.MinMappedBand) _globalEngine.MinMappedBand = currentRpmBand;
                                if (currentRpmBand > _globalEngine.MaxMappedBand) _globalEngine.MaxMappedBand = currentRpmBand;

                                _globalEngine.BandSamples[currentRpmBand]++;
                                double existingP = _globalEngine.PowerMap[currentRpmBand];
                                double alpha = existingP == 0 ? 1.0 : (currentPower >= existingP ? 0.35 : 0.08);

                                _globalEngine.PowerMap[currentRpmBand] = existingP == 0 ? currentPower : (existingP * (1.0 - alpha)) + (currentPower * alpha);
                                _globalEngine.TorqueMap[currentRpmBand] = _globalEngine.TorqueMap[currentRpmBand] == 0 ? currentTorque : (_globalEngine.TorqueMap[currentRpmBand] * (1.0 - alpha)) + (currentTorque * alpha);
                                _globalEngine.BoostMap[currentRpmBand] = _globalEngine.BoostMap[currentRpmBand] == 0 ? currentBoost : Math.Max(_globalEngine.BoostMap[currentRpmBand] * 0.95, (_globalEngine.BoostMap[currentRpmBand] * (1.0 - alpha)) + (currentBoost * alpha));
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

                if (currentRatio > 0 && accel >= RatioLearnAccelThreshold && isDrivetrainEngaged && !isAfterAnyShift)
                {
                    gs.RatioHistory[gs.RatioHistoryIdx] = currentRatio;
                    gs.RatioHistoryIdx = (gs.RatioHistoryIdx + 1) % 25;
                    if (gs.RatioSamples < 25) gs.RatioSamples++;
                    
                    if (gs.RatioSamples >= 12)
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
                                if (Math.Abs(median - gs.SavedRpmPerRadS) / gs.SavedRpmPerRadS > 0.045)
                                {
                                    gs.RatioMismatchCount++;
                                    gs.RatioSamples = 0;
                                    gs.RatioHistoryIdx = 0;
                                    
                                    int badGears = 0;
                                    foreach (var state in _gearStates.Values) if (state.RatioMismatchCount >= 2) badGears++;

                                    if (badGears >= 2)
                                    {
                                        Interlocked.Exchange(ref _resetRatiosRequested, 1);
                                    }
                                    else if (gs.RatioMismatchCount >= 3)
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
                                else
                                {
                                    gs.RatioMismatchCount = 0;
                                    gs.RpmPerRadS = (gs.SavedRpmPerRadS * 0.85) + (median * 0.15);
                                    gs.SavedRpmPerRadS = gs.RpmPerRadS;
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

            UpdateOutputsAndState(currentGear, engineIdleRpm, engineMaxRpm, currentRpm, now, isDrivetrainDecoupled, accel, velX, velZ, tractionOk, brake, steer, magnitudeG, maxDrivenSlipRatio, actualLimiter, susNormFL, susNormFR, susNormRL, susNormRR, combinedSlipRL, combinedSlipRR, currentOversteer, angularVelY, drivetrainType);
        }

        private void UpdateOutputsAndState(int currentGear, double engineIdleRpm, double engineMaxRpm, double currentRpm, double now, bool isDrivetrainDecoupled, double accel, double velX, double velZ, bool tractionOk, double brake, double steer, double magnitudeG, double maxDrivenSlipRatio, double actualLimiter, double susNormFL, double susNormFR, double susNormRL, double susNormRR, double combinedSlipRL, double combinedSlipRR, double currentOversteer, double angularVelY, Drivetrain drivetrainType)
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

                double startRpm = Math.Max(1000, _globalEngine.RpmAtMaxPower * 0.55);
                double baseRpmProgress = startRpm; 
                double denseUpperLimit = actualLimiter - 250.0;
                
                if (denseUpperLimit > baseRpmProgress)
                {
                    int startBand = Math.Max(0, (int)Math.Floor(baseRpmProgress / 50.0));
                    int endBand = Math.Min(400, (int)Math.Ceiling(denseUpperLimit / 50.0));
                    int expectedBands = Math.Max(1, endBand - startBand);
                    int filledBands = 0;

                    for (int i = startBand; i <= endBand; i++)
                    {
                        if (_globalEngine.PowerMap[i] > 0) filledBands++;
                    }
                    
                    _out.LiveMappingProgress = Math.Min(100.0, Math.Max(0.0, ((double)filledBands / expectedBands) * 100.0));
                }
                else
                {
                    _out.LiveMappingProgress = 0;
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
                
                bool isEngineMapTrusted = _out.LiveMappingProgress >= 92.0;
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

            if (!isDrivetrainDecoupled && currentRpm >= effectiveTarget && accel >= LightWotThreshold)
            {
                _shiftLightActive = true;
            }
            if (_shiftLightActive && (isDrivetrainDecoupled || currentRpm < effectiveTarget - 200.0 || accel < LiftThreshold))
            {
                _shiftLightActive = false;
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
        public double GhostLimiterRpm { get; set; } = 0; 
    }
    
    class CarStore {
        public int SchemaVersion { get; set; } = 2;
        public Dictionary<string, Dictionary<int, double>> GearRatios { get; set; } = new Dictionary<string, Dictionary<int, double>>(); 
        public Dictionary<string, SavedEngineProfile> EngineProfiles { get; set; } = new Dictionary<string, SavedEngineProfile>();
        public Dictionary<string, Dictionary<int, SavedGearData>> AdvancedGears { get; set; } = new Dictionary<string, Dictionary<int, SavedGearData>>();
    }
}
