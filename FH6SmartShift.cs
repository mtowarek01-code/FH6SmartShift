using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using GameReaderCommon;
using SimHub.Plugins;

namespace FH6SmartShift
{
    [PluginName("FH6SmartShift")]
    [PluginDescription("The Architect's Masterpiece - v1.0.0 (Ultimate Monolith, Zero Stutter & Top Gear Fix)")]
    [PluginAuthor("mtowarek01")]
    public class FH6SmartShift : IPlugin, IDataPlugin
    {
        private const string StoreKey = "fh6SmartShiftingData"; 

        private const double DownshiftCooldownSeconds = 0.5; 
        private const double UpshiftCooldownSeconds = 0.30; 
        private const double ClutchSettleSeconds = 0.15; 
        private const double SaveIntervalSeconds = 15.0; 
        private const double MathEvalIntervalSeconds = 0.20; 

        private const double SafeLandingRpmMultiplier = 0.93; 
        private const double MaxAllowedDownshiftMultiplier = 0.88; 
        
        private const double LearnWotThreshold = 242.0;
        private const double LightWotThreshold = 230.0;
        private const double LiftThreshold = 210.0;
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
        
        private Stopwatch _appTime = new Stopwatch();
        private Stopwatch _dtStopwatch = new Stopwatch();
        private bool _shiftLightActive = false;
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
            public bool IsTopGear = false;
            public bool IsGripLost = false; 
            public double BodySlipAngle = 0;
            public double MaxUsableRpm = 0;
            public double RevLimiterProximity = 0;
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

        public void Init(PluginManager pluginManager)
        {
            try {
                string dir = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "PluginsData", "Common");
                string p = System.IO.Path.Combine(dir, "FH6SmartShift_fh6SmartShiftingData.json");
                if (System.IO.File.Exists(p)) {
                    string todayBak = p + ".bak_" + DateTime.Now.ToString("yyyyMMdd");
                    if (!System.IO.File.Exists(todayBak)) {
                        System.IO.File.Copy(p, todayBak, true);
                        Debug.WriteLine($"[FH6SmartShift] Utworzono dzienna kopie zapasowa: {todayBak}");
                    }
                    
                    var dirInfo = new System.IO.DirectoryInfo(dir);
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
            
            _appTime.Start();
            _dtStopwatch.Start();
            
            pluginManager.AddAction("ResetPowerMemory", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetCurrentRequested, 1));
            pluginManager.AddAction("ResetAllCarsMemory", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetAllRequested, 1));
            pluginManager.AddAction("ResetLimitGears", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetLimitGearsRequested, 1));
            pluginManager.AddAction("ResetGearRatios", this.GetType(), (a, b) => Interlocked.Exchange(ref _resetRatiosRequested, 1));

            this.AttachDelegate("MaxUsableRpm", () => _out.MaxUsableRpm);
            this.AttachDelegate("RevLimiterProximity", () => _out.RevLimiterProximity);
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
                } 
            }
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
                    _isDataDirty = false;
                }
            }
            if (toSave != null)
            {
                Task.Run(() => { 
                    try { this.SaveCommonSettings(StoreKey, toSave); } 
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

        private bool IsRangeDense(double fromRpm, double toRpm, double minFill = 0.85)
        {
            int a = Math.Max(0, (int)Math.Floor(fromRpm / 50.0));
            int b = Math.Min(400, (int)Math.Ceiling(toRpm / 50.0));
            if (b <= a) return false;
            int ok = 0;
            for (int i = a; i <= b; i++) if (_globalEngine.BandSamples[i] > 0) ok++;
            return (double)ok / (b - a + 1) >= minFill;
        }

        private void EvaluateAllMathIntersections(double throttle, int currentGear, double engineMaxRpm, double currentRpm, double actualLimiterLocal)
        {
            Array.Clear(_precomputedPower, 0, _precomputedPower.Length);

            _globalEngine.MaxPower = 0;
            _globalEngine.MaxTorque = 0;
            for (int i = Math.Max(0, _globalEngine.MinMappedBand); i <= Math.Min(400, _globalEngine.MaxMappedBand); i++)
            {
                if (_globalEngine.BandSamples[i] == 0) continue;
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

            double startRpm = Math.Max(1000, _globalEngine.RpmAtMaxPower * 0.50);
            double denseUpperLimit = actualLimiterLocal - 250.0;
            bool isMapDense = false;
            
            if (denseUpperLimit > startRpm)
            {
                int startBand = (int)Math.Floor(startRpm / 50.0);
                int endBandLimit = Math.Min(400, (int)Math.Ceiling(denseUpperLimit / 50.0));
                int expectedBands = Math.Max(1, endBandLimit - startBand);
                int validBandsInRange = 0;
                for (int i = startBand; i <= endBandLimit; i++) {
                    if (_globalEngine.BandSamples[i] > 0) validBandsInRange++;
                }
                if (((double)validBandsInRange / expectedBands) >= 0.85) isMapDense = true; 
            }

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
                                if (consecutiveIntersections >= 6 || (consecutiveIntersections >= 2 && r >= validCeiling - 25))
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

                        bool hitLimiterCeiling = validCeiling >= actualLimiterLocal - 300.0 && (_globalEngine.GhostLimiterRpm > 0 || isMapDense);
                        bool gearDense = foundIntersection && IsRangeDense(firstCross * dropRatio, actualLimiterLocal - 250.0);
                        
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
                        else if (isMapDense || (!gs.IsMathOptimal && !gs.TriggerIsGhost))
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
                    else if (!hasHigherGear && throttle >= LearnWotThreshold && currentGear == g)
                    {
                        if (currentRpm >= actualLimiterLocal - 100)
                        {
                            gs.ShiftTriggerRpm = actualLimiterLocal - 30;
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
                    _lastLearnedBand = -1;
                    _lastLearnedPower = 0;
                    _lastLearnedTime = -100;
                    _lastMapScanTime = -100;
                    _smoothedLiveBuffer = 50.0;
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
                    _lastLearnedBand = -1;
                    _lastLearnedPower = 0;
                    _lastLearnedTime = -100;
                    _lastMapScanTime = -100;
                    _smoothedLiveBuffer = 50.0;
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
                    _lastLearnedBand = -1;
                    _lastLearnedPower = 0;
                    _lastLearnedTorque = 0;
                    _lastLearnedTime = -100;
                    _lastMapScanTime = -100;
                    _smoothedLiveBuffer = 50.0;
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
                                for(int j = i + 1; j < sortedGears.Count; j++) {
                                    int g2 = sortedGears[j];
                                    double r2 = savedRatios[g2];
                                    if (r2 <= 0) continue;
                                    if (r1 <= r2 * 1.02) violations++;
                                }
                                
                                if (violations > maxViolations) {
                                    maxViolations = violations;
                                    worstGear = g1;
                                }
                            }
                            
                            if (maxViolations > 0 && worstGear != -1) {
                                savedRatios.Remove(worstGear);
                                sortedGears.Remove(worstGear);
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
            bool isRatioLearnSafe = (maxAllSlip < 0.10) && (brake < BrakeDeadzone);

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

            double currentRatio = (isRatioLearnSafe && avgWheelSpeed > 20.0 && currentRpm > 1000.0) ? currentRpm / avgWheelSpeed : 0;

            double maxRearSlipAngle = Math.Max(Math.Abs(slipAngleRL), Math.Abs(slipAngleRR));
            double maxRearCombinedSlip = Math.Max(Math.Abs(combinedSlipRL), Math.Abs(combinedSlipRR));
            double currentOversteer = 0.0;
            
            if (speedKmh > 20.0 && (drivetrainType == Drivetrain.RWD || drivetrainType == Drivetrain.AWD))
            {
                double angleScore = Math.Max(0, (maxRearSlipAngle - 0.12) / 0.48); 
                double slipScore = Math.Max(0, (maxRearCombinedSlip - 0.65) / 0.75); 
                
                currentOversteer = Math.Min(1.0, (angleScore * 0.7) + (slipScore * 0.3));
                if (drivetrainType == Drivetrain.AWD) currentOversteer *= 0.65; 
                if (accel < LiftThreshold) currentOversteer *= 0.6; 
            }

            bool isDrivetrainDecoupled = (clutch >= 2.0 || handbrake >= 2.0);
            if (isDrivetrainDecoupled) _lastDecoupledTime = now;
            bool isAfterClutch = (now - _lastDecoupledTime) < ClutchSettleSeconds;

            if (_lastGear >= 1 && currentGear != _lastGear)
            {
                if (currentGear < _lastGear) { _lastDownshiftTime = now; _lastUpshiftTime = -100; _shiftLightActive = false; }
                else { _lastUpshiftTime = now; _lastDownshiftTime = -100; _shiftLightActive = false; }
                
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
            double slipTolerance = isLowGear ? 0.65 : 0.40; 
            
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
                bool dipDetected = (_globalEngine.GhostLimiterRpm > 0 && currentRpm > _globalEngine.GhostLimiterRpm - 150 && _currentRpmAccel <= 0) || (_lastLearnedPower > 0 && currentPower < _lastLearnedPower * 0.80);
                
                bool filtersOk = isGrounded && !inPuddle && !hasDamage && !isReboundDetected && !isRumble && isDrivetrainEngaged && isLearnWot && !isAfterAnyShift && tractionOk;

                if (currentPower > 0 && filtersOk && currentPower > 5000.0 && !dipDetected && !isLimiterProximity)
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
                            if (_lastLearnedBand > 0 && currentRpmBand > _lastLearnedBand && (currentRpmBand - _lastLearnedBand) <= 10)
                            {
                                if (_lastLearnedBand < _globalEngine.MinMappedBand) _globalEngine.MinMappedBand = _lastLearnedBand;
                                if (currentRpmBand > _globalEngine.MaxMappedBand) _globalEngine.MaxMappedBand = currentRpmBand;

                                for (int i = _lastLearnedBand + 1; i <= currentRpmBand; i++)
                                {
                                    double t = (double)(i - _lastLearnedBand) / (currentRpmBand - _lastLearnedBand);
                                    double sweepPower = _lastLearnedPower + t * (currentPower - _lastLearnedPower);
                                    double sweepTorque = _lastLearnedTorque + t * (currentTorque - _lastLearnedTorque);

                                    _globalEngine.BandSamples[i]++;
                                    double alpha = Math.Max(1.0 / _globalEngine.BandSamples[i], 0.05);

                                    _globalEngine.PowerMap[i] = _globalEngine.PowerMap[i] == 0 ? sweepPower : (_globalEngine.PowerMap[i] * (1.0 - alpha)) + (sweepPower * alpha);
                                    _globalEngine.TorqueMap[i] = _globalEngine.TorqueMap[i] == 0 ? sweepTorque : (_globalEngine.TorqueMap[i] * (1.0 - alpha)) + (sweepTorque * alpha);
                                    _globalEngine.BoostMap[i] = _globalEngine.BoostMap[i] == 0 ? currentBoost : (_globalEngine.BoostMap[i] * (1.0 - alpha)) + (currentBoost * alpha);
                                }
                            }
                            else
                            {
                                if (currentRpmBand < _globalEngine.MinMappedBand) _globalEngine.MinMappedBand = currentRpmBand;
                                if (currentRpmBand > _globalEngine.MaxMappedBand) _globalEngine.MaxMappedBand = currentRpmBand;

                                _globalEngine.BandSamples[currentRpmBand]++;
                                double alpha = Math.Max(1.0 / _globalEngine.BandSamples[currentRpmBand], 0.05);

                                _globalEngine.PowerMap[currentRpmBand] = _globalEngine.PowerMap[currentRpmBand] == 0 ? currentPower : (_globalEngine.PowerMap[currentRpmBand] * (1.0 - alpha)) + (currentPower * alpha);
                                _globalEngine.TorqueMap[currentRpmBand] = _globalEngine.TorqueMap[currentRpmBand] == 0 ? currentTorque : (_globalEngine.TorqueMap[currentRpmBand] * (1.0 - alpha)) + (currentTorque * alpha);
                                _globalEngine.BoostMap[currentRpmBand] = _globalEngine.BoostMap[currentRpmBand] == 0 ? currentBoost : (_globalEngine.BoostMap[currentRpmBand] * (1.0 - alpha)) + (currentBoost * alpha);
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
                else
                {
                    _lastLearnedBand = -1;
                    _lastLearnedPower = 0;
                    _lastLearnedTime = -100;
                }

                if (currentRatio > 0 && isLearnWot && isDrivetrainEngaged && !isAfterAnyShift)
                {
                    gs.RatioHistory[gs.RatioHistoryIdx] = currentRatio;
                    gs.RatioHistoryIdx = (gs.RatioHistoryIdx + 1) % 25;
                    if (gs.RatioSamples < 25) gs.RatioSamples++;
                    
                    if (gs.RatioSamples >= 15)
                    {
                        int samplesToTake = gs.RatioSamples;
                        Array.Copy(gs.RatioHistory, _ratioScratch, samplesToTake);
                        Array.Sort(_ratioScratch, 0, samplesToTake);
                        double median = _ratioScratch[samplesToTake / 2];

                        bool isValidRatio = true;
                        if (currentGear > 1 && _gearStates.TryGetValue(currentGear - 1, out var pGState) && pGState.SavedRpmPerRadS > 0)
                        {
                            if (median >= pGState.SavedRpmPerRadS * 0.98) isValidRatio = false;
                        }
                        if (_gearStates.TryGetValue(currentGear + 1, out var nGState) && nGState.SavedRpmPerRadS > 0)
                        {
                            if (median <= nGState.SavedRpmPerRadS * 1.02) isValidRatio = false;
                        }

                        if (isValidRatio)
                        {
                            if (gs.SavedRpmPerRadS > 0)
                            {
                                if (Math.Abs(median - gs.SavedRpmPerRadS) / gs.SavedRpmPerRadS > 0.05)
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
                                }
                            }

                            if (gs.SavedRpmPerRadS == 0)
                            {
                                gs.RpmPerRadS = median; gs.SavedRpmPerRadS = median;
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

                double normalizedSteer = steer / 127.0;
                double rawCounterSeverity = 0.0;

                if (currentOversteer > 0.30) 
                {
                    bool significantSteer = Math.Abs(normalizedSteer) > 0.15;
                    bool significantYaw = Math.Abs(angularVelY) > 0.30;
                    
                    if (significantSteer && significantYaw && (Math.Sign(normalizedSteer) != Math.Sign(angularVelY))) 
                    {
                        rawCounterSeverity = Math.Min(1.0, (Math.Abs(normalizedSteer) - 0.15) / 0.85);
                    }
                }

                double tauCounter = 0.08; 
                double alphaCounter = 1.0 - Math.Exp(-_frameDt / tauCounter);

                _out.CounterSteerSeverity = (_out.CounterSteerSeverity * (1.0 - alphaCounter)) + (rawCounterSeverity * alphaCounter);
                _out.IsCounterSteering = _out.CounterSteerSeverity > 0.10;

                bool isTopGear = false;
                if (currentGear >= 5 && _gearStates.TryGetValue(currentGear, out var cgTop) && cgTop.SavedRpmPerRadS > 0 && cgTop.IsConfirmed)
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

                double startRpm = Math.Max(1000, _globalEngine.RpmAtMaxPower * 0.50);
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

                double range = actualLimiter - baseRpmProgress;
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
                
                bool isEngineMapTrusted = _out.LiveMappingProgress >= 95.0;
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
                            else if (gState.TriggerIsGhost)
                            {
                                _out.GearText[i] = LabelsLimitConfirmed[i];
                                _out.GearColor[i] = "#4CAF50"; 
                            }
                            else if (gState.TriggerIsEdge)
                            {
                                _out.GearText[i] = LabelsLimit[i];
                                _out.GearColor[i] = "#F39C12"; 
                            }
                            else
                            {
                                _out.GearText[i] = LabelsRatio[i];
                                _out.GearColor[i] = "#95A5A6"; 
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

            double visualTrigger = Math.Min(trigger + _smoothedLiveBuffer, actualLimiter);

            if (!isDrivetrainDecoupled && currentRpm >= trigger && accel >= LightWotThreshold) _shiftLightActive = true;
            if (_shiftLightActive && (isDrivetrainDecoupled || currentRpm < trigger - 250 || accel < LiftThreshold)) _shiftLightActive = false;

            double startLight = engineIdleRpm + ((visualTrigger - engineIdleRpm) * 0.75);
            if (visualTrigger <= startLight || (visualTrigger - startLight) <= 0) _out.RevLimiterProximity = 1.0;
            else _out.RevLimiterProximity = _shiftLightActive ? 1.0 : (currentRpm <= startLight ? 0.0 : (currentRpm >= visualTrigger ? 1.0 : (currentRpm - startLight) / (visualTrigger - startLight)));

            bool localDirty;
            lock (_lock) { localDirty = _isDataDirty; }
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