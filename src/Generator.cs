using System;
using System.Collections.Generic;
using System.Linq;
using KangarooSolver;
using KangarooSolver.Goals;
using MechanoAdaptiveGeneration.customK2goals;
using Rhino.Geometry;

namespace MechanoAdaptiveGeneration
{
    public class Generator
    {
        public Generator()
        {
        }

        private BackGroundData _backGroundData;
        private int _count;

        private List<Ellipsoid> _ellipsoids;
        private double _ev1Max;
        private double _ev1Mean;
        private double _ev1Min;
        private double _ev1Stdev;

        private List<double> _eval1;
        private List<double> _eval2;
        private List<double> _eval3;

        private List<Vector3d> _evec1;
        private List<Vector3d> _evec2;
        private List<Vector3d> _evec3;

        private List<IGoal> _goalList;
        private StressTensor[,,] _grid;
        private bool _isInitialized;

        private List<Vector3d> _la;

        private List<Line> _lineList = new List<Line>();

        private List<double> _localData;
        //the script below the grey line
        private PhysicalSystem _ps;
        private double[] _recentVolumeFillingErrors;
        private List<Vector3d> _sa;

        private double _scaleEllipsoids;
        private int _updateInterval;

        private List<int> _fixedPointIndices;

        private double _meshVolume;

        public bool Initialize(Mesh m, List<Point3d> pts, List<double> data, bool updateScale, bool valenceFilter,
                                        double boundaryCollideStrength, List<int> fixedPointIndices,
                                        int maxIterations, double percentVolPacked,
                                        int iterations, List<Point3d> ellipsoidCenters,                                                                        
                                        double minLongAxisLength, double maxLongAxisLength,
                                        double minSlenderness, double maxRadiusCoefficient,
                                        double initialScaleEllipsoids, double alignStrength,
                                        double edgeLengthFactor, double plasticdrag)
        {
            _ps = new PhysicalSystem();
            _goalList = new List<IGoal>();
            _ellipsoids = new List<Ellipsoid>();
            var massProperties = Rhino.Geometry.VolumeMassProperties.Compute(m);
            _meshVolume = massProperties.Volume;

            //set to zero so they are assigned.
            _scaleEllipsoids = 0;
            _ev1Mean = 0;
            _ev1Stdev = 0;
            _ev1Max = 0;
            _ev1Min = 0;

            _lineList = new List<Line>();

            _count = 0;
            _updateInterval = 2;
            _recentVolumeFillingErrors = new double[10];

            _eval1 = new List<double>();
            _eval2 = new List<double>();
            _eval3 = new List<double>();

            _evec1 = new List<Vector3d>();
            _evec2 = new List<Vector3d>();
            _evec3 = new List<Vector3d>();

            _localData = new List<double>();

            _la = new List<Vector3d>();
            _sa = new List<Vector3d>();

            //Gridpoints and tensors there
            _backGroundData = new BackGroundData();
            _grid = new StressTensor[0, 0, 0];

            _ps.ClearParticles();
            _scaleEllipsoids = initialScaleEllipsoids;

            _fixedPointIndices = fixedPointIndices;        

            _goalList.Clear();
            _ellipsoids.Clear();
            _lineList.Clear();

            _localData.Clear();
            _localData = data;
            HelperFunctions.ProcessData(_localData, ref _backGroundData, ref _grid);

            _la.Clear();
            _sa.Clear();

            HelperFunctions.InterpolateTensor(pts.ToArray(), ref _evec1, ref _evec2, ref _evec3, ref _eval1,
                ref _eval2, ref _eval3, _grid, _backGroundData);

            //ellipsoid size is in range mean(ev1) +- 2 stdDev(ev1)
            _ev1Mean = _eval1.Average();
            _ev1Stdev = HelperFunctions.StdDev(ref _eval1);
            _ev1Max = _ev1Mean + 2 * _ev1Stdev;
            _ev1Min = _ev1Mean - 2 * _ev1Stdev;

            var ix = new List<int>();

            if (((_evec1 != null) && (_evec2 != null) && (_evec3 != null)) || true)
                for (var i = 0; i < pts.Count; i++)
                {
                    _ps.AddParticle(pts[i], 1); //add a particle for every point
                    ix.Add(i);
                    _ellipsoids.Add(new Ellipsoid(pts[i]));

                    //translate these values to rs and rl
                    var clampEval = Math.Max(_eval1[i], _ev1Min);
                    clampEval = Math.Min(clampEval, _ev1Max);

                    var evParam = (clampEval - _ev1Min) / (_ev1Max - _ev1Min);

                    var effectiveLongAxisLength = minLongAxisLength +
                                                  (1 - evParam) * (maxLongAxisLength - minLongAxisLength);

                    _la.Add(_scaleEllipsoids * effectiveLongAxisLength * _evec1[i]);

                    //double ratio = Math.Max(Math.Min(Math.Abs(Eval2[i]), maxLongAxisLength / scaleEllipsoids) / effectiveLongAxisLength, minSlenderness);
                    var ratio = Math.Max(minSlenderness, _eval2[i] / _eval1[i]);

                    _sa.Add(_scaleEllipsoids * ratio * effectiveLongAxisLength * _evec2[i]);

                    //if this ellipsoid is a sphere, it can have maximally a radius of maxRadiusCofficient*maxLongAxisLength; This is to restrict the volume.
                    var volumeRatio = maxRadiusCoefficient * maxLongAxisLength /
                                      (Math.Pow(ratio * ratio, 0.33333333) * effectiveLongAxisLength);
                    if (volumeRatio < 1)
                    {
                        _sa[i] *= volumeRatio;
                        _la[i] *= volumeRatio;
                        effectiveLongAxisLength *= volumeRatio;
                    }

                    _ellipsoids[i].UpdateTransform(_scaleEllipsoids * effectiveLongAxisLength * _evec1[i],
                        _scaleEllipsoids * ratio * effectiveLongAxisLength * _evec2[i],
                        _scaleEllipsoids * ratio * effectiveLongAxisLength * _evec3[i]);
                    var a = _scaleEllipsoids * effectiveLongAxisLength;
                    var b = _scaleEllipsoids * ratio * effectiveLongAxisLength;
                }

            _goalList.Add(new SolidPoint(ix, m, true, boundaryCollideStrength));

            //plastic anchor for stopping circulation
            for (var i = 0; i < pts.Count; i++)
                _goalList.Add(new AnchorPlastic(i, pts[i], plasticdrag, 1000));

            for (var i = 0; i < fixedPointIndices.Count; i++)
                _goalList.Add(new Anchor(fixedPointIndices[i], pts[fixedPointIndices[i]], 10000));
            _count = 0;
            //EnergySum = double.MaxValue;

            _isInitialized = true;       
            return true;
        }

        public void Step(double plasticdrag, double boundaryCollideStrength, 
                         double minLongAxisLength, double maxLongAxisLength,
                         double minSlenderness, double maxRadiusCoefficient,
                         double alignStrength, double edgeLengthFactor,
                         bool valenceFilter, bool updateScale)
        {
            if (!_isInitialized)
            {
                return;
            }

            var totalOverlap = 0.0;
            double sumOfCurrentEllipsoidVolumes = 0.0;

            int ptCount = _ps.ParticleCount();

            for (var i = 1; i < ptCount + 1; i++)
                (_goalList[i] as AnchorPlastic).Limit = plasticdrag;

            var goalsToKeep = 1 + ptCount + _fixedPointIndices.Count;
            //Note that if more goals are added in the in the initialization, this number should be increased accordingly
            _goalList.RemoveRange(goalsToKeep, _goalList.Count - goalsToKeep);
            //clear everything apart from the SolidPoint Goal.

            (_goalList[0] as SolidPoint).Strength = boundaryCollideStrength;

            var positions = _ps.GetPositionArray();

            for (var i = 0; i < positions.Length; i++)
                if ((_evec1 != null) && (_evec2 != null) && (_evec3 != null)) 
                {
                    {
                        var clampEval = Math.Max(_eval1[i], _ev1Min);
                        clampEval = Math.Min(clampEval, _ev1Max);
                        var evParam = (clampEval - _ev1Min) / (_ev1Max - _ev1Min);
                        var effectiveLongAxisLength = minLongAxisLength +
                                                      (1 - evParam) * (maxLongAxisLength - minLongAxisLength);

                        _la[i] = _scaleEllipsoids * effectiveLongAxisLength * _evec1[i];

                        var ratio = Math.Max(minSlenderness, _eval2[i] / _eval1[i]);

                        _sa[i] = _scaleEllipsoids * ratio * effectiveLongAxisLength * _evec2[i];

                        //if this ellipsoid is a sphere, it can have maximally a radius of maxRadiusCofficient*maxLongAxisLength; This is to restrict the volume.
                        var volumeRatio = maxRadiusCoefficient * maxLongAxisLength /
                                          (Math.Pow(ratio * ratio, 0.33333333) * effectiveLongAxisLength);
                        if (volumeRatio < 1)
                        {
                            _sa[i] *= volumeRatio;
                            _la[i] *= volumeRatio;
                            effectiveLongAxisLength *= volumeRatio;
                        }

                        _ellipsoids[i].UpdateTransform(_scaleEllipsoids * effectiveLongAxisLength * _evec1[i],
                            _scaleEllipsoids * ratio * effectiveLongAxisLength * _evec2[i],
                            _scaleEllipsoids * ratio * effectiveLongAxisLength * _evec3[i]);
                        var a = _scaleEllipsoids * effectiveLongAxisLength;
                        var b = _scaleEllipsoids * ratio * effectiveLongAxisLength;
                        sumOfCurrentEllipsoidVolumes += 4.0 * Math.PI * a * b * b / 3.0;
                    }

                    if (_count % _updateInterval == 1)
                    {
                        //update the ellipsoid transformations from field
                        var currentVolumeFillingError =
                            Math.Abs(1.0 - sumOfCurrentEllipsoidVolumes / (1.8 * _meshVolume));
                        var numberOfInterpolations = (int)Math.Floor((double)_count / _updateInterval);
                        _recentVolumeFillingErrors[numberOfInterpolations % _updateInterval] =
                            currentVolumeFillingError;
                        HelperFunctions.InterpolateTensor(positions, ref _evec1, ref _evec2, ref _evec3, ref _eval1,
                            ref _eval2, ref _eval3, _grid, _backGroundData);
                    }
                }

            double percentVolPacked = sumOfCurrentEllipsoidVolumes / _meshVolume;

            //A pair of matching lists to contain the collisions
            var collideRef0 = new List<int>();
            var collideRef1 = new List<int>();

            //use SAP to find AABB collisions
            HelperFunctions.SweepAndPrune(_ellipsoids, positions, ref collideRef0, ref collideRef1);

            //narrow phase collision
            var connectedEdges = new List<int>[positions.Length];
            for (var i = 0; i < positions.Length; i++)
                connectedEdges[i] = new List<int>();

            _lineList.Clear();
            for (var i = 0; i < collideRef0.Count(); i++)
            {
                var ea = collideRef0[i];
                var eb = collideRef1[i];
                var collision = new EllipsoidCollide(_ellipsoids[ea], _ellipsoids[eb]);
                var pushDistance = collision.CalculateCollision();

                if (pushDistance != -1)
                {
                    _goalList.Add(new NonLinearRepel(eb, ea, pushDistance, 1, 0.01, -2));

                    var l = new Line(positions[ea], positions[eb]);

                    _goalList.Add(new Align(eb, ea,
                        new Vector3d[3]
                        {
                                    0.5*(_ellipsoids[ea].UnitXAxis + _ellipsoids[eb].UnitXAxis),
                                    0.5*(_ellipsoids[ea].UnitYAxis + _ellipsoids[eb].UnitYAxis),
                                    0.5*(_ellipsoids[ea].UnitZAxis + _ellipsoids[eb].UnitZAxis)
                        }, alignStrength));

                    var lLen = l.Length;

                    if ((pushDistance - lLen) / lLen > edgeLengthFactor)
                    {
                        _lineList.Add(l);
                        connectedEdges[ea].Add(_lineList.Count - 1);
                        connectedEdges[eb].Add(_lineList.Count - 1);
                    }

                    if (!double.IsNaN(pushDistance))
                        totalOverlap = totalOverlap + 2 * Math.Abs(pushDistance);
                }
            }

            _ps.SimpleStep(_goalList);

            //comment out the scaling method you don't want
            if (updateScale && (_count % _updateInterval == 1))
                HelperFunctions.UpdateScaleByVolume(ref _scaleEllipsoids, ref sumOfCurrentEllipsoidVolumes,
                    ref _ellipsoids, ref _meshVolume);

            //Output the mesh, and how many iterations it took to converge
            //ellipsoidCenters = _ps.GetPositions().ToList();

            //longAxes = _la;
            //shortAxes = _sa;

            _count++;
            //if (_count > maxIterations / 10)
            //{
            //    var averageFillingError = _recentVolumeFillingErrors.ToList().Average();
            //    if ((averageFillingError < 0.0001) || (_count > maxIterations))
            //    {
            //        _running = false;
            //        bakeResult = true;
            //    }
            //}

            //iterations = _count;

            //optional post-packing connectivity culling
            if (valenceFilter)
            {
                var keep = new bool[_lineList.Count];
                for (var i = 0; i < ptCount; i++)
                {
                    var currentValence = connectedEdges[i].Count;
                    var edgeIndices = new int[currentValence];
                    // TODO: Replace this for boundary faces/edges
                    var targetValence = 6;

                    var edgeLengths = new double[currentValence];

                    for (var j = 0; j < currentValence; j++)
                    {
                        edgeLengths[j] = _lineList[connectedEdges[i][j]].Length;
                        edgeIndices[j] = j;
                    }

                    Array.Sort(edgeLengths, edgeIndices);

                    var maximum = Math.Min(targetValence, connectedEdges[i].Count);
                    for (var j = 0; j < maximum; j++)
                        keep[connectedEdges[i][edgeIndices[j]]] = true;
                }

                var keptLines = new List<Line>();
                for (var i = 0; i < _lineList.Count; i++)
                    if (keep[i])
                        keptLines.Add(_lineList[i]);

               // lines = keptLines;
            }
            else
            {
               // lines = _lineList;
            }
        }

        public List<Point3d> GetCentres()
        {
            return _ps.GetPositions().ToList();
        }

        public List<Line> GetLines()
        {
            return _lineList;
        }
    }
}
