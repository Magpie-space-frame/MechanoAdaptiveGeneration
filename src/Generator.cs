﻿using System;
using System.Collections.Generic;
using System.Linq;
using KangarooSolver;
using KangarooSolver.Goals;
using MechanoAdaptiveGeneration.customK2goals;
using Rhino.Geometry;

namespace MechanoAdaptiveGeneration
{
    /// <summary>
    /// Class that handles mechano-adaptive generation of space frames
    /// Used by corresponding GH component, but can also be called from a C# Script in GH
    /// </summary>
    public class Generator
    {
        public Generator()
        {
        }

        private BackGroundData _backGroundData;
        private int _count;
        private int _maxIterations;

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

        private List<Line> _lineList = new List<Line>();

        private List<double> _localData;
        //the script below the grey line
        private PhysicalSystem _ps;
        private double[] _recentVolumeFillingErrors;
        private List<Vector3d> _sa;
        private List<Vector3d> _la;

        private double _scaleEllipsoids;
        private int _updateInterval;

        private List<int> _fixedPointIndices;

        private double _meshVolume;
        private double _targetVolumeToFill;
        private double _plasticDragRadius;

        /// <summary>
        /// Initialiases an instance to generate a space frame mechano-adaptively
        /// </summary>
        /// <param name="m">A Mesh delineating the volume to generate a space frame in</param>
        /// <param name="s">An (optional) 2D surface mesh to constrain the nodes to</param>
        /// <param name="pts">Input configuration of points</param>
        /// <param name="data">Input tensorial data</param>
        /// <param name="updateScale">Boolean, if true, ellipsoids will be scaled to comply with volume factor</param>
        /// <param name="valenceFilter">Boolean, if true, edge connections will be filtered to a maximum of 6</param>
        /// <param name="boundaryCollideStrength">weight for collision with boundary</param>
        /// <param name="fixedPointIndices">Indices of points to be fixed</param>
        /// <param name="maxIterations">maximum iterations to be performed</param>
        /// <param name="minLongAxisLength">limit for length of ellipsoid short axis</param>
        /// <param name="maxLongAxisLength">limit for length of ellipsoid long axis</param>
        /// <param name="minSlenderness">limit for aspect ratio of an ellipsoid</param>
        /// <param name="maxRadiusCoefficient">relative limit for volume of an individual ellipsoid</param>
        /// <param name="alignStrength">weight for alignment force goal</param>
        /// <param name="edgeLengthFactor">factor to scale ellipsoids with when looking for colliding neighbours to connect with</param>
        /// <param name="plasticDrag">Weight for the plastic drag goal</param>
        /// <param name="volumeFactor">The multiple of the mesh volume the ellipsoids should fill</param>
        /// <returns></returns>
        public bool Initialize(Mesh m, Mesh s, List<Point3d> pts, List<double> data, bool updateScale, bool valenceFilter,
                                        double boundaryCollideStrength, List<int> fixedPointIndices,
                                        int maxIterations, double minLongAxisLength, double maxLongAxisLength,
                                        double minSlenderness, double maxRadiusCoefficient, double alignStrength,
                                        double edgeLengthFactor, double plasticDrag, double volumeFactor)
        {
            _ps = new PhysicalSystem();
            _goalList = new List<IGoal>();
            _ellipsoids = new List<Ellipsoid>();
            var massProperties = Rhino.Geometry.VolumeMassProperties.Compute(m);
            _meshVolume = massProperties.Volume;
            _targetVolumeToFill = volumeFactor * _meshVolume;

            _plasticDragRadius = 0.01;

            //set to zero so they are assigned.
            _scaleEllipsoids = 0;
            _ev1Mean = 0;
            _ev1Stdev = 0;
            _ev1Max = 0;
            _ev1Min = 0;

            _lineList = new List<Line>();

            _count = 0;
            _updateInterval = 2;
            _maxIterations = maxIterations;
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
            _scaleEllipsoids = Math.Pow(_meshVolume / pts.Count, 1.0 / 3.0);

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
            _goalList.Add(new OnMesh(ix, s, boundaryCollideStrength / 10));

            //plastic anchor for stopping circulation
            for (var i = 0; i < pts.Count; i++)
                _goalList.Add(new AnchorPlastic(i, pts[i], _plasticDragRadius, plasticDrag));

            for (var i = 0; i < fixedPointIndices.Count; i++)
                _goalList.Add(new Anchor(fixedPointIndices[i], pts[fixedPointIndices[i]], 10000));
            _count = 0;
            //EnergySum = double.MaxValue;

            _isInitialized = true;
            return true;
        }

        /// <summary>
        /// Function to perform one step of the MAG algorithm
        /// </summary>
        /// <param name="plasticDrag">Weight for the plastic drag goal</param>
        /// <param name="boundaryCollideStrength">weight for collision with boundary</param>
        /// <param name="minLongAxisLength">limit for length of ellipsoid short axis</param>
        /// <param name="maxLongAxisLength">limit for length of ellipsoid long axis</param>
        /// <param name="minSlenderness">limit for aspect ratio of an ellipsoid</param>
        /// <param name="maxRadiusCoefficient">relative limit for volume of an individual ellipsoid</param>
        /// <param name="alignStrength">weight for alignment force goal</param>
        /// <param name="edgeLengthFactor">factor to scale ellipsoids with when looking for colliding neighbours to connect with</param>
        /// <param name="valenceFilter">Boolean, if true, edge connections will be filtered to a maximum of 6</param>
        /// <param name="updateScale">Boolean, if true, ellipsoids will be scaled to comply with volume factor</param>
        public void Step(double plasticDrag, double boundaryCollideStrength,
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

            for (var i = 2; i < ptCount + 2; i++)
                (_goalList[i] as AnchorPlastic).Limit = _plasticDragRadius;

            var goalsToKeep = 2 + ptCount + _fixedPointIndices.Count;
            //Note that if more goals are added in the in the initialization, this number should be increased accordingly
            _goalList.RemoveRange(goalsToKeep, _goalList.Count - goalsToKeep);
            //clear everything apart from the SolidPoint Goal, OnMesh goal, the plastic drag goals and the fixed point goals .

            (_goalList[0] as SolidPoint).Strength = boundaryCollideStrength;

            var positions = _ps.GetPositionArray();
            if (_count % _updateInterval == 1)
            {
                HelperFunctions.InterpolateTensor(positions, ref _evec1, ref _evec2, ref _evec3, ref _eval1, ref _eval2, ref _eval3, _grid, _backGroundData);
            }

            for (var i = 0; i < positions.Length; i++)
            {
                if ((_evec1 != null) && (_evec2 != null) && (_evec3 != null))
                {
                    {
                        var clampEval = Math.Max(_eval1[i], _ev1Min);
                        clampEval = Math.Min(clampEval, _ev1Max);
                        var evParam = (clampEval - _ev1Min) / (_ev1Max - _ev1Min);
                        var effectiveLongAxisLength = minLongAxisLength +
                                                      (1 - evParam) * (maxLongAxisLength - minLongAxisLength);

                        _la[i] = _scaleEllipsoids * effectiveLongAxisLength * _evec1[i];
                        if (_eval1[i] == 0.0)
                        {
                            _la[i] = new Vector3d(0.0001, 0.0001, 0.0001);
                            //add runtime warning message here! HOW?
                        }
                        var ratio = Math.Max(minSlenderness, _eval2[i] / _eval1[i]);
                        if (Double.IsNaN(ratio)) ratio = 1.0;

                        _sa[i] = _scaleEllipsoids * ratio * effectiveLongAxisLength * _evec2[i];
                        if (_eval2[i] == 0.0) _sa[i] = new Vector3d(0.0001, 0.0001, 0.0001);

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
                }
            }

            if (_count % _updateInterval == 1)
            {
                //update the ellipsoid transformations from field
                double currentVolumeFillingError = Math.Abs(_targetVolumeToFill - sumOfCurrentEllipsoidVolumes);
                int numberOfInterpolations = (int)Math.Floor((double)_count / _updateInterval);
                _recentVolumeFillingErrors[(int)(numberOfInterpolations % 10)] = currentVolumeFillingError;
                if (updateScale)
                {
                    HelperFunctions.UpdateScaleByVolume(ref _scaleEllipsoids, ref sumOfCurrentEllipsoidVolumes, ref _targetVolumeToFill);
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
                    _goalList.Add(new NonLinearRepel(eb, ea, pushDistance, 1, 0.1, -2));

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

        public List<Vector3d> GetLongAxes()
        {
            return _la;
        }

        public List<Vector3d> GetShortAxes()
        {
            return _sa;
        }

        public List<Line> GetLines()
        {
            return _lineList;
        }
    }
}
