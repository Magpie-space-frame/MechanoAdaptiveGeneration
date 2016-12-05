using System;
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

        //algorithm parameters
        private BackGroundData _backGroundData;
        private int _count;
        private int _maxIterations;

        private bool _isInitialized;
        private double[] _recentVolumeFillingErrors;
        private int _updateInterval;
        private double _meshVolume;
        private double _targetVolumeToFill;

        //Goals and associated parameters
        private PhysicalSystem _ps;
        private List<IGoal> _persistentGoalList;
        private StressTensor[,,] _grid;
        private List<double> _localData;
        private List<int> _fixedPointIndices;
        private double _plasticDragRadius;

        //ellipsoid parameters
        private List<Ellipsoid> _ellipsoids;
        private double _ev1Max;
        private double _ev1Mean;
        private double _ev1Min;
        private double _ev1Stdev;
        private List<Vector3d> _sa;
        private List<Vector3d> _la;
        private double _scaleEllipsoids;
        private List<Line> _lineList = new List<Line>();

        //eigenvalue problem solution
        private List<double> _eval1;
        private List<double> _eval2;
        private List<double> _eval3;
        private List<Vector3d> _evec1;
        private List<Vector3d> _evec2;
        private List<Vector3d> _evec3;


        /// <summary>
        /// Initialiases an instance to generate a space frame mechano-adaptively
        /// </summary>
        /// <param name="m">A Mesh delineating the volume to generate a space frame in</param>
        /// <param name="s">An (optional) 2D surface mesh to constrain the nodes to</param>
        /// <param name="pts">Input configuration of points</param>
        /// <param name="data">Input tensorial data</param>
        /// <param name="updateScale">Boolean, if true, ellipsoids will be scaled to comply with volume factor</param>
        /// <param name="boundaryCollideStrength">weight for collision with boundary</param>
        /// <param name="fixedPointIndices">Indices of points to be fixed</param>
        /// <param name="maxIterations">maximum iterations to be performed</param>
        /// <param name="minLongAxisLength">limit for length of ellipsoid short axis</param>
        /// <param name="maxLongAxisLength">limit for length of ellipsoid long axis</param>
        /// <param name="minSlenderness">limit for aspect ratio of an ellipsoid</param>
        /// <param name="alignStrength">weight for alignment force goal</param>
        /// <param name="plasticDrag">Weight for the plastic drag goal</param>
        /// <param name="volumeFactor">The multiple of the mesh volume the ellipsoids should fill</param>
        /// <returns></returns>
        public bool Initialize(Mesh m, Mesh s, List<Point3d> pts, List<double> data, int maxIterations, bool updateScale, double plasticDrag,
                                        double boundaryCollideStrength, double alignStrength, List<int> fixedPointIndices,
                                         double minLongAxisLength, double maxLongAxisLength,
                                        double minSlenderness, double volumeFactor)
        {
            //Algorithm parameters:
            _count = 0;
            _updateInterval = 2;
            _maxIterations = maxIterations;
            _recentVolumeFillingErrors = new double[10];


            //Goals and corresponding parameters:
            _ps = new PhysicalSystem();
            _persistentGoalList = new List<IGoal>();
            _ps.ClearParticles();
            _persistentGoalList.Clear();
            _plasticDragRadius = 0.01;
            _fixedPointIndices = fixedPointIndices;


            //ellipsoids and associated parameters
            _ellipsoids = new List<Ellipsoid>();
            _lineList = new List<Line>();
            _la = new List<Vector3d>();
            _sa = new List<Vector3d>();
            _ellipsoids.Clear();
            _lineList.Clear();
            _la.Clear();
            _sa.Clear();

            var massProperties = Rhino.Geometry.VolumeMassProperties.Compute(m);
            _meshVolume = massProperties.Volume;
            _targetVolumeToFill = volumeFactor * _meshVolume;
            _scaleEllipsoids = Math.Pow(_meshVolume / pts.Count, 1.0 / 3.0);

            //Eigen value problem solution
            _eval1 = new List<double>();
            _eval2 = new List<double>();
            _eval3 = new List<double>();
            _evec1 = new List<Vector3d>();
            _evec2 = new List<Vector3d>();
            _evec3 = new List<Vector3d>();

            //Gridpoints and tensors there
            _localData = new List<double>();
            _backGroundData = new BackGroundData();
            _grid = new StressTensor[0, 0, 0];
            _localData.Clear();
            _localData = data;
            HelperFunctions.ProcessData(_localData, ref _backGroundData, ref _grid);
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

                    //translate these values to rs and rl TODO make function
                    var clampEval = Math.Max(_eval1[i], _ev1Min);
                    clampEval = Math.Min(clampEval, _ev1Max);
                    var evParam = (clampEval - _ev1Min) / (_ev1Max - _ev1Min);
                    var effectiveLongAxisLength = minLongAxisLength + (1 - evParam) * (maxLongAxisLength - minLongAxisLength);

                    var a = _scaleEllipsoids * effectiveLongAxisLength;
                    _la.Add(a * _evec1[i]);

                    var ratio = Math.Max(minSlenderness, _eval2[i] / _eval1[i]);
                    if (Double.IsNaN(ratio)) ratio = 1.0;
                    var b = a * ratio;
                    _sa.Add(b * _evec2[i]);
                    if (_eval2[i] == 0.0) _sa.Add(new Vector3d(0.0001, 0.0001, 0.0001));

                    _ellipsoids[i].UpdateTransform(a * _evec1[i], b * _evec2[i], b * _evec3[i]);
                }

            _persistentGoalList.Add(new SolidPoint(ix, m, true, boundaryCollideStrength));
            _persistentGoalList.Add(new OnMesh(ix, s, boundaryCollideStrength / 10));

            //plastic anchor goal for stopping circulation
            for (var i = 0; i < pts.Count; i++)
                _persistentGoalList.Add(new AnchorPlastic(i, pts[i], _plasticDragRadius, plasticDrag));

            //add anchor goal for fixed points
            for (var i = 0; i < fixedPointIndices.Count; i++)
                _persistentGoalList.Add(new Anchor(fixedPointIndices[i], pts[fixedPointIndices[i]], 10000));

            _isInitialized = true;
            return true;
        }

        /// <summary>
        /// Function to perform one step of the MAG algorithm
        /// </summary>
        /// <param name="updateScale">Boolean, if true, ellipsoids will be scaled to comply with volume factor</param>
        /// <param name="plasticDrag">Weight for the plastic drag goal</param>
        /// <param name="boundaryCollideStrength">weight for collision with boundary</param>
        /// <param name="alignStrength">weight for alignment force goal</param>
        /// <param name="minLongAxisLength">limit for length of ellipsoid short axis</param>
        /// <param name="maxLongAxisLength">limit for length of ellipsoid long axis</param>
        /// <param name="minSlenderness">limit for aspect ratio of an ellipsoid</param>
        public void Step(bool updateScale, double plasticDrag, double boundaryCollideStrength, double alignStrength,
                         double minLongAxisLength, double maxLongAxisLength,
                         double minSlenderness)
        {
            if (!_isInitialized)
            {
                return;
            }

            //Goals and associated parameters
            var temporaryGoalList = new List<IGoal>();
            int ptCount = _ps.ParticleCount();
            for (var i = 2; i < ptCount + 2; i++)
                (_persistentGoalList[i] as AnchorPlastic).Limit = _plasticDragRadius;

            (_persistentGoalList[0] as SolidPoint).Strength = boundaryCollideStrength;
            var positions = _ps.GetPositionArray();

            //Ellipsoids, stresstensors and associated parameters
            double sumOfCurrentEllipsoidVolumes = 0.0;

            if (_count % _updateInterval == 1)
            {
                HelperFunctions.InterpolateTensor(positions, ref _evec1, ref _evec2, ref _evec3, ref _eval1, ref _eval2, ref _eval3, _grid, _backGroundData);

                for (var i = 0; i < positions.Length; i++)
                {
                    if ((_evec1 != null) && (_evec2 != null) && (_evec3 != null))
                    {
                        {
                            //TODO make helperfunction.FirstEvalToLongAxis
                            var clampEval = Math.Max(_eval1[i], _ev1Min);
                            clampEval = Math.Min(clampEval, _ev1Max);
                            var evParam = (clampEval - _ev1Min) / (_ev1Max - _ev1Min);
                            var effectiveLongAxisLength = minLongAxisLength +
                                                          (1 - evParam) * (maxLongAxisLength - minLongAxisLength);

                            var a = _scaleEllipsoids * effectiveLongAxisLength;
                            _la[i] = a * _evec1[i];
                            if (_eval1[i] == 0.0)
                            {
                                _la[i] = new Vector3d(0.0001, 0.0001, 0.0001);
                                //add runtime warning message here! HOW?
                            }

                            var ratio = Math.Max(minSlenderness, _eval2[i] / _eval1[i]);
                            if (Double.IsNaN(ratio)) ratio = 1.0;
                            var b = a * ratio;
                            _sa[i] = b * _evec2[i];
                            if (_eval2[i] == 0.0) _sa[i] = new Vector3d(0.0001, 0.0001, 0.0001);

                            _ellipsoids[i].UpdateTransform(a * _evec1[i], b * _evec2[i], b * _evec3[i]);
                            sumOfCurrentEllipsoidVolumes += 4.0 * Math.PI * a * b * b / 3.0;
                        }
                    }
                }


                //update the ellipsoid transformations from field gloabally
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
                    temporaryGoalList.Add(new NonLinearRepel(eb, ea, pushDistance, 1, 0.1, -2));

                    var l = new Line(positions[ea], positions[eb]);

                    temporaryGoalList.Add(new Align(eb, ea,
                        new Vector3d[3]
                        {
                                    0.5*(_ellipsoids[ea].UnitXAxis + _ellipsoids[eb].UnitXAxis),
                                    0.5*(_ellipsoids[ea].UnitYAxis + _ellipsoids[eb].UnitYAxis),
                                    0.5*(_ellipsoids[ea].UnitZAxis + _ellipsoids[eb].UnitZAxis)
                        }, alignStrength));

                    var lLen = l.Length;

                    if ((pushDistance - lLen) / lLen > 1.0)
                    {
                        _lineList.Add(l);
                        connectedEdges[ea].Add(_lineList.Count - 1);
                        connectedEdges[eb].Add(_lineList.Count - 1);
                    }
                }
            }

            temporaryGoalList.AddRange(_persistentGoalList);
            _ps.SimpleStep(temporaryGoalList);
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
