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

        //parameter classes
        InputGeometryParameters igp;
        KangarooGoalParameters kgp;
        EllipsoidParameters ep;
        AlgorithmConvergenceParameters acp;

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


        private void initializeEllipsoids(InputGeometryParameters igp, EllipsoidParameters ep)
        {
            //ellipsoids and associated parameters
            _ellipsoids = new List<Ellipsoid>();
            _lineList = new List<Line>();
            _la = new List<Vector3d>();
            _sa = new List<Vector3d>();
            _ellipsoids.Clear();
            _lineList.Clear();
            _la.Clear();
            _sa.Clear();

            //ellipsoid size is in range mean(ev1) +- 2 stdDev(ev1)
            _ev1Mean = _eval1.Average();
            _ev1Stdev = HelperFunctions.StdDev(ref _eval1);
            _ev1Max = _ev1Mean + 2 * _ev1Stdev;
            _ev1Min = _ev1Mean - 2 * _ev1Stdev;

            for (var i = 0; i < igp.pts.Count; i++)
            {
                _ps.AddParticle(igp.pts[i], 1); //add a particle for every point
                _ellipsoids.Add(new Ellipsoid(igp.pts[i]));

                //translate these values to rs and rl TODO make function
                var clampEval = Math.Max(_eval1[i], _ev1Min);
                clampEval = Math.Min(clampEval, _ev1Max);
                var evParam = (clampEval - _ev1Min) / (_ev1Max - _ev1Min);
                var effectiveLongAxisLength = ep.minLongAxisLength + (1 - evParam) * (ep.maxLongAxisLength - ep.minLongAxisLength);

                var a = _scaleEllipsoids * effectiveLongAxisLength;
                _la.Add(a * _evec1[i]);

                var ratio = Math.Max(ep.minSlenderness, _eval2[i] / _eval1[i]);
                if (Double.IsNaN(ratio)) ratio = 1.0;
                var b = a * ratio;
                _sa.Add(b * _evec2[i]);
                if (_eval2[i] == 0.0) _sa.Add(new Vector3d(0.0001, 0.0001, 0.0001));

                _ellipsoids[i].UpdateTransform(a * _evec1[i], b * _evec2[i], b * _evec3[i]);
            }
            this.ep = ep;
        }

        private void initializeKangarooGoals(InputGeometryParameters igp, KangarooGoalParameters kgp)
        {
            _ps = new PhysicalSystem();
            _persistentGoalList = new List<IGoal>();
            _ps.ClearParticles();
            _persistentGoalList.Clear();
            _plasticDragRadius = 0.01;
            _fixedPointIndices = kgp.fixedPointIndices;

            var ix = new List<int>();
            for (var i = 0; i < igp.pts.Count; i++)
            {
                ix.Add(i);
            }
            _persistentGoalList.Add(new SolidPoint(ix, igp.m, true, kgp.boundaryCollideStrength));

            //plastic anchor goal for stopping circulation
            for (var i = 0; i < igp.pts.Count; i++)
                _persistentGoalList.Add(new AnchorPlastic(i, igp.pts[i], _plasticDragRadius, kgp.plasticDragDistance));

            if (igp.s != null)
            {
                _persistentGoalList.Add(new OnMesh(ix, igp.s, kgp.boundaryCollideStrength/100));
            }

            //add anchor goal for fixed points
            for (var i = 0; i < kgp.fixedPointIndices.Count; i++)
                _persistentGoalList.Add(new Anchor(kgp.fixedPointIndices[i], igp.pts[kgp.fixedPointIndices[i]], Double.MaxValue));
            this.kgp = kgp;
        }

        private void initializeTensorField(InputGeometryParameters igp)
        {
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
            _localData = igp.data;
            HelperFunctions.ProcessData(_localData, ref _backGroundData, ref _grid);
            //map tensorfield from grid onto points
            HelperFunctions.InterpolateTensor(igp.pts.ToArray(), ref _evec1, ref _evec2, ref _evec3, ref _eval1,
               ref _eval2, ref _eval3, _grid, _backGroundData);
            this.igp = igp;
        }

        private void initializeAlgorithmConvergenceParameters(InputGeometryParameters inputGeomParams, AlgorithmConvergenceParameters algoConvergeParams)
        {
            var massProperties = Rhino.Geometry.VolumeMassProperties.Compute(inputGeomParams.m);
            _meshVolume = massProperties.Volume;
            _targetVolumeToFill = algoConvergeParams.volumeFactor * _meshVolume;
            _scaleEllipsoids = Math.Pow(_targetVolumeToFill / inputGeomParams.pts.Count, 1.0 / 3.0);
            _count = 0;
            _updateInterval = 1;
            _maxIterations = algoConvergeParams.maxIterations;
            _recentVolumeFillingErrors = new double[10];
            this.acp = algoConvergeParams;
        }

        /// <summary>
        /// Initialiases an instance to generate a space frame mechano-adaptively
        /// </summary>
        /// <param name="inputGeomParams">A set of input geometry parameters, which includes a mesh (the volume to generate a space frame in), an optional surface mesh (to constrain the nodes to), a list of starting points and the input tensor field</param>
        /// <param name="kangarooGoalParams">Set of parameters for the Kangaroo goals: alignStrength, boundaryCollideStrength, distance within which plastic drag will act and indices of the points to be kept fixed</param>
        /// <param name="ellipsoidParams">Set of parameters describing constraints on each ellipsoid: minimum and maximum length for the long axis and minimum ratio of short to long axis</param>
        /// <param name="algoConvergeParams">Set of parameters for how our algorithm will recognize convergence: A boolean determining whether the ellipsoid sizing should get updated or not, the proportion of the input volume the ellipsoids should take up, the maximum number of iterations</param>
        /// <returns></returns>
        public bool Initialize(InputGeometryParameters inputGeomParams, KangarooGoalParameters kangarooGoalParams, EllipsoidParameters ellipsoidParams, AlgorithmConvergenceParameters algoConvergeParams)
        {
            initializeAlgorithmConvergenceParameters(inputGeomParams, algoConvergeParams);
            initializeKangarooGoals(inputGeomParams, kangarooGoalParams);
            initializeTensorField(inputGeomParams);
            initializeEllipsoids(inputGeomParams, ellipsoidParams);
 
            _isInitialized = true;
            return true;
        }



        private void updateRecentVolumeFillingErrors(double sumOfCurrentEllipsoidVolumes)
        {
            double currentVolumeFillingError = Math.Abs(_targetVolumeToFill - sumOfCurrentEllipsoidVolumes);
            int numberOfInterpolations = (int)Math.Floor((double)_count / _updateInterval);
            _recentVolumeFillingErrors[(int)(numberOfInterpolations % 10)] = currentVolumeFillingError;

        }

        private void updatePersistentGoals(KangarooGoalParameters kangarooGoalParams)
        {
            for (var i = 1; i < _ps.ParticleCount() + 1; i++)
                (_persistentGoalList[i] as AnchorPlastic).Limit = _plasticDragRadius;

            (_persistentGoalList[0] as SolidPoint).Strength = kangarooGoalParams.boundaryCollideStrength;
        }

        private void updateTemporaryGoals(Rhino.Geometry.Point3d[] positions, ref List<IGoal> temporaryGoalList, KangarooGoalParameters kangarooGoalParams)
        {
            //A pair of matching lists to contain the collisions
            var collideRef0 = new List<int>();
            var collideRef1 = new List<int>();

            //use SAP to find AABB collisions
            HelperFunctions.SweepAndPrune(_ellipsoids, positions, ref collideRef0, ref collideRef1);

            //narrow phase collision
            _lineList.Clear();
            for (var i = 0; i < collideRef0.Count(); i++)
            {
                var ea = collideRef0[i];
                var eb = collideRef1[i];
                var collision = new EllipsoidCollide(_ellipsoids[ea], _ellipsoids[eb]);
                var pushDistance = collision.CalculateCollision();


                if (pushDistance != -1)
                {
                    double VolRatio = Math.Pow((_ellipsoids[ea].GetVolume() / _ellipsoids[eb].GetVolume()), 1 / 3.0);
                    temporaryGoalList.Add(new NonLinearRepel(ea, eb, pushDistance, VolRatio, 10, -2));

                    var l = new Line(positions[ea], positions[eb]);

                    temporaryGoalList.Add(new Align(eb, ea,
                        new Vector3d[3]
                        {
                            0.5*(_ellipsoids[ea].UnitXAxis + _ellipsoids[eb].UnitXAxis),
                            0.5*(_ellipsoids[ea].UnitYAxis + _ellipsoids[eb].UnitYAxis),
                            0.5*(_ellipsoids[ea].UnitZAxis + _ellipsoids[eb].UnitZAxis)
                        }, kangarooGoalParams.alignStrength));

                    _lineList.Add(l);
                }
            }
        }

        private double updateEllipsoidsLocally(ref Rhino.Geometry.Point3d[] positions, EllipsoidParameters ellipsoidParams)
        {
            //Ellipsoids, stress tensors and associated parameters
            double sumOfCurrentEllipsoidVolumes = 0.0;
            double damping = 0.95;

            for (var i = 0; i < positions.Length; i++)
            {
                if ((_evec1[i] != null) && (_evec2[i] != null) && (_evec3[i] != null))
                {
                    {
                        //TODO make helperfunction.FirstEvalToLongAxis
                        var clampEval = Math.Max(_eval1[i], _ev1Min);
                        clampEval = Math.Min(clampEval, _ev1Max);
                        var evParam = (clampEval - _ev1Min) / (_ev1Max - _ev1Min);
                        var effectiveLongAxisLength = ellipsoidParams.minLongAxisLength +
                                                      (1.0 - evParam) * (ellipsoidParams.maxLongAxisLength - ellipsoidParams.minLongAxisLength);

                        var a = effectiveLongAxisLength * _scaleEllipsoids;
                        if (_eval1[i] == 0.0)
                        {
                            _evec1[i] = new Vector3d(0.0001, 0.0001, 0.0001);
                            //add runtime warning message here! HOW?
                        }

                        var ratio = Math.Max(ellipsoidParams.minSlenderness, _eval2[i] / _eval1[i]);
                        if (Double.IsNaN(ratio)) ratio = 1.0;
                        var b = a * ratio;

                        if (_eval2[i] == 0.0) _sa[i] = new Vector3d(0.0001, 0.0001, 0.0001);

                        Vector3d newAAxis = damping  * _ellipsoids[i].GetAAxis() + (1.0 - damping) * a * _evec1[i];
                        Vector3d newBAxis = damping  * _ellipsoids[i].GetBAxis() + (1.0 - damping) * b * _evec2[i];
                        Vector3d newCAxis = damping  * _ellipsoids[i].GetCAxis() + (1.0 - damping) * b * _evec3[i];

                        _la[i] = newAAxis;
                        _sa[i] = newBAxis;

                        _ellipsoids[i].UpdateTransform(newAAxis, newBAxis, newCAxis);
                        sumOfCurrentEllipsoidVolumes += 4.0 * Math.PI * newAAxis.Length * newBAxis.Length * newCAxis.Length/3.0;

                        /*double newAAxisLength = (damping * _ellipsoids[i].GetAAxis().Length + (1.0 - damping) * a);
                        double newBAxisLength = (damping * _ellipsoids[i].GetBAxis().Length + (1.0 - damping) * b);
                        double newCAxisLength = (damping * _ellipsoids[i].GetCAxis().Length + (1.0 - damping) * b);

                        _ellipsoids[i].UpdateTransform(newAAxisLength * _evec1[i], newBAxisLength * _evec2[i], newCAxisLength * _evec3[i]);
                        sumOfCurrentEllipsoidVolumes += 4.0 * Math.PI * newAAxisLength * newBAxisLength * newCAxisLength / 3.0;

                        _la[i] = newAAxisLength * _evec1[i];
                        _sa[i] = newBAxisLength * _evec2[i];*/

                    }
                }
            }
            return sumOfCurrentEllipsoidVolumes;
        }
 
        /// <summary>
        /// Function to perform one step of the MAG algorithm
        /// </summary>
        /// <param name="kangarooGoalParams">Set of parameters for the Kangaroo goals: alignStrength, boundaryCollideStrength, distance within which plastic drag will act and indices of the points to be kept fixed</param>
        /// <param name="ellipsoidParams">Set of parameters describing constraints on each ellipsoid: minimum and maximum length for the long axis and minimum ratio of short to long axis</param>
        /// <param name="algoConvergeParams">Set of parameters for how our algorithm will recognize convergence: A boolean determining whether the ellipsoid sizing should get updated or not, the proportion of the input volume the ellipsoids should take up, the maximum number of iterations</param>
        public void Step(KangarooGoalParameters kangarooGoalParams,
                         EllipsoidParameters ellipsoidParams, AlgorithmConvergenceParameters algoConvergeParams)
        {
            if (!_isInitialized)
            {
                return;
            }

            updatePersistentGoals(kangarooGoalParams);
            var positions = _ps.GetPositionArray();
            int ptCount = _ps.ParticleCount();

            if (_count % _updateInterval == 0)
            {
                //map tensor field from grid onto ellipsoid centre points
                HelperFunctions.InterpolateTensor(positions, ref _evec1, ref _evec2, ref _evec3, ref _eval1, ref _eval2, ref _eval3, _grid, _backGroundData);

                double sumOfCurrentEllipsoidVolumes = updateEllipsoidsLocally(ref positions, ellipsoidParams);
               
                if (algoConvergeParams.updateScale)
                {
                    HelperFunctions.updateEllipsoidsGlobally(ref _scaleEllipsoids, ref sumOfCurrentEllipsoidVolumes, ref _targetVolumeToFill);
                }
                updateRecentVolumeFillingErrors(sumOfCurrentEllipsoidVolumes);
                double percentVolPacked = sumOfCurrentEllipsoidVolumes / _meshVolume;
            }

            var temporaryGoalList = new List<IGoal>();
            updateTemporaryGoals(positions, ref temporaryGoalList, kangarooGoalParams);

            temporaryGoalList.AddRange(_persistentGoalList);
            _ps.SimpleStep(temporaryGoalList);
            _count++;

            if (_count > _maxIterations / 10)
            {
                var averageFillingError = _recentVolumeFillingErrors.ToList().Average();
                if ((averageFillingError < 0.0001) || (_count > _maxIterations))
                {
                    acp.isConverged = true;
                }
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

        public KangarooGoalParameters GetKGP()
        {
            return this.kgp;
        }

        public EllipsoidParameters GetEP()
        {
            return this.ep;    
        }
        public AlgorithmConvergenceParameters GetACP()
        {
            return this.acp;
        }

        public InputGeometryParameters GetIGP()
        {
            return this.igp;
        }

        public int GetCount()
        {
            return this._count;
        }

        public bool IsConverged()
        {
            return this.acp.isConverged;
        }


    }
}
