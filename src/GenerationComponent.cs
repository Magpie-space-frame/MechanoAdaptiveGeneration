using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Grasshopper.Kernel;
using KangarooSolver;
using KangarooSolver.Goals;
using MechanoAdaptiveGeneration.customK2goals;
using Rhino.Geometry;

namespace MechanoAdaptiveGeneration
{
    public class GenerationComponent : GH_Component
    {
        //Gridpoints and tensors there
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
        private bool _running;
        private List<Vector3d> _sa;

        private double _scaleEllipsoids;
        private int _updateInterval;

        /// <summary>
        ///     Each implementation of GH_Component must provide a public
        ///     constructor without any arguments.
        ///     Category represents the Tab in which the component will appear,
        ///     Subcategory the panel. If you use non-existing tab or panel names,
        ///     new tabs/panels will automatically be created.
        /// </summary>
        public GenerationComponent()
            : base("MechanoAdaptiveGeneration", "Generate",
                "",
                "MechanoAdaptuve", "")
        {
        }

        /// <summary>
        ///     Provides an Icon for every component that will be visible in the User Interface.
        ///     Icons need to be 24x24 pixels.
        /// </summary>
        protected override Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        ///     Each component must have a unique Guid to identify it.
        ///     It is vital this Guid doesn't change otherwise old ghx files
        ///     that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("{4a5b00d6-e0df-4aba-b028-706d4b71392a}"); }
        }

        /// <summary>
        ///     Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "The volume to fill as a RhinoMesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Points", "P", "The start positions of the particles", GH_ParamAccess.list);
            pManager.AddNumberParameter("Data", "D", "The tensor data for the volume", GH_ParamAccess.list);
            pManager.AddNumberParameter("Options", "O", "The input options for the generation", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Run", "R", "Run the generation", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Reset", "Rst", "Reset the generation", GH_ParamAccess.item);
            pManager.AddBooleanParameter("UpdateScale", "US", "Update the scale during the generation",
                GH_ParamAccess.item);
            pManager.AddBooleanParameter("ValenceFilter", "VF", "Filter the edges depending on valence",
                GH_ParamAccess.item);
            pManager.AddNumberParameter("BoundaryStrength", "BS", "The strength for the boundary collision",
                GH_ParamAccess.item);
            pManager.AddIntegerParameter("FixedPoints", "FP",
                "The indices of any points that should be fixed during the generation", GH_ParamAccess.list);            
            pManager.AddIntegerParameter("MaxIterations", "MI", "The maximum number of iterations for the generation",
                GH_ParamAccess.item);
        }

        /// <summary>
        ///     Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("PercentVolumePacked", "PVP",
                "The percentage of the volume packed with ellipsoids", GH_ParamAccess.item);
            pManager.AddGenericParameter("EllipsoidCentres", "EC", "The locations of the centres for each ellipsoid",
                GH_ParamAccess.list);
            pManager.AddGenericParameter("LongAxes", "LA", "The long axes for each ellipsoid as a Vector3d",
                GH_ParamAccess.list);
            pManager.AddGenericParameter("ShortAxes", "SA", "The short axes for each ellipsoid as a Vector3d",
                GH_ParamAccess.list);
            pManager.AddGenericParameter("Iterations", "I", "The number of iterations so far", GH_ParamAccess.item);
            pManager.AddGenericParameter("Lines", "L", "The lines connecting ellipsoids", GH_ParamAccess.list);
            pManager.AddGenericParameter("Bake", "B", "A boolean to bake the results when ready", GH_ParamAccess.item);
        }

        /// <summary>
        ///     This is the method that actually does the work.
        /// </summary>
        /// <param name="da">
        ///     The DA object can be used to retrieve data from input parameters and
        ///     to store data in output parameters.
        /// </param>
        protected override void SolveInstance(IGH_DataAccess da)
        {
            //get all the data and assign to pointers
            var m = new Mesh();
            da.GetData(0, ref m);
            var pts = new List<Point3d>();
            da.GetDataList(1, pts);
            var data = new List<double>();
            da.GetDataList(2, data);
            var inputOptions = new List<double>();
            da.GetDataList(3, inputOptions);
            var run = new bool();
            da.GetData(4, ref run);
            var reset = new bool();
            da.GetData(5, ref reset);
            var updateScale = new bool();
            da.GetData(6, ref updateScale);
            var valenceFilter = new bool();
            da.GetData(7, ref valenceFilter);
            var boundaryCollideStrength = new double();
            da.GetData(8, ref boundaryCollideStrength);
            var fixedPointIndices = new List<int>();
            da.GetDataList(9, fixedPointIndices);
            var maxIterations = new int();
            da.GetData(10, ref maxIterations);

            //Set up the outputs - the DA.SetData is at the end
            var percentVolPacked = new double();
            var iterations = 0;
            var ellipsoidCenters = new List<Point3d>();
            var bakeResult = new bool();
            var longAxes = new List<Vector3d>();
            var shortAxes = new List<Vector3d>();
            var lines = new List<Line>();

            //the original start of the run script
          
            var meshVolumeProperties = VolumeMassProperties.Compute(m);
            var meshVolume = meshVolumeProperties.Volume;
            double sumOfCurrentEllipsoidVolumes;

            var minLongAxisLength = inputOptions[0];
            var maxLongAxisLength = inputOptions[1];
            var minSlenderness = inputOptions[2];
            var maxRadiusCoefficient = inputOptions[3];
            var initialScaleEllipsoids = inputOptions[4];
            var alignStrength = inputOptions[5];
            var targetPressure = inputOptions[6];
            var edgeLengthFactor = inputOptions[7];
            var plasticdrag = inputOptions[8];

            GenerateSpaceframe(m, pts, data, run, reset, updateScale,
                                valenceFilter, boundaryCollideStrength, 
                                fixedPointIndices, maxIterations, ref percentVolPacked, 
                                ref iterations, ref ellipsoidCenters, out bakeResult, 
                                ref longAxes, ref shortAxes, ref lines, ref meshVolume, 
                                out sumOfCurrentEllipsoidVolumes, minLongAxisLength, 
                                maxLongAxisLength, minSlenderness, maxRadiusCoefficient, 
                                initialScaleEllipsoids, alignStrength, edgeLengthFactor, plasticdrag);


            //set the outputs
            da.SetData(0, percentVolPacked);
            da.SetDataList(1, ellipsoidCenters);
            da.SetDataList(2, longAxes);
            da.SetDataList(3, shortAxes);
            da.SetData(4, iterations);
            da.SetData(5, lines);
            da.SetData(6, bakeResult);
        }

        /// <summary>
        /// This is the generate method
        /// </summary>
        /// <param name="m"> the parameter m! </param>
        /// <param name="pts"></param>
        /// <param name="data"></param>
        /// <param name="run"></param>
        /// <param name="reset"></param>
        /// <param name="updateScale"></param>
        /// <param name="valenceFilter"></param>
        /// <param name="boundaryCollideStrength"></param>
        /// <param name="fixedPointIndices"></param>
        /// <param name="maxIterations"></param>
        /// <param name="percentVolPacked"></param>
        /// <param name="iterations"></param>
        /// <param name="ellipsoidCenters"></param>
        /// <param name="bakeResult"></param>
        /// <param name="longAxes"></param>
        /// <param name="shortAxes"></param>
        /// <param name="lines"></param>
        /// <param name="meshVolume"></param>
        /// <param name="sumOfCurrentEllipsoidVolumes"></param>
        /// <param name="minLongAxisLength"></param>
        /// <param name="maxLongAxisLength"></param>
        /// <param name="minSlenderness"></param>
        /// <param name="maxRadiusCoefficient"></param>
        /// <param name="initialScaleEllipsoids"></param>
        /// <param name="alignStrength"></param>
        /// <param name="edgeLengthFactor"></param>
        /// <param name="plasticdrag"></param>
        public void GenerateSpaceframe(Mesh m, List<Point3d> pts, List<double> data, bool run,
                                        bool reset, bool updateScale, bool valenceFilter, 
                                        double boundaryCollideStrength, List<int> fixedPointIndices, 
                                        int maxIterations, ref double percentVolPacked, 
                                        ref int iterations, ref List<Point3d> ellipsoidCenters, 
                                        out bool bakeResult, ref List<Vector3d> longAxes, 
                                        ref List<Vector3d> shortAxes, ref List<Line> lines, 
                                        ref double meshVolume, out double sumOfCurrentEllipsoidVolumes, 
                                        double minLongAxisLength, double maxLongAxisLength, 
                                        double minSlenderness, double maxRadiusCoefficient, 
                                        double initialScaleEllipsoids, double alignStrength, 
                                        double edgeLengthFactor, double plasticdrag)
        {
            if (reset || !_isInitialized)
            {
                _ps = new PhysicalSystem();
                _goalList = new List<IGoal>();
                _ellipsoids = new List<Ellipsoid>();

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

                _running = false;
                bakeResult = false;
                _ps.ClearParticles();
                _scaleEllipsoids = initialScaleEllipsoids;
                sumOfCurrentEllipsoidVolumes = 0.0;

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
                        sumOfCurrentEllipsoidVolumes += 4.0 * Math.PI * a * b * b / 3.0;
                    }

                _goalList.Add(new SolidPoint(ix, m, true, boundaryCollideStrength));

                //plastic anchor for stopping circulation
                for (var i = 0; i < pts.Count; i++)
                    _goalList.Add(new AnchorPlastic(i, pts[i], plasticdrag, 1000));

                for (var i = 0; i < fixedPointIndices.Count(); i++)
                    _goalList.Add(new Anchor(fixedPointIndices[i], pts[fixedPointIndices[i]], 10000));
                _count = 0;
                //EnergySum = double.MaxValue;

                _isInitialized = true;
                _running = run;
            }
            else
            {
                var totalOverlap = 0.0;
                sumOfCurrentEllipsoidVolumes = 0.0;

                _running = run;
                bakeResult = false;

                for (var i = 1; i < pts.Count + 1; i++)
                    (_goalList[i] as AnchorPlastic).Limit = plasticdrag;

                var goalsToKeep = 1 + pts.Count + fixedPointIndices.Count();
                //Note that if more goals are added in the in the initialization, this number should be increased accordingly
                _goalList.RemoveRange(goalsToKeep, _goalList.Count - goalsToKeep);
                //clear everything apart from the SolidPoint Goal.

                (_goalList[0] as SolidPoint).Strength = boundaryCollideStrength;

                var positions = _ps.GetPositionArray();

                for (var i = 0; i < positions.Count(); i++)
                    if (((_evec1 != null) && (_evec2 != null) && (_evec3 != null)) || true)
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
                                Math.Abs(1.0 - sumOfCurrentEllipsoidVolumes / (1.8 * meshVolume));
                            var numberOfInterpolations = (int)Math.Floor((double)_count / _updateInterval);
                            _recentVolumeFillingErrors[numberOfInterpolations % _updateInterval] =
                                currentVolumeFillingError;
                            HelperFunctions.InterpolateTensor(positions, ref _evec1, ref _evec2, ref _evec3, ref _eval1,
                                ref _eval2, ref _eval3, _grid, _backGroundData);
                        }
                    }

                percentVolPacked = sumOfCurrentEllipsoidVolumes / meshVolume;

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
                        ref _ellipsoids, ref meshVolume);

                //Output the mesh, and how many iterations it took to converge
                ellipsoidCenters = _ps.GetPositions().ToList();

                longAxes = _la;
                shortAxes = _sa;

                _count++;
                if (_count > maxIterations / 10)
                {
                    var averageFillingError = _recentVolumeFillingErrors.ToList().Average();
                    if ((averageFillingError < 0.0001) || (_count > maxIterations))
                    {
                        _running = false;
                        bakeResult = true;
                    }
                }

                iterations = _count;

                //optional post-packing connectivity culling
                if (valenceFilter)
                {
                    var keep = new bool[_lineList.Count];
                    for (var i = 0; i < pts.Count; i++)
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

                    lines = keptLines;
                }
                else
                {
                    lines = _lineList;
                }
            }
        }

        /// <summary>
        ///     Use this to
        /// </summary>
        protected override void AfterSolveInstance()
        {
            if (_running)
            {
                var doc = OnPingDocument();
                GH_Document.GH_ScheduleDelegate callback = ScheduleCallback;
                doc.ScheduleSolution(1, callback);
            }
        }

        private void ScheduleCallback(GH_Document doc)
        {
            ExpireSolution(false);
        }
    }
}