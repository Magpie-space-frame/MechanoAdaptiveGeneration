using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using KangarooSolver;
using System.Linq;

namespace MechanoAdaptiveGeneration
{
    public class GenerationComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public GenerationComponent()
          : base("MechanoAdaptiveGeneration", "Generate",
              "",
              "MechanoAdaptuve", "")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "The volume to fill as a RhinoMesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Points", "P", "The start positions of the particles", GH_ParamAccess.list);
            pManager.AddNumberParameter("Data", "D", "The tensor data for the volume", GH_ParamAccess.list);
            pManager.AddNumberParameter("Options", "O", "The input options for the generation", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Run", "R", "Run the generation", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Reset", "Rst", "Reset the generation", GH_ParamAccess.item);
            pManager.AddBooleanParameter("UpdateScale", "US", "Update the scale during the generation", GH_ParamAccess.item);
            pManager.AddBooleanParameter("ValenceFilter", "VF", "Filter the edges depending on valence", GH_ParamAccess.item);
            pManager.AddNumberParameter("BoundaryStrength", "BS", "The strength for the boundary collision", GH_ParamAccess.item);
            pManager.AddNumberParameter("FixedPoints", "FP", "The indices of any points that should be fixed during the generation", GH_ParamAccess.list);
            pManager.AddNumberParameter("MaxIterations", "MI", "The maximum number of iterations for the generation", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("PercentVolumePacked", "PVP", "The percentage of the volume packed with ellipsoids", GH_ParamAccess.item);
            pManager.AddGenericParameter("EllipsoidCentres", "EC", "The locations of the centres for each ellipsoid", GH_ParamAccess.list);
            pManager.AddGenericParameter("LongAxes", "LA", "The long axes for each ellipsoid as a Vector3d", GH_ParamAccess.list);
            pManager.AddGenericParameter("ShortAxes", "SA", "The short axes for each ellipsoid as a Vector3d", GH_ParamAccess.list);
            pManager.AddGenericParameter("Iterations", "I", "The number of iterations so far", GH_ParamAccess.item);
            pManager.AddGenericParameter("Lines", "L", "The lines connecting ellipsoids", GH_ParamAccess.list);
            pManager.AddGenericParameter("Bake", "B", "A boolean to bake the results when ready", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //get all the data and assign to pointers
            Mesh M = new Mesh();
            DA.GetData(0, ref M);
            List<Point3d> Pts = new List<Point3d>();
            DA.GetDataList(1, Pts);
            List<double> Data = new List<double>();
            DA.GetDataList(2, Data);
            List<double> inputOptions = new List<double>();
            DA.GetDataList(3, inputOptions);
            bool Run = new bool();
            DA.GetData(4, ref Run);
            bool Reset = new bool();
            DA.GetData(5, ref Reset);
            bool UpdateScale = new bool();
            DA.GetData(6, ref UpdateScale);
            bool ValenceFilter = new bool();
            DA.GetData(7, ref ValenceFilter);
            double BoundaryCollideStrength = new double();
            DA.GetData(8, ref BoundaryCollideStrength);
            List<int> FixedPointIndices = new List<int>();
            DA.GetDataList(9, FixedPointIndices);
            int maxIterations = new int();
            DA.GetData(10, ref maxIterations);

            //Set up the outputs - the DA.SetData is at the end
            double percentVolPacked = new double();
            int Iterations = 0;
            List<Point3d> EllipsoidCenters = new List<Point3d>();
            bool bakeResult = new bool();
            List<Vector3d> LongAxes = new List<Vector3d>();
            List<Vector3d> ShortAxes = new List<Vector3d>();
            List<Line> Lines = new List<Line>();


            //the script below the grey line  
            KangarooSolver.PhysicalSystem PS = new KangarooSolver.PhysicalSystem();
            bool isInitialized = false;

            List<IGoal> GoalList = new List<IGoal>();

            List<Ellipsoid> Ellipsoids = new List<Ellipsoid>();

            //set to zero so they are assigned.
            double scaleEllipsoids = 0;
            double ev1mean = 0;
            double ev1stdev = 0;
            double ev1max = 0;
            double ev1min = 0;

            List<Line> LineList = new List<Line>();

            bool Running;
            int count = 0;
            int updateInterval = 10;
            double[] recentVolumeFillingErrors = new double[10];

            List<Vector3d> Evec1 = new List<Vector3d>();
            List<Vector3d> Evec2 = new List<Vector3d>();
            List<Vector3d> Evec3 = new List<Vector3d>();

            List<double> Eval1 = new List<double>();
            List<double> Eval2 = new List<double>();
            List<double> Eval3 = new List<double>();

            List<double> localData = new List<double>();

            List<Vector3d> La = new List<Vector3d>();
            List<Vector3d> Sa = new List<Vector3d>();

            //Gridpoints and tensors there
            BackGroundData backGroundData = new BackGroundData();
            StressTensor[,,] grid = new StressTensor[0, 0, 0];

            //the original start of the run script
            int numberOfOptionSets = inputOptions.Count / 9;
            var meshVolumeProperties = VolumeMassProperties.Compute(M);
            double meshVolume = meshVolumeProperties.Volume;
            double sumOfCurrentEllipsoidVolumes;

            for (int currentOptionSet = 0; currentOptionSet < 1; currentOptionSet++)
            {
                double minLongAxisLength = inputOptions[currentOptionSet];
                double maxLongAxisLength = inputOptions[currentOptionSet + 1];
                double minSlenderness = inputOptions[currentOptionSet + 2];
                double maxRadiusCoefficient = inputOptions[currentOptionSet + 3];
                double initialScaleEllipsoids = inputOptions[currentOptionSet + 4];
                double AlignStrength = inputOptions[currentOptionSet + 5];
                double targetPressure = inputOptions[currentOptionSet + 6];
                double EdgeLengthFactor = inputOptions[currentOptionSet + 7];
                double plasticdrag = inputOptions[currentOptionSet + 8];

                if (Reset || !isInitialized)
                {
                    Running = false;
                    bakeResult = false;
                    PS.ClearParticles();
                    scaleEllipsoids = initialScaleEllipsoids;
                    sumOfCurrentEllipsoidVolumes = 0.0;

                    GoalList.Clear();
                    Ellipsoids.Clear();
                    LineList.Clear();

                    Evec1.Clear();
                    Evec2.Clear();
                    Evec3.Clear();

                    Eval1.Clear();
                    Eval2.Clear();
                    Eval3.Clear();

                    localData.Clear();
                    localData = Data;
                    HelperFunctions.processData(localData, ref backGroundData, ref grid);

                    La.Clear();
                    Sa.Clear();

                    HelperFunctions.InterpolateTensor(Pts.ToArray(), ref Evec1, ref Evec2, ref Evec3, ref Eval1, ref Eval2, ref Eval3,grid,backGroundData);

                    //ellipsoid size is in range mean(ev1) +- 2 stdDev(ev1)
                    ev1mean = Eval1.Average();
                    ev1stdev = HelperFunctions.StdDev(ref Eval1);
                    ev1max = ev1mean + 2 * ev1stdev;
                    ev1min = ev1mean - 2 * ev1stdev;

                    //ev1max = ev1mean * 1.5;
                    //ev1min = ev1mean * 0.5;
                    
                    var ix = new List<int>();

                    if (Evec1 != null && Evec2 != null && Evec3 != null || true)
                    {
                        for (int i = 0; i < Pts.Count; i++)
                        {
                            PS.AddParticle(Pts[i], 1); //add a particle for every point
                            ix.Add(i);
                            Ellipsoids.Add(new Ellipsoid(Pts[i]));

                            //translate these values to rs and rl

                            double ClampEval = Math.Max(Eval1[i], ev1min);
                            ClampEval = Math.Min(ClampEval, ev1max);

                            double evParam = (ClampEval - ev1min) / (ev1max - ev1min);

                            double effectiveLongAxisLength = minLongAxisLength + (1 - evParam) * (maxLongAxisLength - minLongAxisLength);

                            // double effectiveLongAxisLength = minLongAxisLength + (Eval1[i] - ev1max) / (ev1min - ev1max) * (maxLongAxisLength - minLongAxisLength);

                            //   effectiveLongAxisLength = Math.Max(Math.Min(Math.Abs(effectiveLongAxisLength), maxLongAxisLength / scaleEllipsoids), minLongAxisLength / scaleEllipsoids);
                            La.Add(scaleEllipsoids * effectiveLongAxisLength * Evec1[i]);

                            //double ratio = Math.Max(Math.Min(Math.Abs(Eval2[i]), maxLongAxisLength / scaleEllipsoids) / effectiveLongAxisLength, minSlenderness);
                            double ratio = Math.Max(minSlenderness, Eval2[i] / Eval1[i]);

                            Sa.Add(scaleEllipsoids * ratio * effectiveLongAxisLength * Evec2[i]);

                            //if this ellipsoid is a sphere, it can have maximally a radius of maxRadiusCofficient*maxLongAxisLength; This is to restrict the volume.
                            double volumeRatio = maxRadiusCoefficient * maxLongAxisLength / (Math.Pow(ratio * ratio, 0.33333333) * effectiveLongAxisLength);
                            if (volumeRatio < 1)
                            {
                                Sa[i] *= volumeRatio;
                                La[i] *= volumeRatio;
                                effectiveLongAxisLength *= volumeRatio;
                            }

                            Ellipsoids[i].updateTransform(scaleEllipsoids * effectiveLongAxisLength * Evec1[i], scaleEllipsoids * ratio * effectiveLongAxisLength * Evec2[i], scaleEllipsoids * ratio * effectiveLongAxisLength * Evec3[i]);
                            double a = scaleEllipsoids * effectiveLongAxisLength;
                            double b = scaleEllipsoids * ratio * effectiveLongAxisLength;
                            sumOfCurrentEllipsoidVolumes += 4.0 * Math.PI * a * b * b / 3.0;
                        }
                    }

                    GoalList.Add(new KangarooSolver.Goals.SolidPoint(ix, M, true, BoundaryCollideStrength));

                    //plastic anchor for stopping circulation
                    for (int i = 0; i < Pts.Count; i++)
                    {
                        GoalList.Add(new customK2goals.AnchorPlastic(i, Pts[i], plasticdrag, 1000));
                    }

                    for (int i = 0; i < FixedPointIndices.Count(); i++)
                    {
                        GoalList.Add(new KangarooSolver.Goals.Anchor(FixedPointIndices[i], Pts[FixedPointIndices[i]], 10000));
                    }
                    count = 0;
                    //EnergySum = double.MaxValue;

                    isInitialized = true;
                }
                else
                {
                    double totalOverlap = 0.0;
                    sumOfCurrentEllipsoidVolumes = 0.0;

                    Running = Run;
                    bakeResult = false;

                    for (int i = 1; i < Pts.Count + 1; i++)
                    {
                        (GoalList[i] as customK2goals.AnchorPlastic).Limit = plasticdrag;
                    }

                    int GoalsToKeep = 1 + Pts.Count + FixedPointIndices.Count(); //Note that if more goals are added in the in the initialization, this number should be increased accordingly
                    GoalList.RemoveRange(GoalsToKeep, GoalList.Count - GoalsToKeep); //clear everything apart from the SolidPoint Goal.

                    (GoalList[0] as KangarooSolver.Goals.SolidPoint).Strength = BoundaryCollideStrength;

                    var Positions = PS.GetPositionArray();

                    for (int i = 0; i < Positions.Count(); i++)
                        if (Evec1 != null && Evec2 != null && Evec3 != null || true)
                        {
                            {
                                double ClampEval = Math.Max(Eval1[i], ev1min);
                                ClampEval = Math.Min(ClampEval, ev1max);
                                double evParam = (ClampEval - ev1min) / (ev1max - ev1min);
                                double effectiveLongAxisLength = minLongAxisLength + (1 - evParam) * (maxLongAxisLength - minLongAxisLength);

                                // double effectiveLongAxisLength = minLongAxisLength + (Eval1[i] - ev1max) / (ev1min - ev1max) * (maxLongAxisLength - minLongAxisLength);
                                //   effectiveLongAxisLength = Math.Max(Math.Min(Math.Abs(effectiveLongAxisLength), maxLongAxisLength / scaleEllipsoids), minLongAxisLength / scaleEllipsoids);
                                La[i] = scaleEllipsoids * effectiveLongAxisLength * Evec1[i];

                                //double ratio = Math.Max(Math.Min(Math.Abs(Eval2[i]), maxLongAxisLength / scaleEllipsoids) / effectiveLongAxisLength, minSlenderness);
                                double ratio = Math.Max(minSlenderness, Eval2[i] / Eval1[i]);

                                Sa[i] = scaleEllipsoids * ratio * effectiveLongAxisLength * Evec2[i];

                                //if this ellipsoid is a sphere, it can have maximally a radius of maxRadiusCofficient*maxLongAxisLength; This is to restrict the volume.
                                double volumeRatio = maxRadiusCoefficient * maxLongAxisLength / (Math.Pow(ratio * ratio, 0.33333333) * effectiveLongAxisLength);
                                if (volumeRatio < 1)
                                {
                                    Sa[i] *= volumeRatio;
                                    La[i] *= volumeRatio;
                                    effectiveLongAxisLength *= volumeRatio;
                                }

                                Ellipsoids[i].updateTransform(scaleEllipsoids * effectiveLongAxisLength * Evec1[i], scaleEllipsoids * ratio * effectiveLongAxisLength * Evec2[i], scaleEllipsoids * ratio * effectiveLongAxisLength * Evec3[i]);
                                double a = scaleEllipsoids * effectiveLongAxisLength;
                                double b = scaleEllipsoids * ratio * effectiveLongAxisLength;
                                sumOfCurrentEllipsoidVolumes += 4.0 * Math.PI * a * b * b / 3.0;
                            }

                            if (count % updateInterval == 1)
                            {
                                //update the ellipsoid transformations from field
                                double currentVolumeFillingError = Math.Abs(1.0 - sumOfCurrentEllipsoidVolumes / (1.8 * meshVolume));
                                int numberOfInterpolations = (int)Math.Floor((double)count / updateInterval);
                                recentVolumeFillingErrors[(int)(numberOfInterpolations % updateInterval)] = currentVolumeFillingError;
                                HelperFunctions.InterpolateTensor(Positions, ref Evec1, ref Evec2, ref Evec3, ref Eval1, ref Eval2, ref Eval3,grid,backGroundData);
                            }
                        }

                    percentVolPacked = sumOfCurrentEllipsoidVolumes / meshVolume;

                    //A pair of matching lists to contain the collisions
                    List<int> CollideRef0 = new List<int>();
                    List<int> CollideRef1 = new List<int>();

                    //use SAP to find AABB collisions

                    HelperFunctions.SweepAndPrune(Ellipsoids, Positions, ref CollideRef0, ref CollideRef1);

                    //narrow phase collision

                    var ConnectedEdges = new List<int>[Positions.Length];
                    for (int i = 0; i < Positions.Length; i++)
                    {
                        ConnectedEdges[i] = new List<int>();
                    }

                    LineList.Clear();
                    for (int i = 0; i < CollideRef0.Count(); i++)
                    {
                        int ea = CollideRef0[i];
                        int eb = CollideRef1[i];
                        var Collision = new customK2goals.EllipsoidCollide(Ellipsoids[ea], Ellipsoids[eb]);
                        double PushDistance = Collision.CalculateCollision();

                        if (PushDistance != -1)
                        {
                            //GoalList.Add(new KangarooSolver.Goals.Spring(eb, ea, PushDistance, 1));

                            GoalList.Add(new customK2goals.NonLinearRepel(eb, ea, PushDistance, 1, 0.01, -2));

                            Line L = new Line(Positions[ea], Positions[eb]);

                            GoalList.Add(new Align(eb, ea, new Vector3d[3]
                              {
              0.5 * (Ellipsoids[ea].unitXAxis + Ellipsoids[eb].unitXAxis),
              0.5 * (Ellipsoids[ea].unitYAxis + Ellipsoids[eb].unitYAxis),
              0.5 * (Ellipsoids[ea].unitZAxis + Ellipsoids[eb].unitZAxis),
                              }, AlignStrength));

                            //
                            //          if(PushDistance > EdgeLengthFactor)
                            //          {
                            //            LineList.Add(L);
                            //          }

                            double LLen = L.Length;

                            if ((PushDistance - LLen) / LLen > EdgeLengthFactor)
                            {
                                LineList.Add(L);
                                ConnectedEdges[ea].Add(LineList.Count - 1);
                                ConnectedEdges[eb].Add(LineList.Count - 1);
                            }

                            //
                            //          if((PushDistance - LLen) > EdgeLengthFactor)
                            //          {
                            //            LineList.Add(L);
                            //          }


                            if (!Double.IsNaN(PushDistance))
                            {
                                totalOverlap = totalOverlap + 2 * Math.Abs(PushDistance);
                            }
                        }
                    }

                    //create spring forces and add to goals
                    //for all pairs colliding, also add alignment

                    PS.SimpleStep(GoalList);

                    //comment out the scaling method you don't want
                    if (UpdateScale && count % updateInterval == 1)
                    {
                        HelperFunctions.updateScaleByVolume(ref scaleEllipsoids, ref sumOfCurrentEllipsoidVolumes, ref Ellipsoids, ref meshVolume);
                        //updateScaleByPressure(ref scaleEllipsoids, ref totalOverlap, ref Ellipsoids, ref targetPressure);
                    }

                    //Output the mesh, and how many iterations it took to converge
                    EllipsoidCenters = (List<Point3d>) PS.GetPositions();

                    LongAxes = La;
                    ShortAxes = Sa;

                    count++;
                    if (count > maxIterations / 10)
                    {
                        double averageFillingError = recentVolumeFillingErrors.ToList().Average();
                        if (averageFillingError < 0.01 || count > maxIterations)
                        {
                            Running = false;
                            bakeResult = true;
                        }
                    }

                    Iterations = count;


                    //optional post-packing connectivity culling
                    if (ValenceFilter)
                    {
                        var Keep = new bool[LineList.Count];
                        for (int i = 0; i < Pts.Count; i++)
                        {
                            int CurrentValence = ConnectedEdges[i].Count;
                            var EdgeIndices = new int[CurrentValence];
                            int TargetValence = 6;// TODO: Replace this for boundary faces/edges

                            var EdgeLengths = new double[CurrentValence];

                            for (int j = 0; j < CurrentValence; j++)
                            {
                                EdgeLengths[j] = LineList[ConnectedEdges[i][j]].Length;
                                EdgeIndices[j] = j;
                            }

                            Array.Sort(EdgeLengths, EdgeIndices);

                            int Maximum = Math.Min(TargetValence, ConnectedEdges[i].Count);
                            for (int j = 0; j < Maximum; j++)
                            {
                                Keep[ConnectedEdges[i][EdgeIndices[j]]] = true;
                            }
                        }

                        var KeptLines = new List<Line>();
                        for (int i = 0; i < LineList.Count; i++)
                        {
                            if (Keep[i])
                            {
                                KeptLines.Add(LineList[i]);
                            }
                        }

                        Lines = KeptLines;
                    }
                    else
                    {
                        Lines = LineList;
                    }
                }
            }

            //set the outputs
            DA.SetData(0, percentVolPacked);
            DA.SetDataList(1, EllipsoidCenters);
            DA.SetDataList(2, LongAxes);
            DA.SetDataList(3, ShortAxes);
            DA.SetData(4, Iterations);
            DA.SetData(5, Lines);
            DA.SetData(6, bakeResult);
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("{4a5b00d6-e0df-4aba-b028-706d4b71392a}"); }
        }
    }
}
