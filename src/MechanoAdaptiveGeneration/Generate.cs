using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using KangarooSolver;
using System.Linq;
using MechanoAdaptiveGeneration;
using MechanoAdaptiveGeneration.customK2goals;

public class Generate
{
	public Generate (Mesh M, List<Point3d> Pts, List<double> Data, List<double> inputOptions, bool UpdateScale, bool ValenceFilter, double BoundaryCollideStrength, List<int> FixedPointIndices)
	{
        this.M = M;
        this.Pts = Pts;
        this.Data = Data;
        this.inputOptions = inputOptions;
        this.UpdateScale = UpdateScale;
        this.ValenceFilter = ValenceFilter;
        this.BoundaryCollideStrength = BoundaryCollideStrength;
        this.FixedPointIndices = FixedPointIndices;

        percentVolPacked = new double();
        Iterations = 0;
        EllipsoidCenters = new List<Point3d>();
        bakeResult = new bool();
        LongAxes = new List<Vector3d>();
        ShortAxes = new List<Vector3d>();
        Lines = new List<Line>();
    }

    public void Reset()
    {
        PS = new KangarooSolver.PhysicalSystem();
        GoalList = new List<IGoal>();
        Ellipsoids = new List<Ellipsoid>();

        //set to zero so they are assigned.
        scaleEllipsoids = 0;
        ev1mean = 0;
        ev1stdev = 0;
        ev1max = 0;
        ev1min = 0;

        LineList = new List<Line>();

        count = 0;
        updateInterval = 1;
        recentVolumeFillingErrors = new double[10];


        Eval1 = new List<double>();
        Eval2 = new List<double>();
        Eval3 = new List<double>();

        Evec1 = new List<Vector3d>();
        Evec2 = new List<Vector3d>();
        Evec3 = new List<Vector3d>();

        localData = new List<double>();

        La = new List<Vector3d>();
        Sa = new List<Vector3d>();

        //Gridpoints and tensors there
        backGroundData = new BackGroundData();
        grid = new StressTensor[0, 0, 0];

        PS.ClearParticles();
        scaleEllipsoids = initialScaleEllipsoids;
        sumOfCurrentEllipsoidVolumes = 0.0;

        GoalList.Clear();
        Ellipsoids.Clear();
        LineList.Clear();

        localData.Clear();
        localData = Data;
        HelperFunctions.processData(localData, ref backGroundData, ref grid);

        La.Clear();
        Sa.Clear();

        HelperFunctions.InterpolateTensor(Pts.ToArray(), ref Evec1, ref Evec2, ref Evec3, ref Eval1, ref Eval2, ref Eval3, grid, backGroundData);

        //ellipsoid size is in range mean(ev1) +- 2 stdDev(ev1)
        ev1mean = Eval1.Average();
        ev1stdev = HelperFunctions.StdDev(ref Eval1);
        ev1max = ev1mean + 2 * ev1stdev;
        ev1min = ev1mean - 2 * ev1stdev;

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
            GoalList.Add(new AnchorPlastic(i, Pts[i], plasticdrag, 1000));
        }

        for (int i = 0; i < FixedPointIndices.Count(); i++)
        {
            GoalList.Add(new KangarooSolver.Goals.Anchor(FixedPointIndices[i], Pts[FixedPointIndices[i]], 10000));
        }
        count = 0;
        //EnergySum = double.MaxValue;

        meshVolumeProperties = VolumeMassProperties.Compute(M);
        meshVolume = meshVolumeProperties.Volume;
    }

    public void Step()
    {
           var meshVolumeProperties = VolumeMassProperties.Compute(M);
           double meshVolume = meshVolumeProperties.Volume;
           double sumOfCurrentEllipsoidVolumes;
  
           double minLongAxisLength = inputOptions[0];
           double maxLongAxisLength = inputOptions[1];
           double minSlenderness = inputOptions[2];
           double maxRadiusCoefficient = inputOptions[3];
           double initialScaleEllipsoids = inputOptions[4];
           double AlignStrength = inputOptions[5];
           double targetPressure = inputOptions[6];
           double EdgeLengthFactor = inputOptions[7];
           double plasticdrag = inputOptions[8];

           double totalOverlap = 0.0;
           sumOfCurrentEllipsoidVolumes = 0.0;

           for (int i = 1; i < Pts.Count + 1; i++)
           {
               (GoalList[i] as MechanoAdaptiveGeneration.customK2goals.AnchorPlastic).Limit = plasticdrag;
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
               var Collision = new EllipsoidCollide(Ellipsoids[ea], Ellipsoids[eb]);
               double PushDistance = Collision.CalculateCollision();

               if (PushDistance != -1)
               {
                   //GoalList.Add(new KangarooSolver.Goals.Spring(eb, ea, PushDistance, 1));

                   GoalList.Add(new NonLinearRepel(eb, ea, PushDistance, 1, 0.01, -2));

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
                    EllipsoidCenters = PS.GetPositions().ToList();

                    LongAxes = La;
                    ShortAxes = Sa;

                    double averageFillingError = recentVolumeFillingErrors.ToList().Average();

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

    KangarooSolver.PhysicalSystem PS;
    List<IGoal> GoalList;
    List<Ellipsoid> Ellipsoids;
    double scaleEllipsoids;
    double ev1mean;
    double ev1stdev;
    double ev1max;
    double ev1min;
    List<Line> LineList = new List<Line>();
    int count;
    int updateInterval;
    double[] recentVolumeFillingErrors;
    List<Vector3d> Evec1;
    List<Vector3d> Evec2;
    List<Vector3d> Evec3;
    List<double> Eval1;
    List<double> Eval2;
    List<double> Eval3;
    List<double> localData;
    List<Vector3d> La;
    List<Vector3d> Sa;

    //Gridpoints and tensors there
    BackGroundData backGroundData;
    StressTensor[, ,] grid;

    public Mesh M;           
    public List<Point3d> Pts;           
    public List<double> Data;            
    public List<double> inputOptions;         
    public bool UpdateScale;
    public bool ValenceFilter;
    public double BoundaryCollideStrength;          
    public List<int> FixedPointIndices;

    public double percentVolPacked;
    public int Iterations;
    public List<Point3d> EllipsoidCenters;
    public bool bakeResult;
    public List<Vector3d> LongAxes;
    public List<Vector3d> ShortAxes;
    public List<Line> Lines;

    VolumeMassProperties meshVolumeProperties;
    double meshVolume;
    double sumOfCurrentEllipsoidVolumes;

    double minLongAxisLength ;
    double maxLongAxisLength;
    double minSlenderness ;
    double maxRadiusCoefficient ;
    double initialScaleEllipsoids;
    double AlignStrength ;
    double targetPressure ;
    double EdgeLengthFactor ;
    double plasticdrag ;
}
