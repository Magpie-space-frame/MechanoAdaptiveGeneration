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
        private bool Running;
        MechanoAdaptiveGeneration.Generator Gen;
        private InputGeometryParameters IGP;
        private KangarooGoalParameters KGP;
        private EllipsoidParameters EP;
        private AlgorithmConvergenceParameters ACP;

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
              "MechanoAdaptive", "")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Save", "Save", "Boolean input, save Magpie results when true", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Reset", "Rst", "Reset the generation", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Run", "R", "Run the generation", GH_ParamAccess.item);
            pManager.AddMeshParameter("Mesh", "M", "The volume to fill as a RhinoMesh", GH_ParamAccess.item);
            pManager.AddMeshParameter("Mesh", "S", "The surface to constrain points onto (optional)", GH_ParamAccess.item);
            pManager.AddPointParameter("Points", "P", "The start positions of the particles", GH_ParamAccess.list);
            pManager.AddNumberParameter("Data", "D", "The tensor data for the volume", GH_ParamAccess.list);
            pManager.AddIntegerParameter("FixedPoints", "FP", "The indices of any points that should be fixed during the generation", GH_ParamAccess.list);
            pManager.AddNumberParameter("Options", "O", "The input options for the generation", GH_ParamAccess.list);
            pManager.AddNumberParameter("VolumeFactor", "VF", "The multiple of the input volume the total ellipsoid volume should take up", GH_ParamAccess.item);
            pManager.AddGenericParameter("Output file name", "file", "String containing the path to the file the results are saved to", GH_ParamAccess.item);

            pManager[4].Optional = true;
            pManager[7].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("EllipsoidCentres", "EC", "The locations of the centres for each ellipsoid", GH_ParamAccess.list);
            pManager.AddGenericParameter("LongAxes", "LA", "The long axes for each ellipsoid as a Vector3d", GH_ParamAccess.list);
            pManager.AddGenericParameter("ShortAxes", "SA", "The short axes for each ellipsoid as a Vector3d", GH_ParamAccess.list);
            pManager.AddGenericParameter("PercentVolumePacked", "PVP", "The percentage of the volume packed with ellipsoids", GH_ParamAccess.item);
            pManager.AddGenericParameter("Iterations", "I", "The number of iterations so far", GH_ParamAccess.item);
            pManager.AddGenericParameter("Lines", "L", "The lines connecting ellipsoids", GH_ParamAccess.list);
            pManager.AddGenericParameter("Bake", "B", "A boolean to bake the results when ready", GH_ParamAccess.item);
            pManager.AddGenericParameter("RecentErrors", "RE", "Last ten deviations from target volume", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {

            //get all the data and assign to pointers
            int i = 0;
            bool saveNow = new bool();
            DA.GetData(i++, ref saveNow);
            bool Reset = new bool();
            DA.GetData(i++, ref Reset);
            bool Run = new bool();
            DA.GetData(i++, ref Run);

            Mesh M = new Mesh();
            DA.GetData(i++, ref M);

            Mesh S = new Mesh();
            if (!DA.GetData(i++, ref S))
            {
                S = null;
            }
            
            List<Point3d> Pts = new List<Point3d>();
            DA.GetDataList(i++, Pts);
            List<double> Data = new List<double>();
            DA.GetDataList(i++, Data);
            List<int> FixedPointIndices = new List<int>();
            DA.GetDataList(i++, FixedPointIndices);
            List<double> inputOptions = new List<double>();
            DA.GetDataList(i++, inputOptions);
            double volumeFactor = new double();
            DA.GetData<double>(i++, ref volumeFactor);
            string outputPath= "";
            DA.GetData<string>(i++, ref outputPath);

            bool UpdateScale = true;
            double BoundaryCollideStrength = 1e8;
            double AlignStrength = inputOptions[3];
            double plasticdragDist = inputOptions[4];
            int maxIterations = 1000;

            double minLongAxisLength = inputOptions[0];
            double maxLongAxisLength = inputOptions[1];
            double minSlenderness = inputOptions[2];

            if (Reset)
            {
                Gen = new MechanoAdaptiveGeneration.Generator();
                IGP = new InputGeometryParameters(M, S, Pts, Data);
                KGP = new KangarooGoalParameters(plasticdragDist, BoundaryCollideStrength, AlignStrength, FixedPointIndices);
                EP = new EllipsoidParameters(minLongAxisLength, maxLongAxisLength, minSlenderness);
                ACP = new AlgorithmConvergenceParameters(volumeFactor, maxIterations, UpdateScale);
                Gen.Initialize(IGP, KGP, EP, ACP);
            }

            if (Run && !Gen.IsConverged())
            {
                Gen.Step(KGP, EP, ACP);
            }

            if (saveNow)
            {
                MagpieWriter writer = new MagpieWriter();
                MagpieResults results = new MagpieResults();
                results.packing = new EllipsoidPacking(Gen.GetCentres(), Gen.GetLongAxes(), Gen.GetShortAxes());
                results.kgp = Gen.GetKGP();
                results.ep = Gen.GetEP();
                results.acp = Gen.GetACP();
                results.igp = Gen.GetIGP();
                writer.write(outputPath, results);
            }

            //set the outputs
            List<Point3d> centres = new List<Point3d>();
            List<Vector3d> longAxes = new List<Vector3d>();
            List<Vector3d> shortAxes = new List<Vector3d>();
            double percentVolPacked = new double();
            int Iterations = Gen.GetCount();
            bool bakeResult = new bool();
            List<Double> RecentErrors = new List<Double>();
            List<Line> Lines = new List<Line>();

            centres = Gen.GetCentres();
            longAxes = Gen.GetLongAxes();
            shortAxes = Gen.GetShortAxes();
            Lines = Gen.GetLines();

            DA.SetDataList(0, centres);
            DA.SetDataList(1, longAxes);
            DA.SetDataList(2, shortAxes);

            DA.SetData(3, percentVolPacked);
            DA.SetData(4, Iterations);
            DA.SetDataList(5, Lines);
            DA.SetData(6, bakeResult);
            DA.SetDataList(7, RecentErrors);
        }

        /// <summary>
        /// Use this to 
        /// </summary>
        protected override void AfterSolveInstance()
        {
            if (Running)
            {
                GH_Document doc = OnPingDocument();
                GH_Document.GH_ScheduleDelegate callback = new GH_Document.GH_ScheduleDelegate(ScheduleCallback);
                doc.ScheduleSolution(1, callback);
            }
        }

        private void ScheduleCallback(GH_Document doc)
        {
            ExpireSolution(false);
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
                return MechanoAdaptiveGeneration.Properties.Resources.mapgieIcon;
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
