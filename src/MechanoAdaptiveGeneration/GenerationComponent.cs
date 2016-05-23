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
        bool isInitialized = false;
        //the script below the grey line
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
        StressTensor[,,] grid;

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
            pManager.AddIntegerParameter("FixedPoints", "FP", "The indices of any points that should be fixed during the generation", GH_ParamAccess.list);
            pManager.AddIntegerParameter("MaxIterations", "MI", "The maximum number of iterations for the generation", GH_ParamAccess.item);

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
            DA.GetData<int>(10, ref maxIterations);
            

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
