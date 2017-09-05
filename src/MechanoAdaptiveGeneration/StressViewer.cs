using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;


namespace MechanoAdaptiveGeneration
{
    /// <summary>
    /// 
    /// </summary>
    public class StressViewer : GH_Component
    {

        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public StressViewer()
          : base("StressViewer", "StressView",
              "Visualize stress data as oriented and scaled boxes",
              "MechanoAdaptive", "")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Data", "D", "Stress data as a flat list of numbers, where each consecutive set of 9 values is of the format: position(x,y,z), stress tensor(00,01,02,11,12,22)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Scale", "S", "Scaling factor for output", GH_ParamAccess.item, 0.1);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Boxes", "B", "Stress field visualized as boxes", GH_ParamAccess.list);            
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {

            List<double> Data = new List<double>();
            DA.GetDataList(0, Data);

            double scale = new double();
            DA.GetData(1, ref scale);

            var boxes = new List<object>();
            for (int i = 0; i < (Data.Count / 9); i++)
            {
                double[] vals = new double[9];
                List<double> x = Data.GetRange(i * 9, 9);
                if (!x.Contains(double.NaN))
                {
                    Point3d Pt = new Point3d(x[0], x[1], x[2]);
                    double[] Stress = new double[9] { x[3], x[4], x[5], x[4], x[6], x[7], x[5], x[7], x[8] };
                    double[] EigenVals;
                    Vector3d[] EigenVecs;
                    HelperFunctions.EigenSolve(Stress, out EigenVals, out EigenVecs);

                    Box B = new Box(new Plane(Pt, EigenVecs[0], EigenVecs[1]),
                      new Interval(-EigenVals[0] * scale, EigenVals[0] * scale),
                      new Interval(-EigenVals[1] * scale, EigenVals[1] * scale),
                      new Interval(-EigenVals[2] * scale, EigenVals[2] * scale));

                    boxes.Add(B);
                }
                else
                {
                    boxes.Add(null);
                }
            }

            DA.SetDataList(0, boxes);            
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
                return MechanoAdaptiveGeneration.Properties.Resources.boxes;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("5D82D4A6-5EB4-4345-B7D3-E4C9B27C4FEB"); }
        }
    }
}
