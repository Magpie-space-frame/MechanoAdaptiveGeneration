using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using KangarooSolver;
using System.Linq;

namespace MechanoAdaptiveGeneration
{
    public class ReadComponent : GH_Component
    {
        private MagpieResults results;
        
        // <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public ReadComponent()
          : base("MagpieReader", "OpenMagpieResults",
              "",
              "MechanoAdaptive", "")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Read", "Read", "Boolean input, read file when true", GH_ParamAccess.item);
            pManager.AddGenericParameter("Input file path", "path", "String containing the path to a Magpie file", GH_ParamAccess.item);
        }
        
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            //ellipsoid packing
            pManager.AddGenericParameter("EllipsoidCentres", "EC", "The locations of the centres for each ellipsoid", GH_ParamAccess.list);
            pManager.AddGenericParameter("LongAxes", "LA", "The long axes for each ellipsoid as a Vector3d", GH_ParamAccess.list);
            pManager.AddGenericParameter("ShortAxes", "SA", "The short axes for each ellipsoid as a Vector3d", GH_ParamAccess.list);

            //kangaroo goal parameters
            pManager.AddGenericParameter("Plastic Drag distance", "drag", "The Plastic drag distance used in the plastic anchor goal", GH_ParamAccess.item);
            pManager.AddGenericParameter("Boundary Collide Strength", "boundary", "Kangaroo weight used for collision of ellipsoids with boundary", GH_ParamAccess.item);
            pManager.AddGenericParameter("Align Strength", "align", "The align strength weight used for the align goal", GH_ParamAccess.item);
            pManager.AddGenericParameter("Fixed Point indices", "fixed", "Indices of input points that were kept fixed", GH_ParamAccess.list);

            //ellipsoid parameters
            pManager.AddGenericParameter("Minimum length of short axis", "minSA", "The minimum length a short axis can be", GH_ParamAccess.item);
            pManager.AddGenericParameter("Maximum length of short axis", "maxSA", "The maximum length a short axis can be", GH_ParamAccess.item);
            pManager.AddGenericParameter("Minimum aspect ratio", "minAR", "The smallest aspect ratio (long:short) an ellipsoid can have", GH_ParamAccess.item);

            //algorithm convergence parameters
            pManager.AddGenericParameter("Volume Factor", "VF", "The multiple of the input volume the converged ellipsoids should make up in total", GH_ParamAccess.item);
            pManager.AddGenericParameter("Maximum iterations", "maxIt", "The maximum iterations allowed", GH_ParamAccess.item);
            pManager.AddGenericParameter("UpdateScale", "update", "Specifies whether the scale of the ellipsoids should be updated", GH_ParamAccess.item);
            pManager.AddGenericParameter("Parameters unchanged", "unchanged", "Specifies whether all parameters remained the same throughout packing process", GH_ParamAccess.item);

            //input geometry parameters
            pManager.AddGenericParameter("Input Mesh", "mesh", "Mesh delimiting the input volume", GH_ParamAccess.item);
            pManager.AddGenericParameter("Surface Mesh", "surf", "Mesh specifying the optional input surface", GH_ParamAccess.item);
            pManager.AddGenericParameter("Input points", "input points", "List of Point3d the generation started with", GH_ParamAccess.list);
            pManager.AddGenericParameter("Input field", "field", "Input field for the generation algorithm", GH_ParamAccess.list);
        }

        protected override void SolveInstance(Grasshopper.Kernel.IGH_DataAccess DA)
        {
            bool readNow = new bool();
            DA.GetData(0, ref readNow);
            string inputPath = "";
            DA.GetData(1, ref inputPath);

            List<Point3d> centres = new List<Point3d>();
            List<Vector3d> longAxes = new List<Vector3d>();
            List<Vector3d> shortAxes = new List<Vector3d>();

            MagpieReader magpieReader = new MagpieReader();
            
            if (readNow)
            {
                results = magpieReader.read(inputPath);
            }

            int i = 0;
            centres = results.packing.GetCentres();
            longAxes = results.packing.GetLongAxes();
            shortAxes = results.packing.GetShortAxes();
            DA.SetDataList(i++, centres);
            DA.SetDataList(i++, longAxes);
            DA.SetDataList(i++, shortAxes);

            KangarooGoalParameters kgp = results.kgp;
            DA.SetData(i++, kgp.plasticDragDistance);
            DA.SetData(i++, kgp.boundaryCollideStrength);
            DA.SetData(i++, kgp.alignStrength);
            DA.SetDataList(i++, kgp.fixedPointIndices);

            EllipsoidParameters ep = results.ep;
            DA.SetData(i++, ep.minLongAxisLength);
            DA.SetData(i++, ep.maxLongAxisLength);
            DA.SetData(i++, ep.minSlenderness);

            AlgorithmConvergenceParameters acp = results.acp;
            DA.SetData(i++, acp.volumeFactor);
            DA.SetData(i++, acp.maxIterations);
            DA.SetData(i++, acp.updateScale);
            DA.SetData(i++, acp.parametersChanged);

            InputGeometryParameters igp = results.igp;
            DA.SetData(i++, igp.m);
            DA.SetData(i++, igp.s);
            DA.SetDataList(i++, igp.pts);
            DA.SetDataList(i++, igp.data);


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
                return MechanoAdaptiveGeneration.Properties.Resources.book;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("{8F9B8E65-8573-4625-BF54-BEBE9CA12722}"); }
        }
    }
}
