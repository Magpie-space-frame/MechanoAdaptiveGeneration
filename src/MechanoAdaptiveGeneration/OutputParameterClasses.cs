using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using KangarooSolver;
using System.Linq;

namespace MechanoAdaptiveGeneration
{
    /// <summary>
    /// A class to store the results of a Magpie space frame generation coherently with its input parameters, to be able to reproduce experiments
    /// </summary>
    public class MagpieResults
    {
        public InputGeometryParameters igp;
        public KangarooGoalParameters kgp;
        public AlgorithmConvergenceParameters acp;
        public EllipsoidParameters ep;

        public EllipsoidPacking packing;
        public Boolean paramsUnchangedDuringPacking;
        public Boolean converged;
        public int iterations;

        public string magpieVersion;

        public override string ToString()
        {
            string kangarooGoalParameterString = "Kangaroo Goal Parameters\n";
            kangarooGoalParameterString += this.kgp.plasticDragDistance.ToString() + "\n";
            kangarooGoalParameterString += this.kgp.boundaryCollideStrength.ToString() + "\n";
            kangarooGoalParameterString += this.kgp.alignStrength.ToString() + "\n";
            for (int i = 0; i < this.kgp.fixedPointIndices.Count-1; i++)
            {
                kangarooGoalParameterString += this.kgp.fixedPointIndices[i].ToString() + ',';
            }
            kangarooGoalParameterString += this.kgp.fixedPointIndices[this.kgp.fixedPointIndices.Count-1]+"\n";

            string ellipsoidParameterString = "Ellipsoid Parameters\n";
            ellipsoidParameterString += this.ep.minLongAxisLength.ToString() + "\n";
            ellipsoidParameterString += this.ep.maxLongAxisLength.ToString() + "\n";
            ellipsoidParameterString += this.ep.minSlenderness.ToString() + "\n"; ;

            string algorithmConvergenceParameterString = "Algorithm Convergence Parameters\n";
            algorithmConvergenceParameterString += this.acp.volumeFactor.ToString() + "\n";
            algorithmConvergenceParameterString += this.acp.maxIterations.ToString() + "\n";
            algorithmConvergenceParameterString += this.acp.iterations.ToString() + "\n";
            algorithmConvergenceParameterString += this.acp.updateScale.ToString().ToLower() + "\n";
            algorithmConvergenceParameterString += this.acp.parametersChanged.ToString().ToLower() + "\n";
            algorithmConvergenceParameterString += this.acp.isConverged.ToString().ToLower() + "\n";

            string inputGeometryParameterString = "Input Geometry Parameters\n";

            byte[] inputMeshData = Grasshopper.Kernel.GH_Convert.CommonObjectToByteArray(igp.m);
            string inputMeshAsString = System.Convert.ToBase64String(inputMeshData);
            inputMeshAsString = "Input Mesh\n" + inputMeshAsString + "\n";

            string surfaceMeshAsString = "";
            if (igp.s != null)
            {
                byte[] surfaceMeshData = Grasshopper.Kernel.GH_Convert.CommonObjectToByteArray(igp.s);
                surfaceMeshAsString = System.Convert.ToBase64String(surfaceMeshData);
            }
            surfaceMeshAsString = "Surface Mesh\n" + surfaceMeshAsString + "\n";

            inputGeometryParameterString += inputMeshAsString;
            inputGeometryParameterString += surfaceMeshAsString;
            inputGeometryParameterString += "Input Points\n";
            for (int i = 0; i < this.igp.pts.Count; i++)
            {
                inputGeometryParameterString += this.igp.pts[i].X.ToString() + ',' + this.igp.pts[i].Y.ToString() + ',' + this.igp.pts[i].Z.ToString() + "\n";
            }
            inputGeometryParameterString += "End Input Points\n";
            inputGeometryParameterString += "Input Field\n";
            for (int i = 0; i < this.igp.data.Count; i += 9)
            {
                for (int j = 0; j < 8; j++)
                {
                    inputGeometryParameterString += this.igp.data[i+j].ToString()+',';
                }
                inputGeometryParameterString += this.igp.data[8]+"\n";
            }
            inputGeometryParameterString += "End Input Field\n";

            return this.packing.ToString() + kangarooGoalParameterString + ellipsoidParameterString + algorithmConvergenceParameterString + inputGeometryParameterString;
        }
    }

    public class EllipsoidPacking
    {
        List<Point3d> centres;
        List<Vector3d> longAxes;
        List<Vector3d> shortAxes;

        public EllipsoidPacking(List<Point3d> c, List<Vector3d> la, List<Vector3d> sa)
        {
            this.centres = c;
            this.longAxes = la;
            this.shortAxes = sa;
        }

        /// <summary>
        /// Overrides object.ToString for EllipsoidPacking class
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            int N = this.centres.Count();
            //Debug.Assert(N == this.longAxes.Count(), "Long axes and centres are not of same length");
            //Debug.Assert(N == this.shortAxes.Count(), "Short axes and centres are not of same length");

            string centresString = "Centres\n";
            string longAxesString = "Long Axes\n";
            string shortAxesString = "Short Axes\n";
            for (int i = 0; i < N; i++)
            {
                centresString += (centres[i].ToString() + "\n");
                longAxesString += (longAxes[i].ToString() + "\n");
                shortAxesString += (shortAxes[i].ToString() + "\n");
            }
            string outputString = centresString + longAxesString + shortAxesString;
            return outputString;
        }

        public List<Point3d> GetCentres()
        {
            return this.centres;
        }

        public List<Vector3d> GetLongAxes()
        {
            return this.longAxes;
        }
        public List<Vector3d> GetShortAxes()
        {
            return this.shortAxes;
        }
    }
}
