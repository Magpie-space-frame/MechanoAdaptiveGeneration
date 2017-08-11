using System;
using System.IO;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using KangarooSolver;
using System.Linq;

namespace MechanoAdaptiveGeneration
{
    public class MagpieReader
    {
        public MagpieReader()
        {
        }

        public MagpieResults read(string inputPath)
        {
            MagpieResults results = new MagpieResults();
            string[] rawInput = File.ReadAllLines(inputPath);
            readEllipsoidPacking(rawInput, results);
            readInputParameters(rawInput, results);
            return results;
        }

        private void readEllipsoidPacking(string[] rawInput, MagpieResults results)
        {
            List<Point3d> centres = new List<Point3d>();
            List<Vector3d> longAxes = new List<Vector3d>();
            List<Vector3d> shortAxes = new List<Vector3d>();

            var centresStartIndex = Array.FindIndex(rawInput, row => row == "Centres");
            var longAxesStartIndex = Array.FindIndex(rawInput, row => row == "Long Axes");
            var shortAxesStartIndex = Array.FindIndex(rawInput, row => row == "Short Axes");

            int nEllipsoids = longAxesStartIndex - centresStartIndex - 1;
            for (int i = centresStartIndex + 1; i <= nEllipsoids; i++)
            {
                string centresLine = rawInput[i];
                string longAxesLine = rawInput[i + longAxesStartIndex];
                string shortAxesLine = rawInput[i + shortAxesStartIndex];

                string[] splitCentresLine = centresLine.Split(',');
                string[] splitLongAxesLine = longAxesLine.Split(',');
                string[] splitShortAxesLine = shortAxesLine.Split(',');

                centres.Add(new Point3d(Convert.ToDouble(splitCentresLine[0]), Convert.ToDouble(splitCentresLine[1]), Convert.ToDouble(splitCentresLine[2])));
                longAxes.Add(new Vector3d(Convert.ToDouble(splitLongAxesLine[0]), Convert.ToDouble(splitLongAxesLine[1]), Convert.ToDouble(splitLongAxesLine[2])));
                shortAxes.Add(new Vector3d(Convert.ToDouble(splitShortAxesLine[0]), Convert.ToDouble(splitShortAxesLine[1]), Convert.ToDouble(splitShortAxesLine[2])));
            }
            EllipsoidPacking ellipsoidPacking = new EllipsoidPacking(centres, longAxes, shortAxes);
            results.packing = ellipsoidPacking;
        }



        private void readInputParameters(string[] rawInput, MagpieResults results)
        {
            readKGP(rawInput, results);
            readEGP(rawInput, results);
            readACP(rawInput, results);
            readIGP(rawInput, results);
        }


        private void readKGP(string[] rawInput, MagpieResults results)
        {
            var kangarooGoalParamsIndex = Array.FindIndex(rawInput, row => row == "Kangaroo Goal Parameters");
            //assert index >=0! and <rawinput.length
            double dragDistance = Convert.ToDouble(rawInput[kangarooGoalParamsIndex + 1]);
            double boundaryCollideStrength = Convert.ToDouble(rawInput[kangarooGoalParamsIndex + 2]);
            double alignStrength = Convert.ToDouble(rawInput[kangarooGoalParamsIndex + 3]);

            string fixedPointIndicesLine = rawInput[kangarooGoalParamsIndex + 4];
            string[] splitFixedPointIndicesLine = fixedPointIndicesLine.Split(',');
            List<int> fixedPointIndices = new List<int>();
            for (int i = 0; i < splitFixedPointIndicesLine.Length; i++)
            {
                fixedPointIndices.Add(Convert.ToInt16(splitFixedPointIndicesLine[i]));
            }
            KangarooGoalParameters kgp = new KangarooGoalParameters(dragDistance, boundaryCollideStrength, alignStrength, fixedPointIndices);
            results.kgp = kgp;
        }

        private void readEGP(string[] rawInput, MagpieResults results)
        {
            var ellipsoidParamsIndex = Array.FindIndex(rawInput, row => row == "Ellipsoid Parameters");
            //assert index >=0! and <rawinput.length
            double minLongAxisLength = Convert.ToDouble(rawInput[ellipsoidParamsIndex + 1]);
            double maxLongAxisLength = Convert.ToDouble(rawInput[ellipsoidParamsIndex + 2]);
            double minSlenderness = Convert.ToDouble(rawInput[ellipsoidParamsIndex + 3]);
            EllipsoidParameters ep = new EllipsoidParameters(minLongAxisLength, maxLongAxisLength, minSlenderness);
            results.ep = ep;
        }

        private void readACP(string[] rawInput, MagpieResults results)
        {
            var algorithmConvergenceParamsIndex = Array.FindIndex(rawInput, row => row == "Algorithm Convergence Parameters");
            //assert index >=0! and <rawinput.length
            double volumeFactor = Convert.ToDouble(rawInput[algorithmConvergenceParamsIndex + 1]);
            int maxIterations = Convert.ToInt16(rawInput[algorithmConvergenceParamsIndex + 2]);
            int iterations = Convert.ToInt16(rawInput[algorithmConvergenceParamsIndex + 3]);
            bool updateScale = rawInput[algorithmConvergenceParamsIndex + 4].Equals("false") ? false : true;
            bool parametersChanged = rawInput[algorithmConvergenceParamsIndex + 5].Equals("false") ? false : true;
            bool converged = rawInput[algorithmConvergenceParamsIndex + 6].Equals("false") ? false : true;

            AlgorithmConvergenceParameters acp = new AlgorithmConvergenceParameters(volumeFactor, maxIterations, updateScale);
            results.acp = acp;
            //some additional diagnostic information
            results.paramsUnchangedDuringPacking = !parametersChanged;
            results.converged = converged;
            results.iterations = iterations;
        }

        private void readIGP(string[] rawInput, MagpieResults results)
        {
            var inputGeometryParamsIndex = Array.FindIndex(rawInput, row => row == "Input Geometry Parameters");
            //assert index >=0! and <rawinput.length
            //assert igp[0] = *.3dm
            //assert igp[1] = *.3dm if exists
            string pathToInputMesh = rawInput[inputGeometryParamsIndex + 1];
            string pathToSurfaceMesh = rawInput[inputGeometryParamsIndex + 2];

            var inputPointsIndex = Array.FindIndex(rawInput, row => row == "Input Points");
            var endInputPointsIndex = Array.FindIndex(rawInput, row => row == "End Input Points");

            List<Point3d> inputPoints = new List<Point3d>();
            for (int i = inputPointsIndex + 1; i < endInputPointsIndex; i++)
            {
                string inputPointsLine = rawInput[i];
                string[] splitInputPointsLine = inputPointsLine.Split(',');
                inputPoints.Add(new Point3d(Convert.ToDouble(splitInputPointsLine[0]), Convert.ToDouble(splitInputPointsLine[1]), Convert.ToDouble(splitInputPointsLine[2])));
            }

            var inputFieldIndex = Array.FindIndex(rawInput, row => row == "Input Field");
            var endInputFieldIndex = Array.FindIndex(rawInput, row => row == "End Input Field");
            List<Double> inputField = new List<Double>();
            for (int i = inputFieldIndex + 1; i < endInputFieldIndex; i++)
            {
                string inputFieldLine = rawInput[i];
                string[] splitInputFieldLine = inputFieldLine.Split(',');
                for (int j = 0; j < 9; j++)
                {
                    inputField.Add(Convert.ToDouble(splitInputFieldLine[j]));
                }
            }

            var inputVolumeIndex = Array.FindIndex(rawInput, row => row == "Input Mesh");
            string inputMeshAsString = rawInput[inputVolumeIndex + 1];
            Mesh inputMesh = (Mesh)GH_Convert.ByteArrayToCommonObject<GeometryBase>(System.Convert.FromBase64String(inputMeshAsString));

            var inputSurfaceIndex = Array.FindIndex(rawInput, row => row == "Surface Mesh");
            Mesh surfaceMesh = new Mesh();
            string inputSurfaceAsString = rawInput[inputSurfaceIndex + 1];
            if(inputSurfaceAsString != "") surfaceMesh = (Mesh)GH_Convert.ByteArrayToCommonObject<GeometryBase>(System.Convert.FromBase64String(inputSurfaceAsString));

            InputGeometryParameters igp = new InputGeometryParameters(inputMesh, surfaceMesh, inputPoints, inputField);
            results.igp = igp;
        }
    }

}



