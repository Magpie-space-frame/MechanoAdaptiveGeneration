using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{
    public struct BackGroundData
    {
        public List<Point3d> G;
        public List<double> T0;
        public List<double> T1;
        public List<double> T2;
        public List<double> T3;
        public List<double> T4;
        public List<double> T5;

        public int xCount;
        public int yCount;
        public int zCount;
        public double xSize;
        public double ySize;
        public double zSize;
    }
}