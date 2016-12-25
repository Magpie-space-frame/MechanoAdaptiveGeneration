using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{
    /// <summary>
    /// Data structure for the grid input data
    /// </summary>
    public struct BackGroundData
    {
        public List<Point3d> G;
        public List<double> T0;
        public List<double> T1;
        public List<double> T2;
        public List<double> T3;
        public List<double> T4;
        public List<double> T5;

        public int XCount;
        public int YCount;
        public int ZCount;
        public double XSize;
        public double YSize;
        public double ZSize;
    }
}