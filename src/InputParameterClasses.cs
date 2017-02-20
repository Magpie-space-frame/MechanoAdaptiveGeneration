using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{
    public class InputGeometryParameters
    {
        public Mesh m;
        public Mesh s;
        public List<Point3d> pts;
        public List<double> data;

        public InputGeometryParameters(Mesh m, Mesh s, List<Point3d> pts, List<double> data)
        {
            this.m = m;
            this.s = s;
            this.pts = pts;
            this.data = data;
        }
    }

    public class KangarooGoalParameters
    {
        public double alignStrength;
        public double boundaryCollideStrength;
        public double plasticDragDistance;
        public List<int> fixedPointIndices;

        public KangarooGoalParameters(double plasticDragDistance, double boundaryCollideStrength, double alignStrength, List<int> fixedPointIndices)
        {
            this.plasticDragDistance = plasticDragDistance;
            this.boundaryCollideStrength = boundaryCollideStrength;
            this.alignStrength = alignStrength;
            this.fixedPointIndices = fixedPointIndices;
        }
    }

    public class EllipsoidParameters
    {
        public double minLongAxisLength;
        public double maxLongAxisLength;
        public double minSlenderness;

        public EllipsoidParameters(double minLongAxisLength, double maxLongAxisLength, double minSlenderness)
        {
            this.minLongAxisLength = minLongAxisLength;
            this.maxLongAxisLength = maxLongAxisLength;
            this.minSlenderness = minSlenderness;
        }
    }

    public class AlgorithmConvergenceParameters
    {
        public int maxIterations;
        public double volumeFactor;
        public bool updateScale;

        public AlgorithmConvergenceParameters(double volumeFactor, int maxIterations, bool updateScale)
        {
            this.volumeFactor = volumeFactor;
            this.maxIterations = maxIterations;
            this.updateScale = updateScale;
        }
    }
}
