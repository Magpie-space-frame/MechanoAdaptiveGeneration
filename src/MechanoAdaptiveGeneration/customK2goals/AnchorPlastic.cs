using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KangarooSolver;
using Rhino.Geometry;

namespace MechanoAdaptiveGeneration.customK2goals
{
    public class AnchorPlastic : GoalObject
    {
        public Point3d Location;
        public double Limit;

        public AnchorPlastic(int ix, Point3d P, double R, double K)
        {
            PIndex = new int[1] { ix };
            Move = new Vector3d[1];
            Weighting = new double[1] { K };
            Location = P;
            Limit = R;
        }

        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            Vector3d Between = Location - p[PIndex[0]].Position;
            Move[0] = Between;
            double stretch = Between.Length - Limit;
            if (stretch > 0)
            {
                Between.Unitize();
                Between *= stretch;
                Location -= Between;
                Move[0] -= Between;
            }
        }
    }
}
