using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KangarooSolver;
using Rhino.Geometry;

namespace MechanoAdaptiveGeneration.customK2goals
{
    public class NonLinearRepel : GoalObject
    {
        public int Exponent;
        public double Length;
        public double Multiplier;
        public double Stiffness;

        public NonLinearRepel(int Pt0, int Pt1, double Len, double Mult, double k, int q)
        {

            PIndex = new int[2] { Pt0, Pt1 };
            Move = new Vector3d[2];
            Weighting = new double[2] { k, k };
            Exponent = q;
            Length = Len;
            Multiplier = Mult;
            Stiffness = k;
        }

        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            Vector3d Current = p[PIndex[1]].Position - p[PIndex[0]].Position;

            double CurrentLength = Current.Length;
            double Overlap = Length - CurrentLength;
            Current.Unitize();

            if (Overlap > 0)
            {
                Move[0] = -0.15 * Current * Overlap;
                Move[1] = 0.15 * Current * Overlap;

                Weighting[0] = Weighting[1] =
                  //  Stiffness * (Math.Pow(CurrentLength * Multiplier, Exponent) - Math.Pow(Length * Multiplier, Exponent));
                  Stiffness * Math.Pow((Length - Overlap) * Multiplier, Exponent);
            }
            else
            {
                Weighting[0] = Weighting[1] = 0;
            }
        }

        public override object Output(List<KangarooSolver.Particle> p)
        {
            Line L = new Line(p[PIndex[0]].Position, p[PIndex[1]].Position);
            return L;
        }
    }
}
