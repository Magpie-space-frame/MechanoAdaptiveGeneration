﻿using System;
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
        public double Ratio;
        public double Stiffness;

        public NonLinearRepel(int Pt0, int Pt1, double Len, double Rat, double k, int q)
        {

            PIndex = new int[2] { Pt0, Pt1 };
            Move = new Vector3d[2];
            Weighting = new double[2] { k, k };
            Exponent = q;
            Length = Len;
            Ratio = Rat;
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
                double SqrtRatio = Math.Sqrt(Ratio);
                Move[0] = (-0.15 * Current * Overlap)/SqrtRatio;
                Move[1] = (0.15 * Current * Overlap)*SqrtRatio;

                Weighting[0] = Weighting[1] = Stiffness * Math.Pow((Length - Overlap), Exponent);
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
