using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KangarooSolver;

namespace MechanoAdaptiveGeneration
{

    public class Align : GoalObject
    {
        public double Strength;
        public List<Vector3d> Vec;

        public Align(int ea, int eb, Vector3d[] _Vec, double k)
        {
            PIndex = new int[2] { ea, eb };
            Move = new Vector3d[2];
            Weighting = new double[2] { k, k };
            Vec = _Vec.ToList();
            Strength = k;
        }

        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            Point3d p0 = p[PIndex[0]].Position;
            Point3d p1 = p[PIndex[1]].Position;
            var EdgeVec = new Vector3d();
            EdgeVec = p1 - p0;

            double AngleX = Vector3d.VectorAngle(Vec[0], EdgeVec);
            double AngleY = Vector3d.VectorAngle(Vec[1], EdgeVec);
            double AngleZ = Vector3d.VectorAngle(Vec[2], EdgeVec);
            double AngleNX = Vector3d.VectorAngle(-Vec[0], EdgeVec);
            double AngleNY = Vector3d.VectorAngle(-Vec[1], EdgeVec);
            double AngleNZ = Vector3d.VectorAngle(-Vec[2], EdgeVec);

            double lowest = AngleX;
            Vector3d TargetDir = Vec[0];

            if (AngleY < lowest)
            {
                TargetDir = Vec[1];
                lowest = AngleY;
            }

            if (AngleZ < lowest)
            {
                TargetDir = Vec[2];
                lowest = AngleZ;
            }

            if (AngleNX < lowest)
            {
                TargetDir = -Vec[0];
                lowest = AngleNX;
            }
            if (AngleNY < lowest)
            {
                TargetDir = -Vec[1];
                lowest = AngleNY;
            }
            if (AngleNZ < lowest)
            {
                TargetDir = -Vec[2];
            }

            Vector3d V = EdgeVec;
            Vector3d Dir = TargetDir;

            Vector3d To = (V - (V * Dir) * Dir) * 0.25;

            Move[0] = To;
            Move[1] = -To;

            double cutoffAngle = Math.PI / 9;

            if (!Dir.IsZero && lowest < cutoffAngle)
            {
                double Val1 = (cutoffAngle - lowest) / cutoffAngle; //gives a value between 0 and 1, increasing as it gets closer to full alignment
                double Val2 = Val1 * Strength;
                Weighting[0] = Val2;
                Weighting[1] = Val2;
            }
            else
            {
                Weighting[0] = 0;
                Weighting[1] = 0;
            }
        }
    }
}
