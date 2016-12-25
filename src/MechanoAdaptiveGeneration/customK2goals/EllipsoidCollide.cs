using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration.customK2goals
{
    /// <summary>
    /// Class to handle a collision of two ellipsoids
    /// Outputs necessary data to use in a KangarooSolver collision goal (NonlinearRepel)
    /// </summary>
    public class EllipsoidCollide
    {
        Ellipsoid _e1;
        Ellipsoid _e2;

        public double Overlap;
        public double Extension;

        public EllipsoidCollide(Ellipsoid _e1, Ellipsoid _e2)
        {
            this._e1 = _e1;
            this._e2 = _e2;
        }

        public double CalculateCollision()
        {
            _e1.CalculateAxesAndScales();
            _e2.CalculateAxesAndScales();

            //calculate distance vectors back and forth, and the length of the distance
            //this will become part of the narrow phase check
            Vector3d pjk = new Vector3d(_e2.Position - _e1.Position);
            double pjkLength = pjk.Length;
            Vector3d pkj = new Vector3d(-pjk);

            //store direction before transforming distance vectors
            Vector3d pushVector = new Vector3d(pjk);

            //Transform connection vector onto "sphere" coordinates of either ellipsoid
            _e1.CalculateInverse();
            _e2.CalculateInverse();
            //find length of vector inside each ellipsoid
            double jDistance = _e1.FindDistanceInside(pjk);
            double kDistance = _e2.FindDistanceInside(pkj);

            double touchingDistance = jDistance + kDistance;
            if (pjkLength - (touchingDistance) < 0)
            {
                return touchingDistance;
            }
            else
            {
                return -1;
            }

            //      //longest overlap is:
            //      extension = pjkLength - (jDistance + kDistance);
            //
            //      //scalar w quantifying overlap in ]0,1]. w = 1 means no overlap, w close to 0 means a lot of overlap
            //      double overlap = pjkLength / (jDistance + kDistance);
            //
            //      if(overlap <= 1.0 && overlap > 0)
            //      {
            //        pushVector.Unitize();
            //        return extension * pushVector;
            //      } else
            //      {
            //        return Vector3d.Zero;
            //      }

        }
    }
}
