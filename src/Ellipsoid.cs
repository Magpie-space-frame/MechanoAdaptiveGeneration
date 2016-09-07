using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{
    /// <summary>
    /// bla bla bla
    /// </summary>
    public class Ellipsoid
    {

        //centre point of the ellipsoid
        public Point3d Position;
        //transform matrix that defines the scale and rotation make 3x3 (but really a 1x9)
        double[] _transform;
        //the inverse transform this is calculated once and then referenced..
        double[] _inverse;

        //unitized vectors for the axis
        //perhaps store the squared values as they are
        public Vector3d UnitXAxis;
        double _scaleX;
        public Vector3d UnitYAxis;
        double _scaleY;
        public Vector3d UnitZAxis;
        double _scaleZ;

        //points defining the bounding box
        public Point3d BoundingBoxMin;
        public Point3d BoundingBoxMax;

        //constructor
        public Ellipsoid(Point3d tPos)
        {
            //set the values use Vector3d.Axis and caluate bounding box function
            Position = tPos;

            //set the transfrom as an identity matrix
            _transform = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
            _inverse = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

            //could use calculateAxes but a waste of three unitize calls!
            UnitXAxis = Vector3d.XAxis;
            _scaleX = 1.0;
            UnitYAxis = Vector3d.YAxis;
            _scaleY = 1.0;
            UnitZAxis = Vector3d.ZAxis;
            _scaleZ = 1.0;

            CalculateBoundingBox();
        }

        //takes an set of vectors and updates the transform - taken from the eigenvectors of the interpolated field
        public void UpdateTransform(Vector3d xAxis, Vector3d yAxis, Vector3d zAxis)
        {
            _transform[0] = xAxis[0];
            _transform[3] = xAxis[1];
            _transform[6] = xAxis[2];

            _transform[1] = yAxis[0];
            _transform[4] = yAxis[1];
            _transform[7] = yAxis[2];

            _transform[2] = zAxis[0];
            _transform[5] = zAxis[1];
            _transform[8] = zAxis[2];
        }

        public void Scale(double s)
        {
            for (int i = 0; i < 9; i++)
            {
                _transform[i] = s * _transform[i];
            }
        }

        public void CalculateInverse()
        {
            _inverse[0] = _transform[0] / (_scaleX * _scaleX);
            _inverse[1] = _transform[3] / (_scaleX * _scaleX);
            _inverse[2] = _transform[6] / (_scaleX * _scaleX);

            _inverse[3] = _transform[1] / (_scaleY * _scaleY);
            _inverse[4] = _transform[4] / (_scaleY * _scaleY);
            _inverse[5] = _transform[7] / (_scaleY * _scaleY);

            _inverse[6] = _transform[2] / (_scaleZ * _scaleZ);
            _inverse[7] = _transform[5] / (_scaleZ * _scaleZ);
            _inverse[8] = _transform[8] / (_scaleZ * _scaleZ);
        }

        //calculate and return boundingBox
        public void CalculateBoundingBox()
        {
            //takes a transform matrix representation of an ellipsoid and returns two points defining the min x,y,z and max x,y,x
            double[] minAndMaxX = GetMinAndMax(Position.X, 0);
            double[] minAndMaxY = GetMinAndMax(Position.Y, 3);
            double[] minAndMaxZ = GetMinAndMax(Position.Z, 6);

            BoundingBoxMin = new Point3d(minAndMaxX[0], minAndMaxY[0], minAndMaxZ[0]);
            BoundingBoxMax = new Point3d(minAndMaxX[1], minAndMaxY[1], minAndMaxZ[1]);


        }

        //method used by the calculate bounding box method
        private double[] GetMinAndMax(double position, int adder)
        {
            double m11 = _transform[adder];
            double m12 = _transform[adder + 1];
            double m13 = _transform[adder + 2];
            double d = Math.Sqrt(m11 * m11 + m12 * m12 + m13 * m13);
            double[] minMax = { position - d, position + d };
            return minMax;
        }

        //set updated unit vectors for
        public void CalculateAxesAndScales()
        {
            UnitXAxis = new Vector3d(_transform[0], _transform[3], _transform[6]);
            _scaleX = UnitXAxis.Length;
            UnitXAxis.Unitize();
            UnitYAxis = new Vector3d(_transform[1], _transform[4], _transform[7]);
            _scaleY = UnitYAxis.Length;
            UnitYAxis.Unitize();
            UnitZAxis = new Vector3d(_transform[2], _transform[5], _transform[8]);
            _scaleZ = UnitZAxis.Length;
            UnitZAxis.Unitize();
        }

        //combines the postion and the transform to create an Affine transform
        public double[] CreateAffineTransform()
        {
            return new double[] { _transform[0], _transform[1], _transform[2], Position[0], _transform[3], _transform[4], _transform[5], Position[1], _transform[6], _transform[7], _transform[8], Position[2], 0, 0, 0, 1 };
        }

        //using similarity ratios: 1/transformedLength = distanceInside/inputlegth
        public double FindDistanceInside(Vector3d inputVector)
        {
            Vector3d transformed = TransformVector(inputVector);
            return inputVector.Length / transformed.Length;
        }

        //method used by findDistanceInside method
        private Vector3d TransformVector(Vector3d toTransform)
        {
            Vector3d transformedVector = new Vector3d();
            transformedVector[0] = _inverse[0] * toTransform[0] + _inverse[1] * toTransform[1] + _inverse[2] * toTransform[2];
            transformedVector[1] = _inverse[3] * toTransform[0] + _inverse[4] * toTransform[1] + _inverse[5] * toTransform[2];
            transformedVector[2] = _inverse[6] * toTransform[0] + _inverse[7] * toTransform[1] + _inverse[8] * toTransform[2];
            return transformedVector;
        }
    }
}
