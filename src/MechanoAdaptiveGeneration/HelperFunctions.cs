using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{
    /// <summary>
    /// A collection of useful functions for MAG
    /// </summary>
    public class HelperFunctions
    {
        /// <summary>
        /// Calculates the standard deviation
        /// </summary>
        /// <param name="values">list of input values</param>
        /// <returns></returns>
        public static double StdDev(ref List<double> values)
        {
            double ret = 0;
            int count = values.Count();
            if (count > 1)
            {
                //Compute the Average
                double avg = values.Average();

                //Perform the Sum of (value-avg)^2
                double sum = values.Sum(d => (d - avg) * (d - avg));

                //Put it all together
                ret = Math.Sqrt(sum / count);
            }
            return ret;
        }

        //evaluate % total volume taken up by ellipsoids, and correct the scaling factor if needed
        public static void UpdateScaleByVolume(ref double scale, ref double totalEllipsoidVolume, ref double meshVol)
        {
            double targetEllipsoidVolume = meshVol;
            double fullScale = scale * Math.Pow(targetEllipsoidVolume / totalEllipsoidVolume, 1.0 / 3.0);
            double currentScale = scale;
            scale = 0.5 * currentScale + 0.5 * fullScale;
        }

        public static void ProcessData(List<double> dataToProcess, ref BackGroundData backGroundData, ref StressTensor[,,] grid)
        {
            backGroundData.G = new List<Point3d>();
            backGroundData.T0 = new List<double>();
            backGroundData.T1 = new List<double>();
            backGroundData.T2 = new List<double>();
            backGroundData.T3 = new List<double>();
            backGroundData.T4 = new List<double>();
            backGroundData.T5 = new List<double>();

            while (dataToProcess.Any())
            {
                List<double> localDataList = dataToProcess.Take(9).ToList();

                backGroundData.G.Add(new Point3d(localDataList[0], localDataList[1], localDataList[2]));

                backGroundData.T0.Add(localDataList[3]);
                backGroundData.T1.Add(localDataList[4]);
                backGroundData.T2.Add(localDataList[5]);
                backGroundData.T3.Add(localDataList[6]);
                backGroundData.T4.Add(localDataList[7]);
                backGroundData.T5.Add(localDataList[8]);

                dataToProcess = dataToProcess.Skip(9).ToList();
            }

            //find grid count and size
            backGroundData.XCount = backGroundData.YCount = backGroundData.ZCount = 0;
            backGroundData.XSize = backGroundData.YSize = backGroundData.ZSize = 0;

            StressTensor[] tensors = new StressTensor[backGroundData.G.Count];
            for (int i = 0; i < backGroundData.G.Count; i++)
            {
                tensors[i] = new StressTensor(new List<double> { backGroundData.T0[i], backGroundData.T1[i], backGroundData.T2[i], backGroundData.T1[i], backGroundData.T3[i], backGroundData.T4[i], backGroundData.T2[i], backGroundData.T4[i], backGroundData.T5[i] });
            }

            Array.Sort(backGroundData.G.ToArray(), tensors);
            backGroundData.G.Sort();

            backGroundData.ZSize = backGroundData.G[1].Z - backGroundData.G[0].Z;

            for (int i = 1; i < backGroundData.G.Count; i++)
            {
                if (backGroundData.G[i].Y > backGroundData.G[0].Y)
                {
                    backGroundData.YSize = backGroundData.G[i].Y - backGroundData.G[0].Y;
                    backGroundData.ZCount = i;
                    break;
                }
            }

            for (int i = 1; i < backGroundData.G.Count; i++)
            {
                if (backGroundData.G[i].X > backGroundData.G[0].X)
                {
                    backGroundData.XSize = backGroundData.G[i].X - backGroundData.G[0].X;
                    backGroundData.YCount = i / backGroundData.ZCount;
                    break;
                }
            }

            backGroundData.XCount = backGroundData.G.Count / (backGroundData.ZCount * backGroundData.YCount);

            grid = new StressTensor[backGroundData.XCount, backGroundData.YCount, backGroundData.ZCount];

            for (int i = 0; i < backGroundData.XCount; i++)
            {
                for (int j = 0; j < backGroundData.YCount; j++)
                {
                    for (int k = 0; k < backGroundData.ZCount; k++)
                    {
                        grid[i, j, k] = tensors[k + j * backGroundData.ZCount + i * backGroundData.ZCount * backGroundData.YCount];
                    }
                }
            }
        }

        public static void SweepAndPrune(List<Ellipsoid> ellipsoids, Point3d[] positions, ref List<int> collideRef0, ref List<int> collideRef1)
        {
            List<Aabb> boxes = new List<Aabb>();

            //use the points to create AABB objects
            for (int i = 0; i < ellipsoids.Count; i++)
            {
                ellipsoids[i].Position = positions[i];
                ellipsoids[i].CalculateBoundingBox();

                boxes.Add(new Aabb(ellipsoids[i].BoundingBoxMin, ellipsoids[i].BoundingBoxMax, i));
            }

            List<EndPoint> sortedInX = new List<EndPoint>();

            //add the endpoints in x
            foreach (Aabb boxForPoint in boxes)
            {
                sortedInX.Add(boxForPoint.Min[0]);//change this 0 to 1 or 2 to sort in Y or Z - will also need to change the second half of the test function
                sortedInX.Add(boxForPoint.Max[0]);//change this 0 to 1 or 2 to sort in Y or Z
            }

            //sort by the num value of each EndPoint
            sortedInX.Sort(EndPoint.CompareEndPoints);

            List<int> openBoxes = new List<int>();

            foreach (EndPoint endPoint in sortedInX)
            {
                if (endPoint.IsMin)
                {
                    Aabb thisPointOwner = endPoint.Owner;
                    //check against all in openBoxes
                    foreach (int openBoxRef in openBoxes)
                    {
                        //if it collides output the integers of the branches than collide
                        //do they collide in y?
                        if (thisPointOwner.Max[1].Num > boxes[openBoxRef].Min[1].Num && thisPointOwner.Min[1].Num < boxes[openBoxRef].Max[1].Num)
                        {
                            //they collide in y, do they collide in z?
                            if (thisPointOwner.Max[2].Num > boxes[openBoxRef].Min[2].Num && thisPointOwner.Min[2].Num < boxes[openBoxRef].Max[2].Num)
                            {
                                //they collide in z
                                //therefore they collide! Add to the list of collide refs
                                collideRef0.Add(endPoint.Owner.BranchRef);
                                collideRef1.Add(boxes[openBoxRef].BranchRef);
                            }
                        }
                    }
                    //add corresponding box to openBoxes
                    openBoxes.Add(thisPointOwner.BranchRef);

                }
                else
                {
                    //it must be an max point
                    //remove corresponding box from openBoxes
                    openBoxes.Remove(endPoint.Owner.BranchRef);
                }
            }
        }

        public static void InterpolateTensor(Point3d[] pts, ref List<Vector3d> evec1, ref List<Vector3d> evec2, ref List<Vector3d> evec3, ref List<double> eval1, ref List<double> eval2, ref List<double> eval3, StressTensor[,,] grid, BackGroundData backGroundData)
        {
            int nOfPoints = pts.Count();

            var eigenVectors = new Vector3d[nOfPoints][];
            var eigenValues = new Double[nOfPoints][];

            var eva = new Vector3d[nOfPoints];
            var evb = new Vector3d[nOfPoints];
            var evc = new Vector3d[nOfPoints];
            var eigenValuesA = new double[nOfPoints];
            var eigenValuesB = new double[nOfPoints];
            var eigenValuesC = new double[nOfPoints];

            // for each point in P, find its coordinates in the grid
            // output the 8 points/indices of the corners
            // and the point coordinates of the position within that cell

            var arrayOfCellCoords = new Point3d[nOfPoints]; // always in [0,1]^3
            var arrayOfTensors0 = new double[nOfPoints][];
            var arrayOfTensors1 = new double[nOfPoints][];
            var arrayOfTensors2 = new double[nOfPoints][];
            var arrayOfTensors3 = new double[nOfPoints][];
            var arrayOfTensors4 = new double[nOfPoints][];
            var arrayOfTensors5 = new double[nOfPoints][];
            var arrayOfTensors6 = new double[nOfPoints][];
            var arrayOfTensors7 = new double[nOfPoints][];

            System.Threading.Tasks.Parallel.For(0, nOfPoints,
              j =>
              {
                  var pVec = pts[j] - backGroundData.G[0];  //position vector of the current point relative to grid origin

                  var px = pVec.X / backGroundData.XSize;
                  var py = pVec.Y / backGroundData.YSize;
                  var pz = pVec.Z / backGroundData.ZSize;

                  int pxf, pxc, pyf, pyc, pzf, pzc;

                  var coordsInCell = new Point3d();

                  if (px < 0)
                  {
                      pxf = 0; pxc = 1; coordsInCell.X = 0;
                  }
                  else if (px > backGroundData.XCount - 1)
                  {
                      pxf = backGroundData.XCount - 2; pxc = backGroundData.XCount - 1; coordsInCell.X = 1;
                  }
                  else
                  {
                      pxf = (int)Math.Floor(px); pxc = (int)Math.Ceiling(px); coordsInCell.X = px - pxf;
                  }


                  if (py < 0)
                  {
                      pyf = 0; pyc = 1; coordsInCell.Y = 0;
                  }
                  else if (py > backGroundData.YCount - 1)
                  {
                      pyf = backGroundData.YCount - 2; pyc = backGroundData.YCount - 1; coordsInCell.Y = 1;
                  }
                  else
                  {
                      pyf = (int)Math.Floor(py); pyc = (int)Math.Ceiling(py); coordsInCell.Y = py - pyf;
                  }


                  if (pz < 0)
                  {
                      pzf = 0; pzc = 1; coordsInCell.Z = 0;
                  }
                  else if (pz > backGroundData.ZCount - 1)
                  {
                      pzf = backGroundData.ZCount - 2; pzc = backGroundData.ZCount - 1; coordsInCell.Z = 1;
                  }
                  else
                  {
                      pzf = (int)Math.Floor(pz); pzc = (int)Math.Ceiling(pz); coordsInCell.Z = pz - pzf;
                  }


                  //
                  //      int PXF = Math.Max(Math.Min((int) Math.Floor(PX), xCount - 1), 0);
                  //      int PYF = Math.Max(Math.Min((int) Math.Floor(PY), yCount - 1), 0);
                  //      int PZF = Math.Max(Math.Min((int) Math.Floor(PZ), zCount - 1), 0);
                  //
                  //      int PXC = Math.Max(Math.Min((int) Math.Ceiling(PX), xCount - 1), 0);
                  //      int PYC = Math.Max(Math.Min((int) Math.Ceiling(PY), yCount - 1), 0);
                  //      int PZC = Math.Max(Math.Min((int) Math.Ceiling(PZ), zCount - 1), 0);

                  //     arrayOfCellCoords[j] = new Point3d(Math.Min(Math.Max(PX - PXF, 0), 1), Math.Min(Math.Max(PY - PYF, 0), 1), Math.Min(Math.Max(PZ - PZF, 0), 1));

                  arrayOfCellCoords[j] = coordsInCell;

                  arrayOfTensors0[j] = grid[pxf, pyf, pzf].Values;
                  arrayOfTensors1[j] = grid[pxf, pyf, pzc].Values;
                  arrayOfTensors2[j] = grid[pxf, pyc, pzf].Values;
                  arrayOfTensors3[j] = grid[pxf, pyc, pzc].Values;
                  arrayOfTensors4[j] = grid[pxc, pyf, pzf].Values;
                  arrayOfTensors5[j] = grid[pxc, pyf, pzc].Values;
                  arrayOfTensors6[j] = grid[pxc, pyc, pzf].Values;
                  arrayOfTensors7[j] = grid[pxc, pyc, pzc].Values;

                  var corners = new StressTensor[8];

                  corners[0] = new StressTensor();
                  corners[1] = new StressTensor();
                  corners[2] = new StressTensor();
                  corners[3] = new StressTensor();
                  corners[4] = new StressTensor();
                  corners[5] = new StressTensor();
                  corners[6] = new StressTensor();
                  corners[7] = new StressTensor();

                  corners[0].Values = (double[])arrayOfTensors0[j];
                  corners[1].Values = (double[])arrayOfTensors1[j];
                  corners[2].Values = (double[])arrayOfTensors2[j];
                  corners[3].Values = (double[])arrayOfTensors3[j];
                  corners[4].Values = (double[])arrayOfTensors4[j];
                  corners[5].Values = (double[])arrayOfTensors5[j];
                  corners[6].Values = (double[])arrayOfTensors6[j];
                  corners[7].Values = (double[])arrayOfTensors7[j];

                  var eVec = new Vector3d[3];
                  var eVal = new Double[3];

                  var T = new Double[3] { arrayOfCellCoords[j].X, arrayOfCellCoords[j].Y, arrayOfCellCoords[j].Z };
                  //var T = new Double[3]{0.5,0.5,0.5};

                  //THIS IS WHERE THE ACTUAL EIGENVALUE PROBLEM IS SOLVED
                  StressTensor tensorAtPoint = new StressTensor();
                  bool isValid = TriLinearInterpolate(ref tensorAtPoint, corners, T);
                  if (isValid)
                  {
                      EigenSolve(tensorAtPoint.Values, out eVal, out eVec);

                      int[] ott = new int[3]; // o ne, t wo, t hree
                      ott[0] = 0;
                      ott[1] = 1;
                      ott[2] = 2;
                      double[] ev = new double[] { Math.Abs(eVal[0]), Math.Abs(eVal[1]), Math.Abs(eVal[2]) };
                      //sort according to maximum absolute value of Eval
                      Array.Sort(ev, ott);

                      eva[j] = eVec[ott[2]];
                      evb[j] = eVec[ott[1]];
                      evc[j] = eVec[ott[0]];

                      eigenValuesA[j] = Math.Abs(eVal[ott[2]]);
                      eigenValuesB[j] = Math.Abs(eVal[ott[1]]);
                      eigenValuesC[j] = Math.Abs(eVal[ott[0]]);
                  }
              });

            evec1 = eva.ToList();
            evec2 = evb.ToList();
            evec3 = evc.ToList();
            eval1 = eigenValuesA.ToList();
            eval2 = eigenValuesB.ToList();
            eval3 = eigenValuesC.ToList();
        }

        /// <summary>
        /// given stress tensors at the 8 corners of a cube, and coordinates of a point within that cube
        /// perform a trilinear interpolation
        /// </summary>
        /// <param name="tensorAtPoint">resulting tensor from interpolation</param>
        /// <param name="cornerTensors">Tensor values at the corners, ordered 000,001,010,011,100,101,110,111</param>
        /// <param name="T">3 numbers in the range 0 to 1</param>
        /// <returns>the interpolated stress tensor at the given coordinates</returns>
        ///
        public static bool TriLinearInterpolate(ref StressTensor tensorAtPoint, StressTensor[] cornerTensors, double[] T)
        {
            //first get the 4 values along the edges in the x direction
            StressTensor sx0 = LinearInterpolate(cornerTensors[0], cornerTensors[4], T[0]);
            StressTensor sx1 = LinearInterpolate(cornerTensors[1], cornerTensors[5], T[0]);
            StressTensor sx2 = LinearInterpolate(cornerTensors[2], cornerTensors[6], T[0]);
            StressTensor sx3 = LinearInterpolate(cornerTensors[3], cornerTensors[7], T[0]);

            //interpolate along Y
            StressTensor sy0 = LinearInterpolate(sx0, sx2, T[1]);
            StressTensor sy1 = LinearInterpolate(sx1, sx3, T[1]);

            //and finally along Z
            var st = LinearInterpolate(sy0, sy1, T[2]);
            bool hasNaN = false;
            for (int i = 0; i < 9; i++)
            {
                if (Double.IsNaN(st.Values[i])) { hasNaN = true; }
            }
            if (!hasNaN)
            {
                tensorAtPoint = st;
                return true;
            }
            else
            {
                return false;
            }

        }

        public static StressTensor LinearInterpolate(StressTensor tensorA, StressTensor tensorB, double T)
        {
            var interp = new StressTensor();
            for (int i = 0; i < 9; i++)
            {
                int naNorZeroCount = 0;

                interp.Values[i] = 0;
                if (!Double.IsNaN(tensorA.Values[i]))
                {
                    interp.Values[i] += (1 - T) * tensorA.Values[i];
                }
                else
                {
                    interp.Values[i] += (1 - T) * tensorB.Values[i];
                    naNorZeroCount++;
                }
                if (!Double.IsNaN(tensorB.Values[i]))
                {
                    interp.Values[i] += T * tensorB.Values[i];
                }
                else
                {
                    interp.Values[i] += T * tensorA.Values[i];
                    naNorZeroCount++;
                }

                if (naNorZeroCount == 2)
                {
                    interp.Values[i] = double.NaN;
                }
            }

            return interp;
        }

        public static void EigenSolve(double[] x, out double[] eigenValues, out Vector3d[] eigenVectors)
        {
            double[,] matValues = new double[3, 3] { { x[0], x[1], x[2] }, { x[3], x[4], x[5] }, { x[6], x[7], x[8] } };
            double[,] outMatValues = new double[3, 3];
            double[] outVecValues = new double[3];
            Eigen eigenCalculate = new Eigen();

            eigenCalculate.eigen_decomposition(matValues, outMatValues, outVecValues);

            var eigenVectorsUnsorted = new Vector3d[3];

            if (eigenCalculate.CalcDone)
            {
                eigenVectorsUnsorted[0] = new Vector3d(outMatValues[0, 0], outMatValues[1, 0], outMatValues[2, 0]);
                eigenVectorsUnsorted[1] = new Vector3d(outMatValues[0, 1], outMatValues[1, 1], outMatValues[2, 1]);
                eigenVectorsUnsorted[2] = new Vector3d(outMatValues[0, 2], outMatValues[1, 2], outMatValues[2, 2]);
            }

            Array.Sort(outVecValues, eigenVectorsUnsorted, new DescendingAbsoluteVals());
            eigenValues = outVecValues;
            eigenVectors = eigenVectorsUnsorted;
        }

        public class DescendingAbsoluteVals : Comparer<double>
        {
            public override int Compare(double x, double y)
            {
                return (y * y).CompareTo(x * x);
            }
        }
    }
}
