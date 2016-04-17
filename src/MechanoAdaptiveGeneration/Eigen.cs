using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{
    public class Eigen
    {
        public bool startCalc = false;
        public bool calcDone = false;

        ///////////////////////////////
        //Eigen solver implementation
        ///////////////////////////////
        private static double hypot2(double x, double y)
        {
            return Math.Sqrt(x * x + y * y);
        }

        private static void tred2(double[,] V, double[] d, double[] e)
        {
            for (int j = 0; j < 3; j++)
            {
                d[j] = V[3 - 1, j];
            }

            for (int i = 3 - 1; i > 0; i--)
            {
                double scale = 0.0;
                double h = 0.0;

                for (int k = 0; k < i; k++)
                {
                    scale = scale + Math.Abs(d[k]);
                }

                if (scale == 0.0)
                {
                    e[i] = d[i - 1];
                    for (int j = 0; j < i; j++)
                    {
                        d[j] = V[i - 1, j];
                        V[i, j] = 0.0;
                        V[j, i] = 0.0;
                    }
                }
                else
                {
                    for (int k = 0; k < i; k++)
                    {
                        d[k] /= scale;
                        h += d[k] * d[k];
                    }

                    double f = d[i - 1];
                    double g = Math.Sqrt(h);

                    if (f > 0)
                    {
                        g = -g;
                    }

                    e[i] = scale * g;
                    h = h - f * g;
                    d[i - 1] = f - g;

                    for (int j = 0; j < i; j++)
                    {
                        e[j] = 0.0;
                    }

                    for (int j = 0; j < i; j++)
                    {
                        f = d[j];
                        V[j, i] = f;
                        g = e[j] + V[j, j] * f;

                        for (int k = j + 1; k <= i - 1; k++)
                        {
                            g += V[k, j] * d[k];
                            e[k] += V[k, j] * f;
                        }

                        e[j] = g;
                    }

                    f = 0.0;

                    for (int j = 0; j < i; j++)
                    {
                        e[j] /= h;
                        f += e[j] * d[j];
                    }

                    double hh = f / (h + h);

                    for (int j = 0; j < i; j++)
                    {
                        e[j] -= hh * d[j];
                    }

                    for (int j = 0; j < i; j++)
                    {
                        f = d[j];
                        g = e[j];

                        for (int k = j; k <= i - 1; k++)
                        {
                            V[k, j] -= (f * e[k] + g * d[k]);
                        }

                        d[j] = V[i - 1, j];
                        V[i, j] = 0.0;
                    }
                }
                d[i] = h;
            }

            for (int i = 0; i < 3 - 1; i++)
            {
                V[3 - 1, i] = V[i, i];
                V[i, i] = 1.0;

                double h = d[i + 1];

                if (h != 0.0)
                {
                    for (int k = 0; k <= i; k++)
                    {
                        d[k] = V[k, i + 1] / h;
                    }
                    for (int j = 0; j <= i; j++)
                    {
                        double g = 0.0;

                        for (int k = 0; k <= i; k++)
                        {
                            g += V[k, i + 1] * V[k, j];
                        }
                        for (int k = 0; k <= i; k++)
                        {
                            V[k, j] -= g * d[k];
                        }
                    }
                }
                for (int k = 0; k <= i; k++)
                {
                    V[k, i + 1] = 0.0;
                }
            }
            for (int j = 0; j < 3; j++)
            {
                d[j] = V[3 - 1, j];
                V[3 - 1, j] = 0.0;
            }

            V[3 - 1, 3 - 1] = 1.0;
            e[0] = 0.0;
        }

        static void tql2(double[,] V, double[] d, double[] e)
        {
            for (int i = 1; i < 3; i++)
            {
                e[i - 1] = e[i];
            }
            e[3 - 1] = 0.0;

            double f = 0.0;
            double tst1 = 0.0;
            double eps = Math.Pow(2.0, -52.0);
            for (int l = 0; l < 3; l++)
            {
                tst1 = Math.Max(tst1, Math.Abs(d[l]) + Math.Abs(e[l]));
                int m = l;

                while (m < 3)//not in original code, was m < 3!!!
                {
                    if (Math.Abs(e[m]) <= eps * tst1)
                    {
                        break;
                    }
                    m++;
                }

                if (m > l)
                {
                    int iter = 0;
                    do
                    {
                        iter = iter + 1;

                        double g = d[l];
                        double p = (d[l + 1] - g) / (2.0 * e[l]);
                        double r = hypot2(p, 1.0);

                        if (p < 0)
                        {
                            r = -r;
                        }

                        d[l] = e[l] / (p + r);
                        d[l + 1] = e[l] * (p + r);

                        double dl1 = d[l + 1];
                        double h = g - d[l];

                        for (int i = l + 2; i < 3; i++)
                        {
                            d[i] -= h;
                        }

                        f = f + h;
                        p = d[m];

                        double c = 1.0;
                        double c2 = c;
                        double c3 = c;
                        double el1 = e[l + 1];
                        double s = 0.0;
                        double s2 = 0.0;

                        for (int i = m - 1; i >= l; i--)
                        {
                            c3 = c2;
                            c2 = c;
                            s2 = s;
                            g = c * e[i];
                            h = c * p;
                            r = hypot2(p, e[i]);
                            e[i + 1] = s * r;
                            s = e[i] / r;
                            c = p / r;
                            p = c * d[i] - s * g;
                            d[i + 1] = h + s * (c * g + s * d[i]);

                            for (int k = 0; k < 3; k++)
                            {
                                h = V[k, i + 1];
                                V[k, i + 1] = s * V[k, i] + c * h;
                                V[k, i] = c * V[k, i] - s * h;
                            }
                        }

                        p = -s * s2 * c3 * el1 * e[l] / dl1;
                        e[l] = s * p;
                        d[l] = c * p;
                    } while (Math.Abs(e[l]) > eps * tst1);
                }

                d[l] = d[l] + f;
                e[l] = 0.0;
            }

            for (int i = 0; i < 3 - 1; i++)
            {
                int k = i;
                double p = d[i];

                for (int j = i + 1; j < 3; j++)
                {
                    if (d[j] < p)
                    {
                        k = j;
                        p = d[j];
                    }
                }
                if (k != i)
                {
                    d[k] = d[i];
                    d[i] = p;

                    for (int j = 0; j < 3; j++)
                    {
                        p = V[j, i];
                        V[j, i] = V[j, k];
                        V[j, k] = p;
                    }
                }
            }
        }

        public void eigen_decomposition(double[,] A, double[,] V, double[] d)
        {
            double[] e = new double[3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    V[i, j] = A[i, j];
                }
            }
            tred2(V, d, e);
            tql2(V, d, e);

            calcDone = true;
        }
    }
}
