using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Deltaforce_final
{
    class J_CalLambda
    {

         double t1 = 0, t3 = 0;
        double head_length = 90;
        public double lambda0, lambda1, lambda2, lambda3;
        public double lambda0_5, lambda3_5;
        public void Calculate(double x, double y, double z, double t2)
        {
            double t = (Math.PI * t2) / 180;

            double d0x, d0y, d0z;
            double d1x, d1y, d1z;
            double d2x, d2y, d2z;
            double d3x, d3y, d3z;
            double d4x, d4y, d4z;
            double d5x, d5y, d5z;



            d0x = x + (56 * Math.Cos(t) - 90 * Math.Sin(t)) - 175;

            d0y = y - 25 + 22.8234292;

            d0z = (z - 90) + (56 * Math.Sin(t) + 90 * Math.Cos(t)) - 264.4508698;



            d1x = x + (56 * Math.Cos(t) - 90 * Math.Sin(t)) - 175;

            d1y = y + 25 - 27.1765708;

            d1z = (z - 90) + (56 * Math.Sin(t) + 90 * Math.Cos(t)) - 264.4508698;



            d2x = x + (25 * Math.Cos(t) - 90 * Math.Sin(t)) - 25;

            d2y = y + 56 - 244.9199235;

            d2z = (z - 90) + (25 * Math.Sin(t) + 90 * Math.Cos(t)) - 264.4508698;



            d3x = x + (-25 * Math.Cos(t) - 90 * Math.Sin(t)) + 25;

            d3y = y + 56 - 244.9199235;

            d3z = (z - 90) + (-25 * Math.Sin(t) + 90 * Math.Cos(t)) - 264.4508698;



            d4x = x + (-56 * Math.Cos(t) - 90 * Math.Sin(t)) + 175;

            d4y = y + 25 - 27.1765708;

            d4z = (z - 90) + (-56 * Math.Sin(t) + 90 * Math.Cos(t)) - 264.4508698;



            d5x = x + (-56 * Math.Cos(t) - 90 * Math.Sin(t)) + 175;

            d5y = y - 25 + 22.8234292;

            d5z = (z - 90) + (-56 * Math.Sin(t) + 90 * Math.Cos(t)) - 264.4508698;





            lambda0   = d0z + Math.Sqrt(84100  - (d0x * d0x) - (d0y * d0y));
           // lambda0_5 = d1z + Math.Sqrt(84100  - (d1x * d1x) - (d1y * d1y));
            lambda1   = d2z + Math.Sqrt(105625 - (d2x * d2x) - (d2y * d2y));
            lambda2   = d3z + Math.Sqrt(105625 - (d3x * d3x) - (d3y * d3y));
            lambda3   = d4z + Math.Sqrt(84100  - (d4x * d4x) - (d4y * d4y));
           // lambda3_5 = d5z + Math.Sqrt(84100  - (d5x * d5x) - (d5y * d5y));

        }
    }
}


