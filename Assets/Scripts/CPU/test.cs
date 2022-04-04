using UnityEngine;
using System.Collections;
using System.IO;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MathNet.Numerics.LinearAlgebra.Double;



public class test : MonoBehaviour
{
    void Start()
    {
        // matrix computation
        Matrix p_i = new DenseMatrix(4, 1);
        p_i[0, 0] = 1f; p_i[1, 0] = transform.position.x; p_i[2, 0] = transform.position.y; p_i[3, 0] = transform.position.z;
        print(p_i);
        var x = p_i.Transpose() * p_i;

        print((float)x[0, 0]);


        //Vector b = Vector.Build.Dense(new double[] { 1, 1, 1 });

        //Vector x = A.Solve(b);
        //Debug.Log("x = A^-1b: " + x);

        //// write to log.txt and read it again
        using (StreamWriter sw = new StreamWriter("log.txt"))
        {
            sw.Write(this.gameObject.name);
            sw.Write(transform.position);
        }

        //string line = "";
        //using (StreamReader sr = new StreamReader("log.txt"))
        //{
        //    while ((line = sr.ReadLine()) != null)
        //    {
        //        print(line);
        //    }
        //}
    }
}
