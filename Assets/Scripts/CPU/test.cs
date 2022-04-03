using UnityEngine;
using System.Collections;
using System.IO;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<double>;



public class test : MonoBehaviour
{
    void Start()
    {
        // matrix computation
        Matrix A = Matrix.Build.DenseOfArray(new double[,] {
             {1,0,0},
             {0,2,0},
             {0,0,3}});

        Vector b = Vector.Build.Dense(new double[] { 1, 1, 1 });

        Vector x = A.Solve(b);
        Debug.Log("x = A^-1b: " + x);

        // write to log.txt and read it again
        using (StreamWriter sw = new StreamWriter("log.txt"))
        {
            sw.Write("Start of file.\n");
            sw.Write(x);
            sw.Write("End of file.");
        }

        string line = "";
        using (StreamReader sr = new StreamReader("log.txt"))
        {
            while ((line = sr.ReadLine()) != null)
            {
                print(line);
            }
        }
    }
}
