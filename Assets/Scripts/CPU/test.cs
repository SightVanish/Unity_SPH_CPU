using UnityEngine;
using System.Collections;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<double>;


public class test : MonoBehaviour
{
    void Start()
    {
        Matrix A = Matrix.Build.DenseOfArray(new double[,] {
             {1,0,0},
             {0,2,0},
             {0,0,3}});

        Vector b = Vector.Build.Dense(new double[] { 1, 1, 1 });

        Vector x = A.Solve(b);
        Debug.Log("x = A^-1b: " + x);
    }
}
